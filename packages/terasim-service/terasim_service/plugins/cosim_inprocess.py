"""TeraSimCoSimInProcessPlugin: co-simulation plugin for the single-process link.

Replaces the transports of the earlier co-sim stages (Redis lists polled by a
FastAPI service, then gRPC RPCs between two processes) with plain
thread-to-thread rendezvous: the CARLA-facing co-sim loop (client thread) and
the TeraSim simulation loop (sim thread) live in ONE process and exchange
commands/state as Python objects.

  Redis "control" key polling / Tick RPC   -> tick_async() + threading.Event
  Redis "agent_commands" list / RPC field  -> AgentCommand objects (no JSON)
  Redis "state" keys / RPC state_json      -> TickResult.state (dict, no JSON)

One tick_async() call = deliver this step's agent commands, run exactly one
SUMO step, publish the post-step state. The client thread and the sim loop
rendezvous through two events (_tick_requested / _step_done).

Threading contract: a SINGLE co-sim client thread, calling
tick_async() -> handle.result() strictly in that order (the next tick_async
only after the previous handle resolved). The published state dict is a fresh
snapshot each step and is never mutated by the plugin afterwards.

State construction is inherited from TeraSimCoSimPlugin
(_build_simulation_state), as is agent command application
(_apply_agent_command), so all transports share one implementation.
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional

from terasim.overlay import traci
from terasim.simulator import Simulator

from ..utils import AgentCommand
from .cosim import DEFAULT_COSIM_PLUGIN_CONFIG, TeraSimCoSimPlugin


@dataclass
class TickResult:
    """Snapshot of the co-sim state after a step (or at rest)."""

    status: str
    state: Optional[dict]  # SimulationState.model_dump(); None before the first build
    completed_sumo_time: float
    completed_tick_count: int


class TickHandle:
    """Future-like handle for one requested SUMO step.

    result() blocks until the sim thread finishes that step (or the
    simulation ends) and returns the post-step TickResult.
    """

    def __init__(self, plugin: "TeraSimCoSimInProcessPlugin", resolved: Optional[TickResult] = None):
        self._plugin = plugin
        self._resolved = resolved  # pre-resolved for pass-through (ended) calls

    def result(self, timeout: float = 300.0) -> TickResult:
        if self._resolved is not None:
            return self._resolved
        if not self._plugin._step_done.wait(timeout=timeout):
            raise TimeoutError(
                f"SUMO step did not complete within {timeout:.0f}s"
            )
        return self._plugin.get_result()


class TeraSimCoSimInProcessPlugin(TeraSimCoSimPlugin):
    """Variant of TeraSimCoSimPlugin driven by a same-process co-sim client."""

    # Longest time function_before_env_step keeps waiting for a tick request
    # before auto-stopping (same idle safety net as the Redis plugin).
    IDLE_TIMEOUT_S = 600.0

    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        base_dir: str = "output",
        auto_run: bool = False,
    ):
        # Parent __init__ only parses config/env and sets up logging; it does
        # NOT connect to Redis (that happens in its function_before_env_start,
        # which this class overrides), so inheriting it is safe.
        super().__init__(
            simulation_uuid,
            plugin_config=plugin_config,
            base_dir=base_dir,
            auto_run=auto_run,
        )
        if auto_run:
            # auto_run would advance SUMO without tick requests; this link is
            # strictly lock-stepped, so reject it early.
            raise ValueError("TeraSimCoSimInProcessPlugin requires auto_run=False")

        self._lock = threading.Lock()
        self._status = "created"
        self._state = None  # dict (SimulationState.model_dump())
        self._completed_sumo_time = 0.0
        self._completed_tick_count = 0
        self._pending_commands = []  # list[AgentCommand]
        self._stop_requested = False
        self._ready = threading.Event()  # set once wait_for_tick is reached (or startup failed)
        self._tick_requested = threading.Event()
        self._step_done = threading.Event()
        self._client_serial = threading.Lock()  # serialize concurrent tick_async calls

    # ------------------------------------------------------------------
    # client-side API (called from the co-sim client thread)
    # ------------------------------------------------------------------
    def wait_until_ready(self, timeout: float) -> bool:
        """Block until the simulation reaches wait_for_tick (SUMO loaded).

        Returns True when the plugin is ready for tick_async; False on
        timeout or when the simulation already ended during startup.
        """
        if not self._ready.wait(timeout=timeout):
            return False
        with self._lock:
            return self._status == "wait_for_tick"

    def tick_async(self, commands) -> TickHandle:
        """Request one SUMO step (non-blocking).

        commands: list of dicts {agent_id, agent_type, command_type, data}
        (same shape the earlier transports carried as JSON). Returns a
        TickHandle whose .result(timeout) yields the post-step TickResult.
        """
        with self._client_serial:
            with self._lock:
                if self._status in ("finished", "error") or self._stop_requested:
                    return TickHandle(self, resolved=self._result_locked())
                self._pending_commands = [
                    AgentCommand.model_validate(c) for c in commands
                ]
            self._step_done.clear()
            self._tick_requested.set()
            return TickHandle(self)

    def get_result(self) -> TickResult:
        """Fetch the latest state without advancing the simulation."""
        with self._lock:
            return self._result_locked()

    def request_stop(self):
        """Ask the simulation loop to stop (idempotent, thread-safe)."""
        self.logger.info("Stop requested by the co-sim client")
        self._stop_requested = True

    def abort(self, status: str = "error"):
        """Mark the simulation as ended on behalf of a dead sim thread.

        Called by the runner when sim.run() raises: releases a client blocked
        in wait_until_ready()/result() so the process can shut down.
        """
        self._finish(status)
        self._ready.set()

    # ------------------------------------------------------------------
    # lifecycle hooks (replace the Redis I/O of the parent class)
    # ------------------------------------------------------------------
    def function_before_env_start(self, simulator: Simulator, ctx):
        self._set_status("initializing")
        self.logger.info(
            f"Simulation UUID: {self.simulation_uuid}, start initialization!"
        )
        return True

    def function_after_env_start(self, simulator: Simulator, ctx):
        try:
            # Build an initial state so the client can seed its render
            # pipeline (e.g. AV shape init) before the first tick, mirroring
            # the idle state writes of the Redis plugin.
            try:
                state = self._build_simulation_state(simulator)
                with self._lock:
                    self._state = state.model_dump()
            except Exception as e:
                self.logger.warning(f"Initial state build failed (non-fatal): {e}")
            self._set_status("wait_for_tick")
            self._ready.set()
            self.logger.info(
                f"Simulation UUID: {self.simulation_uuid}, finish initialization!"
            )
            return True
        except Exception as e:
            self.logger.exception(f"Unexpected error after start: {e}")
            self.abort("error")
            return False

    def function_before_env_step(self, simulator: Simulator, ctx):
        idle_start = time.time()
        while True:
            if self._stop_requested:
                self.logger.info("Stopping simulation")
                simulator.running = False  # stop the main loop, like the Redis-era stop command
                return False
            if time.time() - idle_start > self.IDLE_TIMEOUT_S:
                self.logger.warning("No tick request for %.0fs, auto-stopping", self.IDLE_TIMEOUT_S)
                simulator.running = False
                return False
            if self._tick_requested.wait(timeout=0.1):
                self._tick_requested.clear()
                break

        # Apply the commands delivered with this tick request (shared
        # implementation with the Redis path via _apply_agent_command).
        with self._lock:
            commands = self._pending_commands
            self._pending_commands = []
        self.controlled_agents_each_step.clear()
        for command in commands:
            self._apply_agent_command(command)

        self._set_status("running")
        self.logger.info("Simulation step started")
        return True

    def function_after_env_step(self, simulator: Simulator, ctx):
        try:
            state = self._build_simulation_state(simulator)
        except Exception as e:
            self.logger.exception(f"State build failed, stopping simulation: {e}")
            self._finish("error")
            return False
        completed_sumo_time = traci.simulation.getTime()
        with self._lock:
            self._state = state.model_dump()
            self._completed_sumo_time = completed_sumo_time
            self._completed_tick_count += 1
            self._status = "ticked"
            completed_tick_count = self._completed_tick_count
        self._step_done.set()
        # One line per step on purpose: with the RPC observation endpoint
        # gone, this log line (console handler prints asctime) is the external
        # interface for step-rate / clock-ratio / vehicle-count measurement.
        # vehicles= is the TOTAL SUMO vehicle count (the measurement x-axis; it
        # must not shrink when TERASIM_COSIM_STATE_FILTER trims the published
        # state); vehicles_state= is what actually went into the state.
        try:
            state_vehicle_count = state.agent_count.get("vehicle", -1)
        except Exception:
            state_vehicle_count = -1
        try:
            total_vehicle_count = traci.vehicle.getIDCount()
        except Exception:
            total_vehicle_count = state_vehicle_count
        self.logger.info(
            "Simulation step finished! completed_sumo_time=%s completed_tick_count=%s "
            "vehicles=%s vehicles_state=%s",
            completed_sumo_time,
            completed_tick_count,
            total_vehicle_count,
            state_vehicle_count,
        )
        return True

    def function_before_env_stop(self, simulator: Simulator, ctx):
        pass

    def function_after_env_stop(self, simulator: Simulator, ctx):
        self._finish("finished")
        self.logger.info(f"Simulation {self.simulation_uuid} finished!")

    # ------------------------------------------------------------------
    # Redis-era helpers that must not touch Redis in this class
    # ------------------------------------------------------------------
    def _check_simulation_status(self) -> bool:
        return not self._stop_requested

    def _write_simulation_state(self, simulator):
        # Not used by this class (state is published in function_after_env_step),
        # but keep a safe implementation in case shared code calls it.
        try:
            state = self._build_simulation_state(simulator)
            with self._lock:
                self._state = state.model_dump()
            return True
        except Exception as e:
            self.logger.error(f"State build failed: {e}")
            return False

    # ------------------------------------------------------------------
    # internal state helpers
    # ------------------------------------------------------------------
    def _set_status(self, status: str):
        with self._lock:
            self._status = status

    def _finish(self, status: str):
        with self._lock:
            self._status = status
        # Release a client waiting on a step that will never come.
        self._step_done.set()

    def _result_locked(self) -> TickResult:
        return TickResult(
            status=self._status,
            state=self._state,
            completed_sumo_time=self._completed_sumo_time,
            completed_tick_count=self._completed_tick_count,
        )
