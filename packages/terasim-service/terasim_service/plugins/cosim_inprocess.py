"""Single-process transport for lock-stepped TeraSim/CARLA co-simulation.

TeraSim/SUMO remains on a dedicated simulation thread because TraCI access is
thread-affine.  The CARLA loop exchanges validated Python objects with that
thread through the same request/done rendezvous used by the direct gRPC
plugin, avoiding protobuf, JSON, sockets, and an extra service process.
"""

import os
import time
from dataclasses import dataclass

from terasim.overlay import traci
from terasim.profiling import add_timing, get_profile, set_value
from terasim.simulator import Simulator

from ..utils import AgentCommand
from .cosim import DEFAULT_COSIM_PLUGIN_CONFIG
from .cosim_direct import TeraSimCoSimDirectPlugin


@dataclass(frozen=True)
class InProcessStepResponse:
    status: str
    state: dict | None
    completed_sumo_time: float
    completed_tick_count: int


class _InProcessTickFuture:
    def __init__(self, plugin: "TeraSimCoSimInProcessPlugin", generation: int):
        self._plugin = plugin
        self._generation = generation

    def result(self, timeout=None):
        return self._plugin.wait_for_tick(self._generation, timeout=timeout)


class InProcessLink:
    """CarlaCosim-side adapter with the same interface as DirectLink."""

    transport_name = "inprocess"

    def __init__(self, plugin: "TeraSimCoSimInProcessPlugin", ready_timeout=600.0):
        self._plugin = plugin
        self._plugin.wait_until_ready(timeout=ready_timeout)

    def tick_async(self, commands):
        return self._plugin.request_tick(commands)

    def get_state(self):
        return self._plugin.get_state()

    def stop(self):
        self._plugin.request_stop()

    def close(self):
        pass


class TeraSimCoSimInProcessPlugin(TeraSimCoSimDirectPlugin):
    """Direct-plugin lifecycle with an object mailbox instead of gRPC."""

    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        base_dir: str = "output",
        auto_run: bool = False,
    ):
        super().__init__(
            simulation_uuid,
            plugin_config=plugin_config,
            base_dir=base_dir,
            auto_run=auto_run,
            grpc_host="127.0.0.1",
            grpc_port=0,
        )
        # CARLA only consumes SUMO Cartesian coordinates. Preserve the public
        # state schema, but skip convertGeo unless explicitly requested.
        if not os.getenv("TERASIM_COSIM_STATE_EXPORT_GEO", "").strip():
            self.state_export_geo_enabled = False
        self._state = None
        self._requested_generation = 0
        self._completed_generation = 0
        self._request_in_flight = False

    @staticmethod
    def _simulation_state_to_plain_dict(state):
        """Expose validated Pydantic fields without recursively re-encoding them."""
        return {
            "header": state.header.__dict__,
            "simulation_time": state.simulation_time,
            "agent_count": state.agent_count,
            "agent_details": {
                agent_type: {
                    agent_id: agent_state.__dict__
                    for agent_id, agent_state in agents.items()
                }
                for agent_type, agents in state.agent_details.items()
            },
            "traffic_light_details": {
                signal_id: signal.__dict__
                for signal_id, signal in state.traffic_light_details.items()
            },
            "construction_zone_details": state.construction_zone_details,
            "construction_objects": {
                object_id: object_state.__dict__
                for object_id, object_state in state.construction_objects.items()
            },
        }

    def function_before_env_start(self, simulator: Simulator, ctx):
        self._set_status("initializing")
        self.logger.info(
            "TeraSim in-process co-simulation initializing. Simulation UUID: %s",
            self.simulation_uuid,
        )
        return True

    def function_after_env_start(self, simulator: Simulator, ctx):
        try:
            expected_delta = float(os.getenv("COSIM_EXPECTED_STEP_LENGTH", "0.05"))
            actual_delta = float(traci.simulation.getDeltaT())
            if abs(actual_delta - expected_delta) > 1e-9:
                self.logger.error(
                    "SUMO deltaT=%s does not match expected %s",
                    actual_delta,
                    expected_delta,
                )
                self._finish("error")
                return False
            state = self._build_simulation_state(simulator)
            with self._lock:
                self._state = self._simulation_state_to_plain_dict(state)
                self._status = "wait_for_tick"
            self.logger.info("TeraSim in-process co-simulation ready")
            return True
        except Exception as exc:
            self.logger.exception("In-process initialization failed: %s", exc)
            self._finish("error")
            return False

    def function_after_env_step(self, simulator: Simulator, ctx):
        state_export_start = time.perf_counter()
        set_value(
            ctx,
            "terasim_internal.sumo_total_vehicle_count",
            traci.vehicle.getIDCount(),
        )
        try:
            state = self._build_simulation_state(simulator)
        except Exception as exc:
            self.logger.exception("State build failed, stopping simulation: %s", exc)
            self._finish("error")
            return False
        add_timing(
            ctx,
            "terasim_internal.state_export.total_s",
            time.perf_counter() - state_export_start,
        )

        conversion_start = time.perf_counter()
        state_dict = self._simulation_state_to_plain_dict(state)
        add_timing(
            ctx,
            "terasim_internal.state_export.serialization_inprocess_s",
            time.perf_counter() - conversion_start,
        )
        step_start = ctx.get("_cosim_profile_step_start_perf")
        if isinstance(step_start, (int, float)):
            set_value(ctx, "terasim_internal.total_s", time.perf_counter() - step_start)

        completed_sumo_time = traci.simulation.getTime()
        completed_tick_count = self._completed_tick_count + 1
        profile = get_profile(ctx)
        self._write_internal_profile(completed_tick_count, completed_sumo_time, profile)
        with self._lock:
            self._state = state_dict
            self._completed_sumo_time = completed_sumo_time
            self._completed_tick_count = completed_tick_count
            self._completed_generation = self._requested_generation
            self._request_in_flight = False
            self._status = "ticked"
        self._step_done.set()
        return True

    def function_after_env_stop(self, simulator: Simulator, ctx):
        with self._lock:
            failed = self._status == "error"
        if not failed:
            self._finish("finished")
        self.logger.info("TeraSim in-process simulation %s finished", self.simulation_uuid)

    def _response(self):
        with self._lock:
            return InProcessStepResponse(
                status=self._status,
                state=self._state,
                completed_sumo_time=self._completed_sumo_time,
                completed_tick_count=self._completed_tick_count,
            )

    def wait_until_ready(self, timeout=600.0):
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                status = self._status
            if status in {"wait_for_tick", "ticked", "running"}:
                return
            if status in {"finished", "error"}:
                raise RuntimeError(f"TeraSim ended during initialization: {status}")
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"TeraSim in-process link not ready within {timeout:.1f}s"
                )
            time.sleep(0.01)

    def request_tick(self, commands):
        validated_commands = [
            command
            if isinstance(command, AgentCommand)
            else AgentCommand.model_validate(command)
            for command in commands
        ]
        with self._lock:
            if self._status in {"finished", "error"} or self._stop_requested:
                return _InProcessTickFuture(self, self._completed_generation)
            if self._request_in_flight:
                raise RuntimeError("Only one in-process SUMO tick may be in flight")
            self._pending_commands = validated_commands
            self._requested_generation += 1
            generation = self._requested_generation
            self._request_in_flight = True
            self._step_done.clear()
        self._tick_requested.set()
        return _InProcessTickFuture(self, generation)

    def wait_for_tick(self, generation, timeout=None):
        effective_timeout = self.STEP_TIMEOUT_S if timeout is None else timeout
        if not self._step_done.wait(timeout=effective_timeout):
            raise TimeoutError(
                f"In-process SUMO tick {generation} timed out after "
                f"{effective_timeout:.1f}s"
            )
        with self._lock:
            if (
                self._completed_generation < generation
                and self._status not in {"finished", "error"}
            ):
                raise RuntimeError(
                    f"Stale in-process completion: requested={generation} "
                    f"completed={self._completed_generation}"
                )
        return self._response()

    def get_state(self):
        return self._response()

    def request_stop(self):
        with self._lock:
            self._stop_requested = True
            self._request_in_flight = False
        # Unblock both a simulation thread waiting for a request and a CARLA
        # thread waiting for completion.
        self._tick_requested.set()
        self._step_done.set()
