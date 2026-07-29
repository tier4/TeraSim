"""TeraSimCoSimDirectPlugin: Redis/FastAPI-free co-simulation plugin.

Serves the CosimDirect gRPC contract (direct/cosim_direct.proto) from inside
the TeraSim process, replacing the Redis + FastAPI relay of TeraSimCoSimPlugin:

  Redis "control" key polling      -> threading.Event set by the Tick RPC
  Redis "agent_commands" list      -> commands carried inside the Tick RPC
  Redis "state"/"status"/counters  -> TickResponse / GetState RPC fields

One Tick RPC = apply the request's agent commands, run exactly one SUMO step,
and return the post-step state. The gRPC handler thread and the simulation
main loop rendezvous through two events (_tick_requested / _step_done).

Assumes a single co-sim client (the 3-cosim CarlaCosim); concurrent Tick
calls are serialized and would each consume one step.

State construction is inherited from TeraSimCoSimPlugin
(_build_simulation_state), as is agent command handling
(_handle_agent_command), so both transports share one implementation.
"""

import json
import os
import threading
import time
from concurrent import futures

import grpc

from terasim.overlay import traci
from terasim.profiling import add_timing, get_profile, set_value
from terasim.simulator import Simulator

from ..direct import cosim_direct_pb2, cosim_direct_pb2_grpc
from .cosim import DEFAULT_COSIM_PLUGIN_CONFIG, TeraSimCoSimPlugin


class _CosimDirectServicer(cosim_direct_pb2_grpc.CosimDirectServicer):
    """Thin delegation layer so the plugin object owns all the state."""

    def __init__(self, plugin: "TeraSimCoSimDirectPlugin"):
        self._plugin = plugin

    def Tick(self, request, context):
        return self._plugin.rpc_tick(request, context)

    def GetState(self, request, context):
        return self._plugin.rpc_get_state(request, context)

    def Stop(self, request, context):
        return self._plugin.rpc_stop(request, context)


class TeraSimCoSimDirectPlugin(TeraSimCoSimPlugin):
    """Drop-in variant of TeraSimCoSimPlugin that talks gRPC instead of Redis."""

    # Longest time function_before_env_step keeps waiting for a Tick RPC before
    # auto-stopping (same idle safety net as the Redis plugin).
    IDLE_TIMEOUT_S = 600.0
    # Longest time a Tick RPC waits for the SUMO step to complete. Generous:
    # odaiba steps are ~100 ms, but map loading hiccups should not kill the link.
    STEP_TIMEOUT_S = 300.0

    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        base_dir: str = "output",
        auto_run: bool = False,
        grpc_host: str = "127.0.0.1",
        grpc_port: int = 8200,
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
            # auto_run would advance SUMO without Tick RPCs; the direct
            # contract is strictly lock-stepped, so reject it early.
            raise ValueError("TeraSimCoSimDirectPlugin requires auto_run=False")
        self.grpc_host = grpc_host
        self.grpc_port = grpc_port
        self.profile_steps_enabled = self._parse_bool_env(
            "TERASIM_COSIM_PROFILE_STEPS",
            self._parse_bool_env("COSIM_PROFILE_STEPS", False),
        )
        self.state_filter_max_vehicles = max(
            0, self._parse_int_env("TERASIM_COSIM_STATE_MAX_VEHICLES", 0)
        )
        self.profile_jsonl_path = os.getenv("TERASIM_COSIM_PROFILE_JSONL", "").strip()

        self._lock = threading.Lock()
        self._status = "created"
        self._state_json = ""
        self._completed_sumo_time = 0.0
        self._completed_tick_count = 0
        self._pending_commands = []  # raw JSON bytes, same wire format as the Redis list
        self._stop_requested = False
        self._tick_requested = threading.Event()
        self._step_done = threading.Event()
        self._rpc_serial = threading.Lock()  # serialize concurrent Tick calls
        self._grpc_server = None

    def _filter_vehicle_ids_for_state(self, vehicle_ids):
        filtered_ids, position_cache = super()._filter_vehicle_ids_for_state(vehicle_ids)
        max_vehicles = self.state_filter_max_vehicles
        if max_vehicles <= 0 or len(filtered_ids) <= max_vehicles:
            return filtered_ids, position_cache

        center_id = self.state_filter_center_id
        center_position = position_cache.get(center_id)
        if center_position is None and center_id in filtered_ids:
            center_position = traci.vehicle.getPosition3D(center_id)
            position_cache[center_id] = center_position

        if center_position is None:
            selected_ids = sorted(filtered_ids)[:max_vehicles]
        else:
            candidates = []
            for vehicle_id in filtered_ids:
                position = position_cache.get(vehicle_id)
                if position is None:
                    position = traci.vehicle.getPosition3D(vehicle_id)
                    position_cache[vehicle_id] = position
                dx = position[0] - center_position[0]
                dy = position[1] - center_position[1]
                candidates.append((dx * dx + dy * dy, vehicle_id))
            candidates.sort(key=lambda item: (item[0], item[1]))
            selected_ids = [vehicle_id for _, vehicle_id in candidates[:max_vehicles]]
        return selected_ids, position_cache

    def _write_internal_profile(self, completed_tick_count, completed_sumo_time, profile):
        if not self.profile_jsonl_path or not isinstance(profile, dict):
            return
        profile_dir = os.path.dirname(self.profile_jsonl_path)
        if profile_dir:
            os.makedirs(profile_dir, exist_ok=True)
        record = {
            "completed_tick_count": completed_tick_count,
            "simulation_time": completed_sumo_time,
            "profile": profile,
        }
        with open(self.profile_jsonl_path, "a", encoding="utf-8") as profile_file:
            profile_file.write(json.dumps(record, separators=(",", ":")) + "\n")

    # ------------------------------------------------------------------
    # lifecycle hooks (replace the Redis I/O of the parent class)
    # ------------------------------------------------------------------
    def function_before_env_start(self, simulator: Simulator, ctx):
        try:
            self._set_status("initializing")
            self._grpc_server = grpc.server(
                futures.ThreadPoolExecutor(max_workers=4),
                options=[
                    ("grpc.max_send_message_length", 128 * 1024 * 1024),
                    ("grpc.max_receive_message_length", 128 * 1024 * 1024),
                ],
            )
            cosim_direct_pb2_grpc.add_CosimDirectServicer_to_server(
                _CosimDirectServicer(self), self._grpc_server
            )
            bound_port = self._grpc_server.add_insecure_port(
                f"{self.grpc_host}:{self.grpc_port}"
            )
            if bound_port == 0:
                self.logger.error(
                    f"Failed to bind gRPC server on {self.grpc_host}:{self.grpc_port}"
                )
                return False
            self._grpc_server.start()
            self.logger.info(
                f"CosimDirect gRPC server listening on {self.grpc_host}:{self.grpc_port}. "
                f"Simulation UUID: {self.simulation_uuid}, start initialization!"
            )
            return True
        except Exception as e:
            self.logger.exception(f"Unexpected error during initialization: {e}")
            return False

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
                return False
            self.logger.info("Validated SUMO deltaT=%ss", actual_delta)
            # Build an initial state so GetState has data (e.g. traffic lights)
            # before the first Tick, mirroring the idle state writes of the
            # Redis plugin.
            try:
                state = self._build_simulation_state(simulator)
                with self._lock:
                    self._state_json = state.model_dump_json()
            except Exception as e:
                self.logger.warning(f"Initial state build failed (non-fatal): {e}")
            self._set_status("wait_for_tick")
            self.logger.info(
                f"Simulation UUID: {self.simulation_uuid}, finish initialization!"
            )
            return True
        except Exception as e:
            self.logger.exception(f"Unexpected error after start: {e}")
            return False

    def function_before_env_step(self, simulator: Simulator, ctx):
        idle_start = time.time()
        while True:
            if self._stop_requested:
                self.logger.info("Stopping simulation")
                simulator.running = False
                return False
            if time.time() - idle_start > self.IDLE_TIMEOUT_S:
                self.logger.warning("No Tick for %.0fs, auto-stopping", self.IDLE_TIMEOUT_S)
                simulator.running = False
                return False
            if self._tick_requested.wait(timeout=0.1):
                self._tick_requested.clear()
                break

        if getattr(self, "profile_steps_enabled", False):
            ctx["cosim_profile"] = {"terasim_internal": {}}
            ctx["_cosim_profile_step_start_perf"] = time.perf_counter()
            simulator.ctx = ctx
        command_ingestion_start = time.perf_counter()

        with self._lock:
            commands = self._pending_commands
            self._pending_commands = []
        self.controlled_agents_each_step.clear()
        self._active_agent_command_profile_ctx = ctx
        try:
            for raw in commands:
                if not self._handle_agent_command(raw):
                    if self._should_continue_after_agent_command_failure():
                        continue
                    self._record_agent_command_failure(simulator)
                    self._finish("error")
                    simulator.running = False
                    self.logger.critical(
                        "Fail-closed: direct Tick rejected before SUMO simulationStep"
                    )
                    return False
        finally:
            self._active_agent_command_profile_ctx = None

        add_timing(
            ctx,
            "terasim_internal.pre_step_command_ingestion_s",
            time.perf_counter() - command_ingestion_start,
        )
        self._set_status("running")
        self.logger.info("Simulation step started")
        return True

    def function_after_env_step(self, simulator: Simulator, ctx):
        state_export_start = time.perf_counter()
        set_value(
            ctx,
            "terasim_internal.sumo_total_vehicle_count",
            traci.vehicle.getIDCount(),
        )
        try:
            state = self._build_simulation_state(simulator)
        except Exception as e:
            self.logger.exception(f"State build failed, stopping simulation: {e}")
            self._finish("error")
            return False
        add_timing(
            ctx,
            "terasim_internal.state_export.total_s",
            time.perf_counter() - state_export_start,
        )

        serialization_start = time.perf_counter()
        state_json = state.model_dump_json()
        add_timing(
            ctx,
            "terasim_internal.state_export.serialization_grpc_response_s",
            time.perf_counter() - serialization_start,
        )
        step_start = ctx.get("_cosim_profile_step_start_perf")
        if isinstance(step_start, (int, float)):
            set_value(
                ctx,
                "terasim_internal.total_s",
                time.perf_counter() - step_start,
            )

        completed_sumo_time = traci.simulation.getTime()
        completed_tick_count = self._completed_tick_count + 1
        profile = get_profile(ctx)
        self._write_internal_profile(completed_tick_count, completed_sumo_time, profile)
        with self._lock:
            self._state_json = state_json
            self._completed_sumo_time = completed_sumo_time
            self._completed_tick_count = completed_tick_count
            self._status = "ticked"
        self._step_done.set()
        self.logger.info(
            "Simulation step finished! completed_sumo_time=%s completed_tick_count=%s",
            completed_sumo_time,
            completed_tick_count,
        )
        return True

    def function_before_env_stop(self, simulator: Simulator, ctx):
        pass

    def function_after_env_stop(self, simulator: Simulator, ctx):
        with self._lock:
            failed = self._status == "error"
        if not failed:
            self._finish("finished")
        if self._grpc_server is not None:
            # Let in-flight RPCs drain (they will see status="finished").
            self._grpc_server.stop(grace=2.0)
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
                self._state_json = state.model_dump_json()
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
        # Release any Tick RPC waiting on the step that will never come.
        self._step_done.set()

    def _response(self) -> cosim_direct_pb2.TickResponse:
        with self._lock:
            return cosim_direct_pb2.TickResponse(
                status=self._status,
                state_json=self._state_json,
                completed_sumo_time=self._completed_sumo_time,
                completed_tick_count=self._completed_tick_count,
            )

    # ------------------------------------------------------------------
    # RPC implementations (called from gRPC handler threads)
    # ------------------------------------------------------------------
    def rpc_tick(self, request, context):
        with self._rpc_serial:
            with self._lock:
                if self._status in ("finished", "error") or self._stop_requested:
                    pass_through = True
                else:
                    pass_through = False
                    self._pending_commands = [
                        json.dumps(
                            {
                                "agent_id": c.agent_id,
                                "agent_type": c.agent_type,
                                "command_type": c.command_type,
                                "data": json.loads(c.data_json) if c.data_json else {},
                            }
                        ).encode("utf-8")
                        for c in request.commands
                    ]
            if pass_through:
                return self._response()

            self._step_done.clear()
            self._tick_requested.set()
            if not self._step_done.wait(timeout=self.STEP_TIMEOUT_S):
                self.logger.error(
                    "Tick RPC timed out after %.0fs waiting for the SUMO step",
                    self.STEP_TIMEOUT_S,
                )
            return self._response()

    def rpc_get_state(self, request, context):
        return self._response()

    def rpc_stop(self, request, context):
        self.logger.info("Stop requested over gRPC")
        self._stop_requested = True
        return cosim_direct_pb2.StopResponse(ok=True)
