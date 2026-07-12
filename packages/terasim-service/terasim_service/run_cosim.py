"""Single-process 3-cosim runner: TeraSim and the CARLA co-sim client in ONE process.

Replaces the two-process direct link (terasim_service.run_direct +
examples/scripts/carla_cosim_3cosim.py talking gRPC): the TeraSim simulation
loop runs on a background thread, the CARLA-facing co-sim loop runs on the
main thread, and the two rendezvous through TeraSimCoSimInProcessPlugin
(plain function calls + threading events - no Redis/HTTP/gRPC and no JSON
serialization of states or commands).

3-cosim passive mode is preserved: the psim bridge (autoware_carla_interface)
owns world.tick(); this process only follows via world.wait_for_tick() and
mirrors TeraSim/SUMO background traffic into CARLA. The psim ego
(role_name="ego_vehicle") is protected from cleanup and its pose is fed back
to the SUMO "AV" so background traffic avoids it.

Usage:
  python -m terasim_service.run_cosim \
      --config examples/scenarios/cosim_town01.yaml --carla_port 2013
"""

import argparse
import signal
import sys
import threading
import uuid
from pathlib import Path

from loguru import logger

from .plugins.cosim import DEFAULT_COSIM_PLUGIN_CONFIG
from .plugins.cosim_inprocess import TeraSimCoSimInProcessPlugin
from .utils.base import (
    create_environment,
    create_simulator,
    load_config,
    resolve_config_paths,
    set_random_seed,
)


def _run_simulation(sim, plugin):
    """sim.run() wrapper for the sim thread: report a crash instead of hanging the client."""
    try:
        sim.run()
    except Exception:
        logger.exception("[run_cosim] simulation thread crashed")
        plugin.abort("error")


def main():
    p = argparse.ArgumentParser(
        description="Run TeraSim + CARLA co-sim client in one process (no Redis/gRPC)"
    )
    p.add_argument("--config", required=True, help="TeraSim scenario yaml path")
    p.add_argument("--ready_timeout", default=600.0, type=float,
                   help="max seconds to wait for the SUMO network to load")
    # CARLA client options (mirror examples/scripts/carla_cosim_3cosim.py)
    p.add_argument("--carla_host", default="127.0.0.1")
    p.add_argument("--carla_port", default=2013, type=int,
                   help="psim CARLA RPC port (default 2013)")
    p.add_argument("--carla_timeout", default=600.0, type=float,
                   help="CARLA client timeout; large to tolerate procedurally-generated "
                        "worlds where get_map().to_opendrive() may take time")
    p.add_argument("-s", "--step_length", default=0.05, type=float)
    p.add_argument("--map_name", default="", type=str,
                   help="leave empty: psim already loaded the world; do NOT reload")
    p.add_argument("--sync_tls", action="store_true",
                   help="also sync SUMO traffic lights -> CARLA (off by default)")
    p.add_argument("--protected_roles", nargs="+", default=["AV", "ego_vehicle"],
                   help="CARLA role_names never destroyed by cleanup (psim ego = ego_vehicle)")
    p.add_argument("--av_carla_role", default="ego_vehicle",
                   help="CARLA role_name whose pose is fed back to the SUMO AV")
    args = p.parse_args()

    # 3-cosim passive mode: fixed flags consumed by CarlaCosim.tick()/_cleanup_actors()/close()
    # (same set carla_cosim_3cosim.py pinned for the two-process paths).
    args.passive_tick = True   # follow psim's world.tick() via wait_for_tick(); never tick here
    args.async_mode = False
    args.skip_tls = not args.sync_tls
    args.control_av = True     # feed the Autoware ego pose back to the SUMO AV
    args.terasim_config = args.config  # CarlaCosim reads the SUMO net path from the scenario yaml

    # --- build the TeraSim simulation exactly like run_experiments/api ---
    config = load_config(args.config)
    config = resolve_config_paths(config, args.config)
    simulation_id = str(uuid.uuid4())

    base_dir = (
        Path(config["output"]["dir"])
        / config["output"]["name"]
        / "raw_data"
        / config["output"]["nth"]
        / simulation_id
    )
    base_dir.mkdir(parents=True, exist_ok=True)

    set_random_seed(config["seed"])

    env = create_environment(config, base_dir)
    sim = create_simulator(config, base_dir)
    sim.bind_env(env)

    plugin_config = dict(DEFAULT_COSIM_PLUGIN_CONFIG)
    plugin_config["centered_agent_ID"] = "AV"
    plugin = TeraSimCoSimInProcessPlugin(
        simulation_uuid=simulation_id,
        plugin_config=plugin_config,
        base_dir=str(base_dir),
        auto_run=False,
    )
    plugin.inject(sim, {})

    # docker stop sends SIGTERM; fold it into the KeyboardInterrupt path so the
    # finally block below stops TeraSim and cleans up the CARLA actors.
    def _sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sigterm)

    logger.info(
        f"[run_cosim] simulation_id={simulation_id} output={base_dir} "
        f"carla={args.carla_host}:{args.carla_port}"
    )

    sim_thread = threading.Thread(
        target=_run_simulation, args=(sim, plugin), name="terasim-sim", daemon=True
    )
    sim_thread.start()

    logger.info("[run_cosim] waiting for the SUMO network to load ...")
    if not plugin.wait_until_ready(timeout=args.ready_timeout):
        logger.error(
            f"[run_cosim] TeraSim did not reach wait_for_tick within "
            f"{args.ready_timeout:.0f}s (status={plugin.get_result().status}); exiting"
        )
        plugin.request_stop()
        sim_thread.join(timeout=30)
        return 1

    # Import here so terasim_service stays importable without the carla wheel
    # (only this runner needs it).
    from .utils.carla import CarlaCosim

    args.inprocess_plugin = plugin
    carla_cosim = CarlaCosim(args)
    # IMPORTANT: do NOT apply world settings here. The psim bridge owns
    # synchronous_mode and fixed_delta_seconds.

    exit_code = 0
    try:
        while carla_cosim.tick():
            pass
    except KeyboardInterrupt:
        logger.info("[run_cosim] cancelled by user")
    except Exception:
        logger.exception("[run_cosim] co-sim loop crashed")
        exit_code = 1
    finally:
        logger.info("[run_cosim] cleaning up (passive: keep ego, do not reset CARLA settings)")
        carla_cosim.close()  # stops the plugin, so the sim thread can exit
        sim_thread.join(timeout=60)
        if sim_thread.is_alive():
            logger.warning("[run_cosim] simulation thread did not exit within 60s")
        logger.info(f"[run_cosim] simulation_id={simulation_id} exited")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
