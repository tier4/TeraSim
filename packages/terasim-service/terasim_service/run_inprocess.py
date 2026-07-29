"""Run TeraSim/SUMO and CarlaCosim in one Python process.

SUMO remains on a dedicated thread and is advanced only after the CARLA
thread submits the current frame's feedback commands.
"""

import argparse
import os
import signal
import sys
import threading
import uuid
from pathlib import Path

import carla
from loguru import logger

from .plugins.cosim import DEFAULT_COSIM_PLUGIN_CONFIG
from .plugins.cosim_inprocess import InProcessLink, TeraSimCoSimInProcessPlugin
from .utils.base import (
    create_environment,
    create_simulator,
    load_config,
    resolve_config_paths,
    set_random_seed,
)
from .utils.carla import CarlaCosim


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Run lock-stepped TeraSim/CARLA co-simulation in one process"
    )
    parser.add_argument("--config", required=True, help="scenario YAML path")
    parser.add_argument("--carla_host", default="127.0.0.1")
    parser.add_argument("--carla_port", type=int, default=2000)
    parser.add_argument("--carla_timeout", type=float, default=600.0)
    parser.add_argument("--step_length", type=float, default=0.05)
    parser.add_argument("--map_name", default="")
    parser.add_argument(
        "--vehicle_control_mode",
        choices=["teleport", "ackermann_physics"],
        default=os.getenv("CARLA_COSIM_VEHICLE_CONTROL_MODE", "teleport"),
    )
    parser.add_argument("--control_av", action="store_true")
    parser.add_argument("--async_mode", action="store_true")
    return parser


def _configure_carla_world(cosim, step_length, async_mode):
    settings = cosim.world.get_settings()
    if async_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
    else:
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = step_length
    cosim.world.apply_settings(settings)
    if not async_mode:
        current = cosim.world.get_settings()
        if not current.synchronous_mode:
            raise RuntimeError("Failed to enable CARLA synchronous mode")
    cosim.world.set_weather(carla.WeatherParameters.WetSunset)


def main():
    args = _build_parser().parse_args()
    if args.async_mode and os.getenv(
        "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE", "off"
    ).strip().lower() == "apply":
        raise ValueError("Ackermann feedback apply mode requires synchronous CARLA")

    switch_interval = float(os.getenv("TERASIM_INPROCESS_GIL_INTERVAL", "0.001"))
    sys.setswitchinterval(max(0.0001, switch_interval))

    config = resolve_config_paths(load_config(args.config), args.config)
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
    simulator = create_simulator(config, base_dir)
    simulator.bind_env(env)

    plugin_config = dict(DEFAULT_COSIM_PLUGIN_CONFIG)
    plugin_config["centered_agent_ID"] = "AV"
    plugin = TeraSimCoSimInProcessPlugin(
        simulation_uuid=simulation_id,
        plugin_config=plugin_config,
        base_dir=str(base_dir),
        auto_run=False,
    )
    plugin.inject(simulator, {})

    simulation_thread = threading.Thread(
        target=simulator.run,
        name="terasim-sumo",
        daemon=False,
    )
    simulation_thread.start()

    link = None
    carla_cosim = None
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def handle_termination(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, handle_termination)
    try:
        link = InProcessLink(plugin)
        # Fields consumed by CarlaCosim's shared HTTP/gRPC/in-process path.
        args.inprocess_link = link
        args.direct_addr = None
        args.terasim_config = args.config
        args.terasim_host = "inprocess"
        args.terasim_port = 0
        args.passive_tick = False
        args.skip_tls = False

        carla_cosim = CarlaCosim(args)
        _configure_carla_world(carla_cosim, args.step_length, args.async_mode)
        max_steps = max(0, int(os.getenv("CARLA_COSIM_MAX_STEPS", "0")))
        completed_steps = 0
        logger.info(
            "[run_inprocess] simulation_id={} output={} GIL_interval={}s",
            simulation_id,
            base_dir,
            sys.getswitchinterval(),
        )
        while carla_cosim.tick():
            completed_steps += 1
            if max_steps and completed_steps >= max_steps:
                logger.info("Reached CARLA_COSIM_MAX_STEPS={}", max_steps)
                break
    except KeyboardInterrupt:
        logger.info("Cancelled by user")
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        if carla_cosim is not None:
            carla_cosim.close()
        elif link is not None:
            link.stop()
        else:
            plugin.request_stop()
        simulation_thread.join(timeout=30.0)
        if simulation_thread.is_alive():
            logger.error("TeraSim/SUMO thread did not stop within 30 seconds")


if __name__ == "__main__":
    main()
