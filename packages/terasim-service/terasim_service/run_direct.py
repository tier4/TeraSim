"""Standalone TeraSim runner for the direct (Redis/FastAPI-free) co-simulation.

Replaces `POST /start_simulation` + the FastAPI service process: builds the
environment/simulator from a scenario YAML exactly like
api.run_simulation_task, injects TeraSimCoSimDirectPlugin (which serves the
CosimDirect gRPC contract in-process), and runs the simulation in the
foreground. The CarlaCosim client connects to --grpc_port and drives the
simulation one Tick RPC per SUMO step.

Usage:
  python -m terasim_service.run_direct --config <scenario.yaml> [--grpc_port 8200]
"""

import argparse
import uuid
from pathlib import Path

from loguru import logger

from .plugins.cosim import DEFAULT_COSIM_PLUGIN_CONFIG
from .plugins.cosim_direct import TeraSimCoSimDirectPlugin
from .utils.base import (
    create_environment,
    create_simulator,
    load_config,
    resolve_config_paths,
    set_random_seed,
)


def main():
    parser = argparse.ArgumentParser(
        description="Run TeraSim with the direct (gRPC) co-sim plugin, no Redis/FastAPI"
    )
    parser.add_argument("--config", required=True, help="scenario YAML path")
    parser.add_argument("--grpc_host", default="127.0.0.1")
    parser.add_argument("--grpc_port", type=int, default=8200)
    args = parser.parse_args()

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
    plugin = TeraSimCoSimDirectPlugin(
        simulation_uuid=simulation_id,
        plugin_config=plugin_config,
        base_dir=str(base_dir),
        auto_run=False,
        grpc_host=args.grpc_host,
        grpc_port=args.grpc_port,
    )
    plugin.inject(sim, {})

    logger.info(
        f"[run_direct] simulation_id={simulation_id} "
        f"grpc={args.grpc_host}:{args.grpc_port} output={base_dir}"
    )
    sim.run()
    logger.info(f"[run_direct] simulation_id={simulation_id} exited")


if __name__ == "__main__":
    main()
