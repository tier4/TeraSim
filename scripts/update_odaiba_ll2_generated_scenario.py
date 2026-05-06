#!/usr/bin/env python3
"""Inject generated SUMO settings into the LL2 packaged co-sim scenario."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Update LL2 generated scenario with SUMO artifacts.")
    parser.add_argument("--metadata", required=True, help="Path to metadata.json with av_route_edge_ids")
    parser.add_argument("--scenario", required=True, help="Path to cosim_odaiba_ll2_generated.yaml")
    parser.add_argument(
        "--sumo-net-file",
        default=None,
        help="SUMO net path to write into environment.parameters and input.",
    )
    parser.add_argument(
        "--sumo-config-file",
        default=None,
        help="SUMO config path to write into environment.parameters and input.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metadata_path = Path(args.metadata)
    scenario_path = Path(args.scenario)

    metadata = json.loads(metadata_path.read_text())
    route_edges = metadata.get("av_route_edge_ids")
    if not route_edges:
        raise SystemExit(f"No av_route_edge_ids found in {metadata_path}")

    scenario = yaml.safe_load(scenario_path.read_text())
    environment_parameters = scenario["environment"]["parameters"]
    environment_parameters["AV_cfg"]["route"] = route_edges

    if args.sumo_net_file:
        environment_parameters["sumo_net_file_path"] = args.sumo_net_file
        scenario["input"]["sumo_net_file"] = args.sumo_net_file

    if args.sumo_config_file:
        environment_parameters["sumo_cfg_file_path"] = args.sumo_config_file
        scenario["input"]["sumo_config_file"] = args.sumo_config_file

    scenario_path.write_text(
        yaml.safe_dump(scenario, sort_keys=False, allow_unicode=False),
        encoding="utf-8",
    )
    print(f"Updated {scenario_path} with {len(route_edges)} AV route edges")
    if args.sumo_net_file:
        print(f"Updated SUMO net file: {args.sumo_net_file}")
    if args.sumo_config_file:
        print(f"Updated SUMO config file: {args.sumo_config_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
