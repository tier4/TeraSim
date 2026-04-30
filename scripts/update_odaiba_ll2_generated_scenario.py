#!/usr/bin/env python3
"""Inject the generated AV route into the LL2 packaged co-sim scenario."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Update LL2 generated scenario with AV route edges.")
    parser.add_argument("--metadata", required=True, help="Path to metadata.json with av_route_edge_ids")
    parser.add_argument("--scenario", required=True, help="Path to cosim_odaiba_ll2_generated.yaml")
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
    scenario["environment"]["parameters"]["AV_cfg"]["route"] = route_edges

    scenario_path.write_text(
        yaml.safe_dump(scenario, sort_keys=False, allow_unicode=False),
        encoding="utf-8",
    )
    print(f"Updated {scenario_path} with {len(route_edges)} AV route edges")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
