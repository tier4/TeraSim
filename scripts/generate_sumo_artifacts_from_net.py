#!/usr/bin/env python3
"""Generate minimal SUMO artifacts for a TeraSim map directory.

This script is intentionally lightweight so it can run inside the existing
`terasim-service:cosim` image without installing the full `terasim-envgen`
dependency stack.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import sumolib


NDE_VTYPES = [
    {
        "id": "NDE_URBAN",
        "length": "5.00",
        "width": "1.85",
        "minGap": "3.28",
        "maxSpeed": "10.00",
        "carFollowModel": "IDM",
        "accel": "1.84",
        "decel": "1.29",
        "tau": "1.17",
        "emergencyDecel": "7.06",
        "lcSpeedGain": "0",
        "lcCooperative": "0",
        "lcKeepRight": "1",
        "speedFactor": "normc(1,0.1,0.8,1.2)",
    },
    {
        "id": "NDE_HIGHWAY",
        "length": "5.00",
        "width": "1.85",
        "minGap": "5.92",
        "maxSpeed": "28.31",
        "carFollowModel": "IDM",
        "accel": "5.95",
        "decel": "5.96",
        "tau": "1.72",
        "emergencyDecel": "7.06",
        "lcSpeedGain": "0",
        "lcCooperative": "0",
        "lcKeepRight": "1",
        "speedFactor": "normc(1,0.1,0.8,1.2)",
    },
]


def ensure_metadata(metadata_path: Path) -> dict:
    if metadata_path.exists():
        data = json.loads(metadata_path.read_text())
        if not isinstance(data, dict):
            raise ValueError(f"{metadata_path} must contain a JSON object")
    else:
        data = {}
    metadata_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    return data


def get_random_trips_script() -> str:
    candidates = [
        os.environ.get("SUMO_RANDOM_TRIPS"),
        "/usr/local/lib/python3.10/site-packages/sumo/tools/randomTrips.py",
        "/usr/share/sumo/tools/randomTrips.py",
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    raise FileNotFoundError("Could not locate randomTrips.py")


def generate_av_fallback_route(net_path: Path, seed: int | None = None) -> list[sumolib.net.edge.Edge]:
    if seed is not None:
        random.seed(seed)

    sumo_net = sumolib.net.readNet(str(net_path), withInternal=True)
    regular_edges = [edge for edge in sumo_net.getEdges() if not edge.isSpecial()]
    if not regular_edges:
        raise RuntimeError(f"No regular edges found in {net_path}")

    peripheral_edges = []
    for edge in regular_edges:
        if len(edge.getIncoming()) <= 1 or len(edge.getOutgoing()) <= 1:
            peripheral_edges.append(edge)

    candidates = peripheral_edges if len(peripheral_edges) >= 2 else regular_edges
    if len(candidates) > 128:
        candidates = random.sample(candidates, 128)

    best_route_edges = None
    best_score = -1.0

    for i, src in enumerate(candidates):
        for dst in candidates[i + 1:]:
            try:
                route_edges, _ = sumo_net.getShortestPath(src, dst)
                if not route_edges:
                    route_edges, _ = sumo_net.getShortestPath(dst, src)
                if not route_edges:
                    continue
                score = sum(edge.getLength() for edge in route_edges)
                if score > best_score:
                    best_score = score
                    best_route_edges = route_edges
            except Exception:
                continue

    if best_route_edges:
        return list(best_route_edges)
    return [max(regular_edges, key=lambda edge: edge.getLength())]


def save_av_route_metadata(
    net_path: Path,
    metadata_path: Path,
    *,
    seed: int | None = None,
    force_new_route: bool = False,
) -> list[str]:
    metadata = ensure_metadata(metadata_path)
    sumo_net = sumolib.net.readNet(str(net_path), withInternal=True)

    if force_new_route:
        metadata.pop("av_route", None)
        metadata.pop("av_route_sumo", None)
        metadata.pop("av_route_edge_ids", None)

    if metadata.get("av_route"):
        try:
            av_route_xy = [
                sumo_net.convertLonLat2XY(point[1], point[0])
                for point in metadata["av_route"]
            ]
            av_route_edges = sumolib.route.mapTrace(
                av_route_xy,
                sumo_net,
                delta=10,
                fillGaps=100,
                verbose=False,
            )
        except Exception:
            av_route_edges = []
        if not av_route_edges:
            av_route_objects = generate_av_fallback_route(net_path, seed=seed)
        else:
            av_route_objects = av_route_edges
    else:
        av_route_objects = generate_av_fallback_route(net_path, seed=seed)

    av_route_with_internal = sumolib.route.addInternal(sumo_net, av_route_objects)

    latlon_points = []
    try:
        for edge in av_route_with_internal:
            for x, y in edge.getShape():
                lon, lat = sumo_net.convertXY2LonLat(x, y)
                latlon_points.append([lat, lon])
    except Exception:
        latlon_points = []

    metadata["av_route_sumo"] = latlon_points
    metadata["av_route_edge_ids"] = [edge.getID() for edge in av_route_objects]
    if "av_route" not in metadata and latlon_points:
        metadata["av_route"] = latlon_points
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")

    return [edge.getID() for edge in av_route_objects]


def create_minimal_routes(routes_path: Path) -> None:
    root = ET.Element("routes")
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(routes_path, encoding="utf-8", xml_declaration=True)


def generate_vehicle_routes(
    net_path: Path,
    trips_path: Path,
    routes_path: Path,
    *,
    end_time: int,
    period: float,
    seed: int,
) -> None:
    random_trips = get_random_trips_script()
    base_cmd = [
        sys.executable,
        random_trips,
        "-n",
        str(net_path),
        "-o",
        str(trips_path),
        "-r",
        str(routes_path),
        "-e",
        str(end_time),
        "-p",
        str(period),
        "--random",
        "--seed",
        str(seed),
        "--lanes",
        "--length",
        "--vehicle-class",
        "passenger",
        "--vclass",
        "passenger",
        "--prefix",
        "vehicle",
        "--validate",
        "--route-file",
        str(routes_path),
    ]

    commands = [
        base_cmd + ["--fringe-factor", "100", "--allow-fringe"],
        base_cmd + ["--fringe-factor", "1.0", "--allow-fringe"],
    ]

    last_error = None
    for cmd in commands:
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            return
        except subprocess.CalledProcessError as exc:
            last_error = exc

    create_minimal_routes(routes_path)
    if last_error:
        print("Warning: randomTrips.py failed; created minimal routes file instead.", file=sys.stderr)
        if last_error.stderr:
            print(last_error.stderr, file=sys.stderr)


def ensure_vtypes_and_av_route(routes_path: Path, av_route_edges: list[str]) -> None:
    if routes_path.exists():
        tree = ET.parse(routes_path)
        root = tree.getroot()
    else:
        root = ET.Element("routes")
        tree = ET.ElementTree(root)

    existing_vtypes = {elem.get("id") for elem in root.findall("vType")}
    insert_index = 0
    for vtype in reversed(NDE_VTYPES):
        if vtype["id"] in existing_vtypes:
            continue
        elem = ET.Element("vType", vtype)
        root.insert(insert_index, elem)
    existing_route = root.find("route[@id='av_route']")
    if existing_route is not None:
        root.remove(existing_route)
    root.append(ET.Element("route", {"id": "av_route", "edges": " ".join(av_route_edges)}))

    ET.indent(tree, space="  ")
    tree.write(routes_path, encoding="utf-8", xml_declaration=True)


def create_sumo_config(net_path: Path, routes_path: Path, cfg_path: Path, *, end_time: int) -> None:
    root = ET.Element("configuration")
    input_section = ET.SubElement(root, "input")
    ET.SubElement(input_section, "net-file", {"value": net_path.name})
    ET.SubElement(input_section, "route-files", {"value": routes_path.name})
    ET.SubElement(input_section, "step-length", {"value": "0.1"})

    time_section = ET.SubElement(root, "time")
    ET.SubElement(time_section, "begin", {"value": "0"})
    ET.SubElement(time_section, "end", {"value": str(end_time)})

    processing = ET.SubElement(root, "processing")
    ET.SubElement(processing, "lateral-resolution", {"value": "0.5"})
    ET.SubElement(processing, "time-to-teleport", {"value": "-1"})
    ET.SubElement(processing, "collision.action", {"value": "warn"})
    ET.SubElement(processing, "collision.check-junctions", {"value": "true"})
    ET.SubElement(processing, "collision.mingap-factor", {"value": "0"})

    report = ET.SubElement(root, "report")
    ET.SubElement(report, "verbose", {"value": "true"})
    ET.SubElement(report, "no-step-log", {"value": "true"})

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(cfg_path, encoding="utf-8", xml_declaration=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate SUMO artifacts from a SUMO network.")
    parser.add_argument("--net", required=True, help="Path to the SUMO network file.")
    parser.add_argument("--output-dir", required=True, help="Directory for generated files.")
    parser.add_argument("--metadata", default=None, help="Path to metadata.json (default: <output-dir>/metadata.json).")
    parser.add_argument("--end-time", type=int, default=3600, help="Simulation end time in seconds.")
    parser.add_argument("--period", type=float, default=2.0, help="Vehicle generation period for randomTrips.")
    parser.add_argument("--seed", type=int, default=2026, help="Random seed.")
    parser.add_argument(
        "--av-route-seed",
        type=int,
        default=None,
        help="Random seed used only for AV route generation. Defaults to --seed.",
    )
    parser.add_argument(
        "--force-new-av-route",
        action="store_true",
        help="Discard any saved AV route metadata and generate a fresh AV route.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    net_path = Path(args.net).resolve()
    output_dir = Path(args.output_dir).resolve()
    metadata_path = Path(args.metadata).resolve() if args.metadata else output_dir / "metadata.json"

    output_dir.mkdir(parents=True, exist_ok=True)

    trips_path = output_dir / "trips.trips.xml"
    routes_path = output_dir / "vehicles.rou.xml"
    cfg_path = output_dir / "simulation.sumocfg"
    av_route_seed = args.seed if args.av_route_seed is None else args.av_route_seed

    av_route_edges = save_av_route_metadata(
        net_path,
        metadata_path,
        seed=av_route_seed,
        force_new_route=args.force_new_av_route,
    )
    generate_vehicle_routes(
        net_path,
        trips_path,
        routes_path,
        end_time=args.end_time,
        period=args.period,
        seed=args.seed,
    )
    ensure_vtypes_and_av_route(routes_path, av_route_edges)
    create_sumo_config(net_path, routes_path, cfg_path, end_time=args.end_time)

    print(f"Generated metadata: {metadata_path}")
    print(f"Generated trips: {trips_path}")
    print(f"Generated routes: {routes_path}")
    print(f"Generated SUMO config: {cfg_path}")
    print(f"AV route seed: {av_route_seed}")
    print(f"Forced new AV route: {args.force_new_av_route}")
    print(f"Generated AV route edges: {' '.join(av_route_edges)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
