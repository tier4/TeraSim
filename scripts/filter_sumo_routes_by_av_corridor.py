#!/usr/bin/env python3
"""Filter SUMO vehicle routes by an AV route corridor.

The first implementation is intentionally conservative: if a vehicle route touches
any edge inside the AV corridor, the vehicle is kept with its original full route
and original depart time. Vehicle routes are never trimmed or retimed.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


Point = tuple[float, float]
Segment = tuple[Point, Point]
BBox = tuple[float, float, float, float]


ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_edge_tokens(route_text: str | None) -> list[str]:
    if not route_text:
        return []
    return [edge_id.strip() for edge_id in route_text.replace(",", " ").split() if edge_id.strip()]


def is_regular_edge_id(edge_id: str) -> bool:
    return not edge_id.startswith(":")


def is_regular_edge(edge) -> bool:
    edge_id = edge.getID()
    if not is_regular_edge_id(edge_id):
        return False
    try:
        return not edge.isSpecial()
    except AttributeError:
        return True


def xy_points(points: Iterable[Iterable[float]]) -> list[Point]:
    xy = []
    for point in points:
        coords = list(point)
        if len(coords) < 2:
            continue
        xy.append((float(coords[0]), float(coords[1])))
    return xy


def get_edge_shape(edge) -> list[Point]:
    try:
        shape = edge.getShape()
    except TypeError:
        shape = edge.getShape(False)
    except AttributeError:
        shape = []

    if not shape:
        try:
            lanes = edge.getLanes()
        except AttributeError:
            lanes = []
        if lanes:
            shape = lanes[0].getShape()

    return xy_points(shape)


def shape_to_segments(shape: list[Point]) -> list[Segment]:
    return [(shape[i], shape[i + 1]) for i in range(len(shape) - 1)]


def bbox_from_points(points: Iterable[Point]) -> BBox | None:
    points = list(points)
    if not points:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def bbox_distance_squared(first: BBox, second: BBox) -> float:
    first_min_x, first_min_y, first_max_x, first_max_y = first
    second_min_x, second_min_y, second_max_x, second_max_y = second
    dx = max(second_min_x - first_max_x, first_min_x - second_max_x, 0.0)
    dy = max(second_min_y - first_max_y, first_min_y - second_max_y, 0.0)
    return dx * dx + dy * dy


def point_distance_squared(first: Point, second: Point) -> float:
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return dx * dx + dy * dy


def point_segment_distance_squared(point: Point, segment: Segment) -> float:
    start, end = segment
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    length_squared = vx * vx + vy * vy
    if length_squared <= 0.0:
        return point_distance_squared(point, start)
    t = ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / length_squared
    t = max(0.0, min(1.0, t))
    projected = (start[0] + t * vx, start[1] + t * vy)
    return point_distance_squared(point, projected)


def orientation(first: Point, second: Point, third: Point) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (
        second[1] - first[1]
    ) * (third[0] - first[0])


def point_on_segment(point: Point, segment: Segment, eps: float = 1e-9) -> bool:
    start, end = segment
    return (
        min(start[0], end[0]) - eps <= point[0] <= max(start[0], end[0]) + eps
        and min(start[1], end[1]) - eps <= point[1] <= max(start[1], end[1]) + eps
        and abs(orientation(start, end, point)) <= eps
    )


def segments_intersect(first: Segment, second: Segment, eps: float = 1e-9) -> bool:
    a, b = first
    c, d = second
    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    if o1 * o2 < -eps and o3 * o4 < -eps:
        return True
    return (
        point_on_segment(c, first, eps)
        or point_on_segment(d, first, eps)
        or point_on_segment(a, second, eps)
        or point_on_segment(b, second, eps)
    )


def segment_distance_squared(first: Segment, second: Segment) -> float:
    if segments_intersect(first, second):
        return 0.0
    return min(
        point_segment_distance_squared(first[0], second),
        point_segment_distance_squared(first[1], second),
        point_segment_distance_squared(second[0], first),
        point_segment_distance_squared(second[1], first),
    )


def collect_route_definitions(root: ET.Element) -> dict[str, list[str]]:
    routes = {}
    for elem in root.iter():
        if local_name(elem.tag) != "route":
            continue
        route_id = elem.get("id")
        edges = parse_edge_tokens(elem.get("edges"))
        if route_id and edges:
            routes[route_id] = edges
    return routes


def get_vehicle_route_edges(
    vehicle_element: ET.Element, route_definitions: dict[str, list[str]]
) -> list[str] | None:
    for child in vehicle_element:
        if local_name(child.tag) == "route":
            edges = parse_edge_tokens(child.get("edges"))
            if edges:
                return edges

    route_id = vehicle_element.get("route")
    if route_id:
        return route_definitions.get(route_id)

    return None


def get_route_edges(root: ET.Element, route_id: str) -> list[str]:
    for elem in root.iter():
        if local_name(elem.tag) == "route" and elem.get("id") == route_id:
            edges = parse_edge_tokens(elem.get("edges"))
            if edges:
                return edges
    raise ValueError(f"Route id {route_id!r} with an edges attribute was not found")


def get_net_edge(net, edge_id: str):
    try:
        return net.getEdge(edge_id)
    except Exception as exc:
        raise ValueError(f"Edge {edge_id!r} was not found in SUMO net") from exc


def build_av_segments(net, av_route_edges: list[str]) -> list[Segment]:
    segments: list[Segment] = []
    for edge_id in av_route_edges:
        if not is_regular_edge_id(edge_id):
            continue
        edge = get_net_edge(net, edge_id)
        if not is_regular_edge(edge):
            continue
        segments.extend(shape_to_segments(get_edge_shape(edge)))
    if not segments:
        raise ValueError("AV route did not produce any regular edge geometry")
    return segments


def iter_regular_net_edges(net):
    try:
        edges = net.getEdges()
    except AttributeError as exc:
        raise ValueError("SUMO net object does not expose getEdges()") from exc
    for edge in edges:
        if is_regular_edge(edge):
            yield edge


def build_corridor_core_edges(net, av_route_edges: list[str], radius: float) -> set[str]:
    if radius < 0:
        raise ValueError("radius must be non-negative")

    av_segments = build_av_segments(net, av_route_edges)
    av_bbox = bbox_from_points(point for segment in av_segments for point in segment)
    if av_bbox is None:
        raise ValueError("AV route geometry is empty")

    radius_squared = radius * radius
    core_edges = set()

    for edge in iter_regular_net_edges(net):
        shape = get_edge_shape(edge)
        segments = shape_to_segments(shape)
        edge_bbox = bbox_from_points(shape)
        if not segments or edge_bbox is None:
            continue
        if bbox_distance_squared(edge_bbox, av_bbox) > radius_squared:
            continue
        if any(
            segment_distance_squared(edge_segment, av_segment) <= radius_squared
            for edge_segment in segments
            for av_segment in av_segments
        ):
            core_edges.add(edge.getID())

    return core_edges


def route_intersects_core_edges(route_edges: list[str], core_edges: set[str]) -> bool:
    return any(edge_id in core_edges for edge_id in route_edges if is_regular_edge_id(edge_id))


def filter_routes_tree(
    root: ET.Element,
    core_edges: set[str],
    *,
    keep_unmatched_prob: float = 0.0,
    seed: int | None = None,
    protected_vehicle_ids: set[str] | None = None,
) -> dict:
    if not 0.0 <= keep_unmatched_prob <= 1.0:
        raise ValueError("keep_unmatched_prob must be between 0 and 1")

    rng = random.Random(seed)
    protected_vehicle_ids = protected_vehicle_ids or set()
    route_definitions = collect_route_definitions(root)
    report = {
        "total_vehicles": 0,
        "kept_vehicles": 0,
        "kept_by_corridor": 0,
        "kept_by_random": 0,
        "kept_protected": 0,
        "kept_unknown_route": 0,
        "removed_vehicles": 0,
    }

    for child in list(root):
        if local_name(child.tag) != "vehicle":
            continue

        report["total_vehicles"] += 1
        vehicle_id = child.get("id", "")
        if vehicle_id in protected_vehicle_ids:
            report["kept_vehicles"] += 1
            report["kept_protected"] += 1
            continue

        route_edges = get_vehicle_route_edges(child, route_definitions)
        if route_edges is None:
            report["kept_vehicles"] += 1
            report["kept_unknown_route"] += 1
            continue

        if route_intersects_core_edges(route_edges, core_edges):
            report["kept_vehicles"] += 1
            report["kept_by_corridor"] += 1
            continue

        if keep_unmatched_prob > 0.0 and rng.random() < keep_unmatched_prob:
            report["kept_vehicles"] += 1
            report["kept_by_random"] += 1
            continue

        root.remove(child)
        report["removed_vehicles"] += 1

    return report


def read_sumo_net(net_path: Path):
    try:
        import sumolib
    except ImportError as exc:
        raise SystemExit("sumolib is required to read SUMO net files") from exc

    return sumolib.net.readNet(str(net_path), withInternal=True)


def filter_routes_by_av_corridor(
    *,
    net,
    routes_path: Path,
    output_path: Path,
    av_route_id: str,
    radius: float,
    keep_unmatched_prob: float = 0.0,
    seed: int | None = None,
    protected_vehicle_ids: set[str] | None = None,
) -> dict:
    tree = ET.parse(routes_path)
    root = tree.getroot()
    av_route_edges = get_route_edges(root, av_route_id)
    core_edges = build_corridor_core_edges(net, av_route_edges, radius)
    report = filter_routes_tree(
        root,
        core_edges,
        keep_unmatched_prob=keep_unmatched_prob,
        seed=seed,
        protected_vehicle_ids=protected_vehicle_ids,
    )
    report.update(
        {
            "av_route_id": av_route_id,
            "av_route_edge_count": len(av_route_edges),
            "corridor_radius": radius,
            "core_edge_count": len(core_edges),
            "keep_unmatched_prob": keep_unmatched_prob,
        }
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Filter SUMO vehicle routes before simulation by keeping only vehicles whose "
            "full route intersects the AV route corridor."
        )
    )
    parser.add_argument("--net", required=True, help="Path to SUMO net.xml.")
    parser.add_argument("--routes", required=True, help="Path to input SUMO routes XML.")
    parser.add_argument("--output", required=True, help="Path to filtered output routes XML.")
    parser.add_argument("--av-route-id", default="av_route", help="Route id used by the AV.")
    parser.add_argument("--radius", type=float, required=True, help="AV corridor radius in meters.")
    parser.add_argument(
        "--keep-unmatched-prob",
        type=float,
        default=0.0,
        help="Probability for retaining vehicles that do not intersect the corridor.",
    )
    parser.add_argument(
        "--seed", type=int, default=2026, help="Random seed for retained unmatched routes."
    )
    parser.add_argument("--report", default=None, help="Optional JSON report output path.")
    parser.add_argument(
        "--protected-vehicle-id",
        action="append",
        default=["AV"],
        help="Vehicle id to always retain. Can be passed multiple times.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = filter_routes_by_av_corridor(
        net=read_sumo_net(Path(args.net)),
        routes_path=Path(args.routes),
        output_path=Path(args.output),
        av_route_id=args.av_route_id,
        radius=args.radius,
        keep_unmatched_prob=args.keep_unmatched_prob,
        seed=args.seed,
        protected_vehicle_ids=set(args.protected_vehicle_id or []),
    )

    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    print(
        "Filtered SUMO routes: "
        f"kept={report['kept_vehicles']}/{report['total_vehicles']} "
        f"removed={report['removed_vehicles']} "
        f"core_edges={report['core_edge_count']}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
