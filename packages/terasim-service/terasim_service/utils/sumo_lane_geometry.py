import math

import numpy as np


def extract_next_link_lane_ids(next_links):
    """Return internal and destination lane IDs from TraCI ``getNextLinks`` data."""
    lane_ids = []
    for link in next_links or []:
        # getNextLinks: (lane, via, priority, opened, foe, state, direction, length)
        for index in (1, 0):
            if index >= len(link):
                continue
            lane_id = link[index]
            if isinstance(lane_id, str) and lane_id and lane_id not in lane_ids:
                lane_ids.append(lane_id)
    return lane_ids


def _as_2d_points(shape):
    try:
        points = [(float(point[0]), float(point[1])) for point in shape]
    except (TypeError, ValueError, IndexError):
        return []
    return points


def flatten_lane_shapes(lane_shapes):
    """Flatten lane shape polylines while dropping repeated junction points."""
    points = []
    for lane_shape in lane_shapes or []:
        for point in _as_2d_points(lane_shape):
            if points and point == points[-1]:
                continue
            points.append(point)
    return points


def _project_distance_on_polyline(points, position):
    if len(points) < 2:
        return None

    try:
        pos_x = float(position[0])
        pos_y = float(position[1])
    except (TypeError, ValueError, IndexError):
        return None

    travelled = 0.0
    best_distance_sq = None
    best_projected_distance = 0.0

    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length_sq = dx * dx + dy * dy
        if segment_length_sq <= 0.0:
            continue
        segment_length = math.sqrt(segment_length_sq)
        t = ((pos_x - start[0]) * dx + (pos_y - start[1]) * dy) / segment_length_sq
        t = min(1.0, max(0.0, t))
        proj_x = start[0] + dx * t
        proj_y = start[1] + dy * t
        distance_sq = (pos_x - proj_x) ** 2 + (pos_y - proj_y) ** 2
        if best_distance_sq is None or distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_projected_distance = travelled + segment_length * t
        travelled += segment_length

    return best_projected_distance


def project_position_to_lane_shape(lane_shape, position, lane_length=None):
    """Project an x/y position onto a lane shape.

    The returned lane position uses SUMO's lane length rather than the raw
    polyline length when ``lane_length`` is provided. This matters because
    ``vehicle.moveTo`` expects the distance from the lane start to the vehicle
    front bumper in SUMO lane coordinates.
    """
    points = _as_2d_points(lane_shape)
    if len(points) < 2:
        return None

    try:
        pos_x = float(position[0])
        pos_y = float(position[1])
    except (TypeError, ValueError, IndexError):
        return None

    travelled = 0.0
    total_length = 0.0
    best = None
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 0.0:
            continue
        segment_length_sq = segment_length * segment_length
        t = ((pos_x - start[0]) * dx + (pos_y - start[1]) * dy) / segment_length_sq
        t = min(1.0, max(0.0, t))
        proj_x = start[0] + dx * t
        proj_y = start[1] + dy * t
        distance = math.hypot(pos_x - proj_x, pos_y - proj_y)
        projected_distance = travelled + segment_length * t
        sumo_angle = (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0
        if best is None or distance < best["distance"]:
            best = {
                "shape_position": projected_distance,
                "distance": distance,
                "sumo_angle": sumo_angle,
                "projected_x": proj_x,
                "projected_y": proj_y,
            }
        travelled += segment_length
        total_length += segment_length

    if best is None or total_length <= 0.0:
        return None

    if lane_length is None:
        sumo_lane_length = total_length
    else:
        try:
            sumo_lane_length = max(0.0, float(lane_length))
        except (TypeError, ValueError):
            return None
    best["lane_position"] = min(
        sumo_lane_length,
        max(0.0, best["shape_position"] / total_length * sumo_lane_length),
    )
    return best


def _angle_difference_degrees(first, second):
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def select_route_aware_lane_projection(
    position,
    sumo_angle,
    lane_candidates,
    current_lane_id="",
    lane_switch_hysteresis=0.35,
    heading_weight=0.02,
    max_distance=None,
    max_heading_error=90.0,
    prefer_current_lane=False,
):
    """Select the best projection among caller-supplied route-aware lanes.

    This function deliberately does not search the whole SUMO network. The
    caller is responsible for supplying only the current edge, adjacent lanes,
    route successors and applicable internal lanes.
    """
    try:
        sumo_angle = float(sumo_angle)
        lane_switch_hysteresis = max(0.0, float(lane_switch_hysteresis))
        heading_weight = max(0.0, float(heading_weight))
        max_heading_error = max(0.0, float(max_heading_error))
        if max_distance is not None:
            max_distance = max(0.0, float(max_distance))
    except (TypeError, ValueError):
        return None

    best = None
    current_lane_projection = None
    for candidate in lane_candidates or []:
        lane_id = candidate.get("lane_id")
        if not lane_id:
            continue
        projection = project_position_to_lane_shape(
            candidate.get("shape"),
            position,
            candidate.get("length"),
        )
        if projection is None:
            continue
        heading_error = _angle_difference_degrees(
            sumo_angle,
            projection["sumo_angle"],
        )
        if heading_error > max_heading_error:
            continue
        if max_distance is not None and projection["distance"] > max_distance:
            continue

        continuity_penalty = 0.0 if lane_id == current_lane_id else lane_switch_hysteresis
        score = projection["distance"] + heading_weight * heading_error + continuity_penalty
        result = {
            **projection,
            "lane_id": lane_id,
            "heading_error": heading_error,
            "score": score,
        }
        if lane_id == current_lane_id:
            current_lane_projection = result
        if best is None or result["score"] < best["score"]:
            best = result
    if prefer_current_lane and current_lane_projection is not None:
        return current_lane_projection
    return best


def point_at_distance_on_polyline(points, target_distance, z=0.0):
    """Return a 3D point at the requested travelled distance along a polyline."""
    if len(points) < 2:
        return None

    try:
        target_distance = max(0.0, float(target_distance))
        z = float(z)
    except (TypeError, ValueError):
        return None

    travelled = 0.0
    last_valid_end = None
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 0.0:
            continue
        last_valid_end = end
        if travelled + segment_length >= target_distance:
            ratio = (target_distance - travelled) / segment_length
            return (
                start[0] + dx * ratio,
                start[1] + dy * ratio,
                z,
            )
        travelled += segment_length

    if last_valid_end is None:
        return None
    return last_valid_end[0], last_valid_end[1], z


def find_lookahead_position_from_lane_shapes(
    lane_shapes, current_position, lookahead_distance, z=0.0
):
    """Find a lookahead point on lane-shape polylines near a moving SUMO vehicle."""
    points = flatten_lane_shapes(lane_shapes)
    projected_distance = _project_distance_on_polyline(points, current_position)
    if projected_distance is None:
        return None
    return point_at_distance_on_polyline(points, projected_distance + lookahead_distance, z)


def compile_lane_shapes(lane_shapes):
    """Compile lane polylines into NumPy arrays reusable across simulation ticks."""
    points = np.asarray(flatten_lane_shapes(lane_shapes), dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 2:
        return None

    starts = points[:-1]
    deltas = points[1:] - starts
    length_sq = np.einsum("ij,ij->i", deltas, deltas)
    valid = length_sq > 0.0
    if not np.any(valid):
        return None

    starts = starts[valid]
    deltas = deltas[valid]
    length_sq = length_sq[valid]
    lengths = np.sqrt(length_sq)
    cumulative_starts = np.concatenate(
        (np.zeros(1, dtype=np.float64), np.cumsum(lengths[:-1]))
    )
    cumulative_ends = cumulative_starts + lengths
    return {
        "starts": starts,
        "deltas": deltas,
        "length_sq": length_sq,
        "lengths": lengths,
        "cumulative_starts": cumulative_starts,
        "cumulative_ends": cumulative_ends,
        "last_end": starts[-1] + deltas[-1],
    }


def find_lookahead_positions_from_compiled_paths(
    compiled_paths,
    current_positions,
    lookahead_distances,
    z_values,
):
    """Calculate route lookahead points for multiple vehicles in vectorized groups.

    Vehicles sharing the same compiled route path are projected together. The
    result matches the scalar lane-shape lookup, while avoiding repeated
    polyline flattening and per-segment Python loops.
    """
    count = len(compiled_paths)
    if not (
        len(current_positions) == count
        and len(lookahead_distances) == count
        and len(z_values) == count
    ):
        raise ValueError("compiled path and vehicle input lengths must match")

    results = [None] * count
    groups = {}
    for index, compiled_path in enumerate(compiled_paths):
        if compiled_path is not None:
            groups.setdefault(id(compiled_path), (compiled_path, []))[1].append(index)

    for compiled_path, indices in groups.values():
        try:
            positions = np.asarray(
                [current_positions[index] for index in indices], dtype=np.float64
            )
            distances = np.maximum(
                0.0,
                np.asarray(
                    [lookahead_distances[index] for index in indices], dtype=np.float64
                ),
            )
            z_array = np.asarray([z_values[index] for index in indices], dtype=np.float64)
        except (TypeError, ValueError):
            continue
        if positions.ndim != 2 or positions.shape[1] != 2:
            continue

        starts = compiled_path["starts"]
        deltas = compiled_path["deltas"]
        length_sq = compiled_path["length_sq"]
        lengths = compiled_path["lengths"]
        cumulative_starts = compiled_path["cumulative_starts"]
        cumulative_ends = compiled_path["cumulative_ends"]

        offsets = positions[:, None, :] - starts[None, :, :]
        ratios = np.einsum("bsi,si->bs", offsets, deltas) / length_sq[None, :]
        ratios = np.clip(ratios, 0.0, 1.0)
        projections = starts[None, :, :] + ratios[..., None] * deltas[None, :, :]
        distance_sq = np.sum((positions[:, None, :] - projections) ** 2, axis=2)
        nearest_indices = np.argmin(distance_sq, axis=1)
        rows = np.arange(len(indices))
        projected_distances = (
            cumulative_starts[nearest_indices]
            + lengths[nearest_indices] * ratios[rows, nearest_indices]
        )
        target_distances = projected_distances + distances
        target_indices = np.searchsorted(cumulative_ends, target_distances, side="left")
        beyond_path = target_indices >= len(lengths)
        target_indices = np.minimum(target_indices, len(lengths) - 1)
        target_ratios = (
            target_distances - cumulative_starts[target_indices]
        ) / lengths[target_indices]
        target_ratios = np.clip(target_ratios, 0.0, 1.0)
        target_points = (
            starts[target_indices] + target_ratios[:, None] * deltas[target_indices]
        )
        target_points[beyond_path] = compiled_path["last_end"]

        for row, result_index in enumerate(indices):
            results[result_index] = (
                float(target_points[row, 0]),
                float(target_points[row, 1]),
                float(z_array[row]),
            )

    return results


def project_position_by_sumo_angle(position, sumo_angle, distance, z=0.0):
    """Project a SUMO position forward using SUMO's heading convention."""
    try:
        x = float(position[0])
        y = float(position[1])
        sumo_angle = float(sumo_angle)
        distance = float(distance)
        z = float(z)
    except (TypeError, ValueError, IndexError):
        return None
    heading = math.radians(90.0 - sumo_angle)
    return (
        x + math.cos(heading) * distance,
        y + math.sin(heading) * distance,
        z,
    )


def reconstruct_position_from_lane_geometry(
    lane_shape,
    lane_position: float,
    lateral_offset: float,
    z: float = 0.0,
):
    """Reconstruct a SUMO x/y/z position from lane-relative coordinates."""
    if not lane_shape or len(lane_shape) < 2:
        return None

    try:
        points = [(float(point[0]), float(point[1])) for point in lane_shape]
        target_distance = max(0.0, float(lane_position))
        lateral_offset = float(lateral_offset)
        z = float(z)
    except (TypeError, ValueError, IndexError):
        return None

    travelled = 0.0
    last_segment = None
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 0.0:
            continue
        last_segment = (start, dx, dy, segment_length)
        if travelled + segment_length >= target_distance:
            ratio = (target_distance - travelled) / segment_length
            center_x = start[0] + dx * ratio
            center_y = start[1] + dy * ratio
            normal_x = -dy / segment_length
            normal_y = dx / segment_length
            return (
                center_x + normal_x * lateral_offset,
                center_y + normal_y * lateral_offset,
                z,
            )
        travelled += segment_length

    if last_segment is None:
        return None

    start, dx, dy, segment_length = last_segment
    end_x = start[0] + dx
    end_y = start[1] + dy
    normal_x = -dy / segment_length
    normal_y = dx / segment_length
    return (
        end_x + normal_x * lateral_offset,
        end_y + normal_y * lateral_offset,
        z,
    )
