#!/usr/bin/env python3
from __future__ import annotations
import json, math, statistics, sys
from pathlib import Path

def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []
def nested(record, path, default=0.0):
    node = record
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node: return default
        node = node[part]
    return node
def values(rows, path):
    result = []
    for row in rows:
        value = nested(row, path, None)
        if isinstance(value, (int, float)) and math.isfinite(value): result.append(float(value))
    return result
def mean_ms(rows, path):
    vals = values(rows, path)
    return statistics.fmean(vals) * 1000 if vals else 0.0
def mean_value(rows, path):
    vals = values(rows, path)
    return statistics.fmean(vals) if vals else 0.0
def percentile(vals, p):
    if not vals: return 0.0
    ordered = sorted(vals); index = (len(ordered) - 1) * p; lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] if lo == hi else ordered[lo] * (hi - index) + ordered[hi] * (index - lo)
def main(root_arg):
    root = Path(root_arg); summaries = []
    for manifest_path in sorted(root.glob("rhi-*/manifest.json")):
        condition_dir = manifest_path.parent; manifest = json.loads(manifest_path.read_text())
        warmup = manifest["warmup_steps"]; measured = manifest["measurement_steps"]
        carla = read_jsonl(condition_dir / "carla_profile.jsonl")[:measured]
        terasim = [r for r in read_jsonl(condition_dir / "terasim_profile.jsonl") if r.get("completed_tick_count", 0) > warmup][:measured]
        tprofiles = [r.get("profile", {}) for r in terasim]; totals = values(carla, "total_s")
        mean_total = statistics.fmean(totals) if totals else 0.0
        row = {**manifest, "condition": condition_dir.name, "samples": len(carla), "terasim_samples": len(terasim),
            "total_mean_ms": mean_total*1000, "total_p95_ms": percentile(totals,.95)*1000,
            "deadline_50ms_pct": sum(v<=.05 for v in totals)/len(totals)*100 if totals else 0.0,
            "realtime_factor": .05/mean_total if mean_total else 0.0,
            "world_tick_mean_ms": mean_ms(carla,"carla_tick.world_tick_s"),
            "carla_state_apply_mean_ms": mean_ms(carla,"carla_state_apply.total_s"),
            "feedback_conversion_mean_ms": mean_ms(carla,"feedback.command_conversion_s"),
            "terasim_roundtrip_mean_ms": mean_ms(carla,"terasim_roundtrip.total_s"),
            "exported_vehicles_mean": mean_value(carla,"counts.exported_sumo_vehicles"),
            "carla_vehicles_mean": mean_value(carla,"counts.carla_vehicle_actors"),
            "physics_vehicles_mean": mean_value(carla,"counts.physics_vehicles"),
            "physics_vehicles_max": max(values(carla,"counts.physics_vehicles") or [0]),
            "detail_vehicles_mean": mean_value(tprofiles,"terasim_internal.state_export.detail_vehicle_count"),
            "sumo_total_vehicles_mean": mean_value(tprofiles,"terasim_internal.sumo_total_vehicle_count"),
            "terasim_internal_mean_ms": mean_ms(tprofiles,"terasim_internal.total_s"),
            "sumo_step_mean_ms": mean_ms(tprofiles,"terasim_internal.sumo_simulation_step_s"),
            "behavior_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation_s"),
            "nde_background_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation.nde_background_update_s"),
            "nade_decision_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation.nade_decision_control_s"),
            "state_export_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.total_s"),
            "state_ackermann_detail_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.ackermann_detail_s"),
            "lookahead_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.lookahead_lane_geometry_s")}
        summaries.append(row)
    (root/"summary.json").write_text(json.dumps(summaries,indent=2)+"\n")
    header=["RHI","radius","samples","SUMO all","exported","CARLA","physics avg/max","detail avg","total avg/p95 ms","<=50ms","RTF","world.tick ms","TeraSim ms","SUMO step ms","behavior ms","state export ms"]
    lines=["# Ackermann feedback performance","","| "+" | ".join(header)+" |","|"+"---|"*len(header)]
    for r in summaries:
        cells=[r["rhi"],f'{r["radius_m"]:.0f}m',str(r["samples"]),f'{r["sumo_total_vehicles_mean"]:.1f}',f'{r["exported_vehicles_mean"]:.1f}',f'{r["carla_vehicles_mean"]:.1f}',f'{r["physics_vehicles_mean"]:.1f}/{r["physics_vehicles_max"]:.0f}',f'{r["detail_vehicles_mean"]:.1f}',f'{r["total_mean_ms"]:.1f}/{r["total_p95_ms"]:.1f}',f'{r["deadline_50ms_pct"]:.0f}%',f'{r["realtime_factor"]:.2f}',f'{r["world_tick_mean_ms"]:.1f}',f'{r["terasim_internal_mean_ms"]:.1f}',f'{r["sumo_step_mean_ms"]:.1f}',f'{r["behavior_mean_ms"]:.1f}',f'{r["state_export_mean_ms"]:.1f}']
        lines.append("| "+" | ".join(cells)+" |")
    (root/"summary.md").write_text("\n".join(lines)+"\n"); print("\n".join(lines))
if __name__ == "__main__": main(sys.argv[1])
