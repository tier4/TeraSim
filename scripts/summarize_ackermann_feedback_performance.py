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
def std_ms(rows, path):
    vals = values(rows, path)
    return statistics.stdev(vals) * 1000 if len(vals) > 1 else 0.0
def mean_value(rows, path):
    vals = values(rows, path)
    return statistics.fmean(vals) if vals else 0.0
def std_value(rows, path):
    vals = values(rows, path)
    return statistics.stdev(vals) if len(vals) > 1 else 0.0
def percentile(vals, p):
    if not vals: return 0.0
    ordered = sorted(vals); index = (len(ordered) - 1) * p; lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] if lo == hi else ordered[lo] * (hi - index) + ordered[hi] * (index - lo)
def mean_sd(mean, sd, decimals=1):
    return f"{mean:.{decimals}f} ± {sd:.{decimals}f}"
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
            "total_mean_ms": mean_total*1000,
            "total_sd_ms": statistics.stdev(totals)*1000 if len(totals) > 1 else 0.0,
            "total_p95_ms": percentile(totals,.95)*1000,
            "deadline_50ms_pct": sum(v<=.05 for v in totals)/len(totals)*100 if totals else 0.0,
            "realtime_factor": .05/mean_total if mean_total else 0.0,
            "world_tick_mean_ms": mean_ms(carla,"carla_tick.world_tick_s"),
            "world_tick_sd_ms": std_ms(carla,"carla_tick.world_tick_s"),
            "carla_state_apply_mean_ms": mean_ms(carla,"carla_state_apply.total_s"),
            "feedback_conversion_mean_ms": mean_ms(carla,"feedback.command_conversion_s"),
            "terasim_roundtrip_mean_ms": mean_ms(carla,"terasim_roundtrip.total_s"),
            "exported_vehicles_mean": mean_value(carla,"counts.exported_sumo_vehicles"),
            "exported_vehicles_sd": std_value(carla,"counts.exported_sumo_vehicles"),
            "carla_vehicles_mean": mean_value(carla,"counts.carla_vehicle_actors"),
            "carla_vehicles_sd": std_value(carla,"counts.carla_vehicle_actors"),
            "physics_vehicles_mean": mean_value(carla,"counts.physics_vehicles"),
            "physics_vehicles_sd": std_value(carla,"counts.physics_vehicles"),
            "physics_vehicles_max": max(values(carla,"counts.physics_vehicles") or [0]),
            "detail_vehicles_mean": mean_value(tprofiles,"terasim_internal.state_export.detail_vehicle_count"),
            "detail_vehicles_sd": std_value(tprofiles,"terasim_internal.state_export.detail_vehicle_count"),
            "sumo_total_vehicles_mean": mean_value(tprofiles,"terasim_internal.sumo_total_vehicle_count"),
            "sumo_total_vehicles_sd": std_value(tprofiles,"terasim_internal.sumo_total_vehicle_count"),
            "terasim_internal_mean_ms": mean_ms(tprofiles,"terasim_internal.total_s"),
            "terasim_internal_sd_ms": std_ms(tprofiles,"terasim_internal.total_s"),
            "sumo_step_mean_ms": mean_ms(tprofiles,"terasim_internal.sumo_simulation_step_s"),
            "sumo_step_sd_ms": std_ms(tprofiles,"terasim_internal.sumo_simulation_step_s"),
            "behavior_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation_s"),
            "behavior_sd_ms": std_ms(tprofiles,"terasim_internal.behavior_generation_s"),
            "nde_background_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation.nde_background_update_s"),
            "nade_decision_mean_ms": mean_ms(tprofiles,"terasim_internal.behavior_generation.nade_decision_control_s"),
            "state_export_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.total_s"),
            "state_export_sd_ms": std_ms(tprofiles,"terasim_internal.state_export.total_s"),
            "feedback_ingestion_mean_ms": mean_ms(tprofiles,"terasim_internal.pre_step_command_ingestion_s"),
            "feedback_ingestion_sd_ms": std_ms(tprofiles,"terasim_internal.pre_step_command_ingestion_s"),
            "feedback_total_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.total_s"),
            "feedback_parse_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.parse_s"),
            "feedback_position_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.position_feedback_s"),
            "feedback_traci_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.total_s"),
            "feedback_get_lane_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.vehicle_get_lane_id_s"),
            "feedback_lane_shape_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.lane_get_shape_s"),
            "feedback_lane_length_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.lane_get_length_s"),
            "feedback_move_to_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.vehicle_move_to_s"),
            "feedback_set_previous_speed_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.traci.vehicle_set_previous_speed_s"),
            "feedback_python_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.python.total_s"),
            "feedback_projection_mean_ms": mean_ms(tprofiles,"terasim_internal.feedback_command_breakdown.python.current_lane_projection_s"),
            "feedback_commands_mean": mean_value(tprofiles,"terasim_internal.feedback_command_breakdown.command_count"),
            "feedback_lane_cache_hits_mean": mean_value(tprofiles,"terasim_internal.feedback_command_breakdown.lane_geometry_cache_hits"),
            "feedback_lane_cache_misses_mean": mean_value(tprofiles,"terasim_internal.feedback_command_breakdown.lane_geometry_cache_misses"),
            "state_ackermann_detail_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.ackermann_detail_s"),
            "lookahead_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.lookahead_lane_geometry_s"),
            "detail_traci_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.ackermann_detail_breakdown.traci.total_s"),
            "detail_python_mean_ms": mean_ms(tprofiles,"terasim_internal.state_export.ackermann_detail_breakdown.python.total_s")}
        summaries.append(row)
    rhi_order = {"normal": 0, "nullrhi": 1}
    summaries.sort(key=lambda row: (rhi_order.get(row["rhi"], 99), row["radius_m"]))
    (root/"summary.json").write_text(json.dumps(summaries,indent=2)+"\n")
    header=["RHI","radius","samples","SUMO all","exported","CARLA","physics mean ± SD/max","detail","total mean ± SD/p95 ms","<=50ms","RTF","world.tick ms","TeraSim ms","SUMO step ms","behavior ms","state export ms"]
    lines=["# Ackermann feedback performance","","| "+" | ".join(header)+" |","|"+"---|"*len(header)]
    for r in summaries:
        cells=[
            r["rhi"], f'{r["radius_m"]:.0f}m', str(r["samples"]),
            mean_sd(r["sumo_total_vehicles_mean"], r["sumo_total_vehicles_sd"]),
            mean_sd(r["exported_vehicles_mean"], r["exported_vehicles_sd"]),
            mean_sd(r["carla_vehicles_mean"], r["carla_vehicles_sd"]),
            f'{mean_sd(r["physics_vehicles_mean"], r["physics_vehicles_sd"])}/{r["physics_vehicles_max"]:.0f}',
            mean_sd(r["detail_vehicles_mean"], r["detail_vehicles_sd"]),
            f'{mean_sd(r["total_mean_ms"], r["total_sd_ms"])}/{r["total_p95_ms"]:.1f}',
            f'{r["deadline_50ms_pct"]:.0f}%', f'{r["realtime_factor"]:.2f}',
            mean_sd(r["world_tick_mean_ms"], r["world_tick_sd_ms"]),
            mean_sd(r["terasim_internal_mean_ms"], r["terasim_internal_sd_ms"]),
            mean_sd(r["sumo_step_mean_ms"], r["sumo_step_sd_ms"]),
            mean_sd(r["behavior_mean_ms"], r["behavior_sd_ms"]),
            mean_sd(r["state_export_mean_ms"], r["state_export_sd_ms"]),
        ]
        lines.append("| "+" | ".join(cells)+" |")
    lines.extend([
        "",
        "## 統計表記と計測条件",
        "",
        "特記がない値は、warm-up後の計測stepについての `mean ± sample SD` です。SDは標本標準偏差（分母 n−1）です。totalのみ、末尾にp95も併記し、`mean ± SD / p95` と表記します。時間は `time.perf_counter()` によるwall-clock timeです。",
        "このSDは各条件1 run内のstep間変動を表し、独立した反復run間の不確実性ではありません。run間の再現性を論文で評価する場合は、各条件を複数seedまたは複数repeatで追加計測する必要があります。",
        "",
        "",
        "計測条件は、in-process通信、libsumo、交通流period 0.2秒、simulation step 0.05秒、warm-up 600 step、計測100 step、SUMO 8 threads、FCD出力なし、CARLA actor filter 300 m、TeraSim state filter 320 mです。physics半径0 mは正の微小値で実装しており、AVのみphysics ONを意味します。",
        "",
        "各処理時間は非同期pipeline内の異なる区間を測るinclusive timingです。TeraSim requestはCARLA側の別処理と一部重なるため、各時間列を加算してもtotalとは一致しません。",
        "",
        "## 各列の意味と内部処理",
        "",
        "| 列 | 意味 | 内部で行う処理・計測範囲 |",
        "|---|---|---|",
        "| RHI | CARLAの描画モードです。`normal`は通常描画、`nullrhi`はNullRHI起動です。 | timed regionではなく実験条件です。 |",
        "| radius | AVを中心にCARLA physicsと詳細Ackermann stateを有効化する進入半径です。 | AVとの平面距離で選択します。境界でのON/OFFチャタリングを防ぐため、離脱側には10 mのhysteresisを使用します。 |",
        "| samples | 統計に使用したwarm-up後のclient cycle数です。 | CARLA profileと、対応する完了済みTeraSim tickを最大100件読みます。 |",
        "| SUMO all | SUMO simulation全体に存在する車両数です。 | state export直前に `vehicle.getIDCount()` で取得します。CARLAへの出力範囲外の車両も含みます。 |",
        "| exported | TeraSimからCARLAへ出力したSUMO車両state数です。 | AV中心320 mのstate filter/context subscriptionで選択し、基本位置・姿勢・速度・車両属性を構築します。 |",
        "| CARLA | 永続role-name actor indexで確認できたCARLA vehicle actor数です。 | radius filteringとphysics選択の前に、現在同期中のCARLA vehicle actorを数えます。spawn失敗やactor lifecycleの時差によりexportedと一致しない場合があります。 |",
        "| physics mean ± SD/max | CARLA physicsがONの車両数で、最大値も併記します。 | AVは常時ONです。背景車両はradius内でON、`radius + 10 m`より外でOFFになります。ON車両にはAckermann指令、OFF車両にはbatched transformを適用します。 |",
        "| detail | 詳細なAckermann関連SUMO stateを取得した車両数です。 | lane相対位置、加速度、desired speed、emergency deceleration、feedback acknowledgement、角速度、route-aware lookahead geometryを追加取得します。選択範囲はphysics半径に揃えます。 |",
        "| total mean ± SD/p95 ms | CARLA側co-simulation client 1 cycleのend-to-end wall-clock時間です。 | 前回TeraSim tickの完了処理、SUMO stateのCARLA反映、`world.tick()`、CARLA feedback収集、次のin-process TeraSim tick投入を含みます。 |",
        "| <=50ms | totalが50 ms以内だったcycleの割合です。 | `count(total <= 0.05 s) / samples × 100`で、20 Hz deadline達成率を表します。 |",
        "| RTF | mean totalから求めたreal-time factorです。 | `0.05 s / mean(total)`です。1より大きければ、平均throughputは20 Hz real timeより高速です。 |",
        "| world.tick ms | CARLAの同期 `world.tick()` がblockした時間です。 | CARLAを0.05秒分進め、server側physics、actor更新、通常RHI時のrendering、同期完了を待ちます。直後のsnapshot refreshは含みません。 |",
        "| TeraSim ms | requestされたTeraSim tick 1回の内部wall-clock時間です。 | CARLA feedback command取込み、environment/vehicle maintenance、NDE/NADE処理、SUMO進行、bookkeeping、state construction/export、in-process plain-dict変換を含みます。 |",
        "| SUMO step ms | 最後の `simulationStep()` 呼出しだけの時間です。 | SUMO時刻を次のfull stepへ進め、交通流、信号、衝突、到着・出発を更新します。先に実行される `executeMove()` half-stepは含まず、behavior側に含まれます。 |",
        "| behavior ms | NADE environmentの行動生成時間です。 | AV/context準備、NDE背景車両のpolicy・route・lane-change判断、観測生成、`executeMove()` half-stepと車両list更新、NADE importance sampling・collision avoidance判断、SUMO control command実行を含みます。 |",
        "| state export ms | CARLAへ返すTeraSim stateの構築時間です。 | ID選択/filtering、基本vehicle state、詳細Ackermann/lookahead state、VRU・construction・traffic-light stateを収集します。構築後のin-process plain-dict変換はこの列には含まず、TeraSimには含みます。 |",
    ])
    feedback_header = ["RHI","radius","samples","SUMO all","CARLA","physics mean ± SD/max","total mean ± SD/p95 ms","<=50ms","RTF","world.tick ms","TeraSim ms","feedback ingestion ms","SUMO step ms","behavior ms","state export ms"]
    feedback_lines = [
        "# Ackermann feedback performance — feedback ingestion view",
        "",
        "| " + " | ".join(feedback_header) + " |",
        "|" + "---|" * len(feedback_header),
    ]
    for r in summaries:
        feedback_cells = [
            r["rhi"], f'{r["radius_m"]:.0f}m', str(r["samples"]),
            mean_sd(r["sumo_total_vehicles_mean"], r["sumo_total_vehicles_sd"]),
            mean_sd(r["carla_vehicles_mean"], r["carla_vehicles_sd"]),
            f'{mean_sd(r["physics_vehicles_mean"], r["physics_vehicles_sd"])}/{r["physics_vehicles_max"]:.0f}',
            f'{mean_sd(r["total_mean_ms"], r["total_sd_ms"])}/{r["total_p95_ms"]:.1f}',
            f'{r["deadline_50ms_pct"]:.0f}%', f'{r["realtime_factor"]:.2f}',
            mean_sd(r["world_tick_mean_ms"], r["world_tick_sd_ms"]),
            mean_sd(r["terasim_internal_mean_ms"], r["terasim_internal_sd_ms"]),
            mean_sd(r["feedback_ingestion_mean_ms"], r["feedback_ingestion_sd_ms"]),
            mean_sd(r["sumo_step_mean_ms"], r["sumo_step_sd_ms"]),
            mean_sd(r["behavior_mean_ms"], r["behavior_sd_ms"]),
            mean_sd(r["state_export_mean_ms"], r["state_export_sd_ms"]),
        ]
        feedback_lines.append("| " + " | ".join(feedback_cells) + " |")
    feedback_lines.extend([
        "",
        "## 表記とfeedback ingestionの定義",
        "",
        "数値の統計表記と計測条件は[元のsummary](summary.md)と同じです。時間と車両数は、特記がない限りwarm-up後100 stepの `mean ± sample SD` です。totalは `mean ± SD/p95`、physicsは `mean ± SD/max` です。",
        "",
        "`feedback ingestion ms`はTeraSim内部時間の一部で、CARLAから受け取った各physics車両の実位置・実速度をSUMOへ反映する処理時間です。command解析、route-awareなlane/lanePos投影、`moveTo`、`setPreviousSpeed`、lane-change skip判定、cache・profile bookkeepingを含みます。したがって、`TeraSim + feedback ingestion`と加算すると二重計上になります。",
        "",
        "この表ではfeedback対象数の増加と処理時間の関係を見やすくするため、元の表の`exported`と`detail`を除外しています。その他の列の詳細な定義は[元のsummary](summary.md)を参照してください。",
    ])
    (root/"summary_feedback.md").write_text("\n".join(feedback_lines)+"\n")
    (root/"summary.md").write_text("\n".join(lines)+"\n"); print("\n".join(lines))
if __name__ == "__main__": main(sys.argv[1])
