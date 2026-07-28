#!/usr/bin/env python3
"""Prepare a deterministic period=0.2 Odaiba SUMO cache and co-sim config."""
from __future__ import annotations
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

def write_sumocfg(
    path: Path,
    *,
    net_file: str,
    route_file: str,
    step: float,
    end: float,
    threads: int,
    load_state: str = "",
    begin: float = 0.0,
) -> None:
    root = ET.Element("configuration")
    inputs = ET.SubElement(root, "input")
    ET.SubElement(inputs, "net-file", value=net_file)
    if route_file:
        ET.SubElement(inputs, "route-files", value=route_file)
    if load_state:
        ET.SubElement(inputs, "load-state", value=load_state)
    ET.SubElement(inputs, "step-length", value=str(step))
    times = ET.SubElement(root, "time")
    ET.SubElement(times, "begin", value=str(begin))
    ET.SubElement(times, "end", value=str(end))
    processing = ET.SubElement(root, "processing")
    ET.SubElement(processing, "threads", value=str(threads))
    ET.SubElement(processing, "lateral-resolution", value="0.5")
    ET.SubElement(processing, "time-to-teleport", value="-1")
    ET.SubElement(processing, "collision.action", value="warn")
    ET.SubElement(processing, "collision.check-junctions", value="true")
    ET.SubElement(processing, "collision.mingap-factor", value="0")
    report = ET.SubElement(root, "report")
    ET.SubElement(report, "verbose", value="false")
    ET.SubElement(report, "no-step-log", value="true")
    ET.indent(root, space="  ")
    path.write_text('<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="unicode") + "\n")

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--container-output-dir", required=True)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--route-file", required=True)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--cache-time", type=float, default=500.0)
    parser.add_argument("--sumo-threads", type=int, default=8)
    args = parser.parse_args()
    if args.sumo_threads < 1:
        parser.error("--sumo-threads must be at least 1")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    generation_name = "sumo_cache_generation.sumocfg"
    runtime_name = "odaiba_period_0p2_cached_t500.sumocfg"
    state_name = "sumo_state_500.xml.gz"
    config_name = "cosim_odaiba_period_0p2_cached_t500.yaml"
    container_dir = args.container_output_dir.rstrip("/")
    write_sumocfg(
        output_dir / generation_name,
        net_file=args.net_file,
        route_file=args.route_file,
        step=args.step_length,
        end=args.cache_time + args.step_length,
        threads=args.sumo_threads,
    )
    write_sumocfg(
        output_dir / runtime_name,
        net_file=args.net_file,
        route_file="",
        step=args.step_length,
        end=3600.0,
        threads=args.sumo_threads,
        load_state=f"{container_dir}/{state_name}",
        begin=args.cache_time,
    )
    with open(args.base_config, encoding="utf-8") as source:
        config = yaml.safe_load(source)
    runtime_cfg = f"{container_dir}/{runtime_name}"
    config["input"]["sumo_config_file"] = runtime_cfg
    config["environment"]["parameters"]["sumo_cfg_file_path"] = runtime_cfg
    config["environment"]["parameters"]["warmup_time_lb"] = args.cache_time
    config["environment"]["parameters"]["warmup_time_ub"] = args.cache_time + 1
    config["simulator"]["parameters"]["gui_flag"] = False
    output_types = config["simulator"]["parameters"].get("sumo_output_file_types") or []
    config["simulator"]["parameters"]["sumo_output_file_types"] = [
        output_type for output_type in output_types if output_type not in {"fcd", "fcd_all"}
    ]
    config["simulator"]["parameters"]["realtime_flag"] = False
    config["output"]["dir"] = "/app/outputs"
    with open(output_dir / config_name, "w", encoding="utf-8") as target:
        yaml.safe_dump(config, target, sort_keys=False)
    print(f"generation_config={container_dir}/{generation_name}")
    print(f"runtime_config={container_dir}/{config_name}")
    print(f"state_file={container_dir}/{state_name}")

if __name__ == "__main__":
    main()
