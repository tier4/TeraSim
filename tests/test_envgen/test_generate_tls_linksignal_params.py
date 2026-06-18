import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def load_generator_module():
    script_path = Path(__file__).parents[2] / "scripts" / "generate_tls_linksignal_params.py"
    spec = importlib.util.spec_from_file_location("generate_tls_linksignal_params", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_synthetic_inputs(tmp_path, tls_id="tls_a"):
    net_path = tmp_path / "input.net.xml"
    net_path.write_text(
        f"""<?xml version="1.0" encoding="utf-8"?>
<net>
  <edge id="incoming_a" from="upstream_a" to="node_a">
    <lane id="incoming_a_0" index="0" speed="13.89" length="10.0" />
  </edge>
  <edge id="incoming_b" from="upstream_b" to="node_b">
    <lane id="incoming_b_0" index="0" speed="13.89" length="10.0" />
  </edge>
  <edge id="out_a" from="node_a" to="downstream_a">
    <lane id="out_a_0" index="0" speed="13.89" length="10.0" />
  </edge>
  <edge id="out_b" from="node_b" to="downstream_b">
    <lane id="out_b_0" index="0" speed="13.89" length="10.0" />
  </edge>
  <tlLogic id="{tls_id}" type="static" programID="0" offset="0">
    <phase duration="10" state="Gr" />
    <param key="linkSignalID:0" value="legacy" />
  </tlLogic>
  <connection from="incoming_a" to="out_a" via=":node_a_0_0" tl="{tls_id}" linkIndex="0" />
  <connection from="incoming_b" to="out_b" via=":node_b_0_0" tl="{tls_id}" linkIndex="1" />
</net>
""",
        encoding="utf-8",
    )

    route_path = tmp_path / "vehicles.rou.xml"
    route_path.write_text("<routes />\n", encoding="utf-8")

    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text('{"map": "synthetic"}\n', encoding="utf-8")

    sumocfg_path = tmp_path / "simulation.sumocfg"
    sumocfg_path.write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<configuration>
  <input>
    <net-file value="input.net.xml" />
    <route-files value="vehicles.rou.xml" />
    <step-length value="0.1" />
  </input>
</configuration>
""",
        encoding="utf-8",
    )

    signal_mapping_path = tmp_path / "signal_id_mapping.json"
    signal_mapping_path.write_text(
        json.dumps(
            {
                "lanelet_to_sumo": [
                    {
                        "actual_sumo_tls_ids": [],
                        "lanelet_regulatory_element_ids": ["reg_a"],
                        "lanelet_traffic_light_way_ids": [],
                        "planned_sumo_node_ids": ["node_a"],
                        "planned_sumo_tls_id": tls_id,
                        "resolution_status": "mapped",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    opendrive_mapping_path = tmp_path / "odaiba_tl_mapping_test.mapping.json"
    opendrive_mapping_path.write_text(
        json.dumps(
            {
                "traffic_light_signal_mapping": {
                    "lanelet2_tl_id_to_signal_ids": {
                        "reg_a": [2000466, 467],
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    return net_path, signal_mapping_path, opendrive_mapping_path, sumocfg_path, route_path


def test_generate_linksignal_params_writes_od_tokens_and_sumocfg(tmp_path):
    module = load_generator_module()
    net_path, signal_mapping_path, od_mapping_path, sumocfg_path, route_path = (
        write_synthetic_inputs(tmp_path)
    )
    output_net = tmp_path / "out" / "tls_synced.net.xml"
    output_sumocfg = tmp_path / "out" / "simulation_tls_synced.sumocfg"
    report_path = tmp_path / "out" / "report.json"

    report = module.generate_tls_linksignal_params(
        sumo_net=net_path,
        signal_id_mapping=signal_mapping_path,
        opendrive_lanelet_mapping=od_mapping_path,
        sumocfg=sumocfg_path,
        output_net=output_net,
        output_sumocfg=output_sumocfg,
        report_path=report_path,
        min_coverage=0.0,
    )

    tl_logic = ET.parse(output_net).getroot().find("tlLogic")
    params = {param.get("key"): param.get("value") for param in tl_logic.findall("param")}
    assert params == {"linkSignalID:0": "od:466 od:467"}
    assert report["records_mapped_to_linksignal"] == 1

    generated_sumocfg = ET.parse(output_sumocfg).getroot()
    assert generated_sumocfg.find("input/net-file").get("value") == str(output_net)
    assert generated_sumocfg.find("input/route-files").get("value") == str(
        route_path.resolve()
    )
    assert generated_sumocfg.find("input/step-length").get("value") == "0.1"
    assert (output_net.parent / "metadata.json").read_text(encoding="utf-8") == '{"map": "synthetic"}\n'


def test_generate_linksignal_params_prefers_actual_tls_id_when_present(tmp_path):
    module = load_generator_module()
    net_path, signal_mapping_path, od_mapping_path, sumocfg_path, _ = write_synthetic_inputs(
        tmp_path, tls_id="actual_tls"
    )
    mapping = json.loads(signal_mapping_path.read_text(encoding="utf-8"))
    mapping["lanelet_to_sumo"][0]["actual_sumo_tls_ids"] = ["actual_tls"]
    mapping["lanelet_to_sumo"][0]["planned_sumo_tls_id"] = "planned_tls"
    signal_mapping_path.write_text(json.dumps(mapping), encoding="utf-8")

    output_net = tmp_path / "out" / "tls_synced.net.xml"
    report = module.generate_tls_linksignal_params(
        sumo_net=net_path,
        signal_id_mapping=signal_mapping_path,
        opendrive_lanelet_mapping=od_mapping_path,
        sumocfg=sumocfg_path,
        output_net=output_net,
        output_sumocfg=tmp_path / "out" / "simulation_tls_synced.sumocfg",
        report_path=tmp_path / "out" / "report.json",
        min_coverage=0.0,
    )

    assert report["target_tls_source_counts"] == {"actual_sumo_tls_ids": 1}
    tl_logic = ET.parse(output_net).getroot().find("tlLogic[@id='actual_tls']")
    assert tl_logic.find("param[@key='linkSignalID:0']").get("value") == "od:466 od:467"


def test_generate_linksignal_params_odaiba_dry_run_has_high_link_index_coverage(tmp_path):
    module = load_generator_module()
    repo_root = Path(__file__).parents[2]
    mapping_dir = repo_root / "examples" / "maps" / "odaiba_ll2" / "tlmappings"
    od_mapping_paths = list(mapping_dir.glob("odaiba_tl_mapping_*.mapping.json"))
    assert len(od_mapping_paths) == 1

    output_net = tmp_path / "odaiba_osmlike_network3_tls_synced.net.xml"
    output_sumocfg = tmp_path / "simulation_tls_synced.sumocfg"
    report_path = tmp_path / "tls_linksignal_report.json"
    report = module.generate_tls_linksignal_params(
        sumo_net=(
            repo_root
            / "examples"
            / "maps"
            / "odaiba_ll2"
            / "odaiba_osmlike_network3.net.xml"
        ),
        signal_id_mapping=mapping_dir / "signal_id_mapping.json",
        opendrive_lanelet_mapping=od_mapping_paths[0],
        sumocfg=repo_root / "examples" / "maps" / "odaiba_ll2" / "simulation.sumocfg",
        output_net=output_net,
        output_sumocfg=output_sumocfg,
        report_path=report_path,
        min_coverage=0.90,
    )

    assert report["coverage"] >= 0.90
    assert report["invalid_link_index_count"] == 0
    assert report["skipped_by_reason"]["unmapped_resolution_status"] == 11

    root = ET.parse(output_net).getroot()
    for tl_logic in root.findall("tlLogic"):
        phase_lengths = [len(phase.get("state", "")) for phase in tl_logic.findall("phase")]
        if not phase_lengths:
            continue
        max_phase_length = max(phase_lengths)
        for param in tl_logic.findall("param"):
            key = param.get("key", "")
            if key.startswith("linkSignalID:"):
                assert int(key.split(":", 1)[1]) < max_phase_length
