import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


def load_filter_module():
    script_path = Path(__file__).parents[2] / "scripts" / "filter_sumo_routes_by_av_corridor.py"
    spec = importlib.util.spec_from_file_location("filter_sumo_routes_by_av_corridor", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeEdge:
    def __init__(self, edge_id, shape, special=False):
        self.edge_id = edge_id
        self.shape = shape
        self.special = special

    def getID(self):
        return self.edge_id

    def getShape(self):
        return self.shape

    def isSpecial(self):
        return self.special


class FakeNet:
    def __init__(self, edges):
        self.edges = {edge.getID(): edge for edge in edges}

    def getEdge(self, edge_id):
        return self.edges[edge_id]

    def getEdges(self):
        return list(self.edges.values())


def make_routes_xml(path):
    path.write_text(
        """<?xml version="1.0" encoding="utf-8"?>
<routes>
  <vType id="vehicle_passenger" vClass="passenger" />
  <vehicle id="kept_start_outside" type="vehicle_passenger" depart="0.00">
    <route edges="far_start av_core far_end" />
  </vehicle>
  <vehicle id="kept_parallel" type="vehicle_passenger" depart="1.00">
    <route edges="parallel" />
  </vehicle>
  <route id="shared_keep" edges="ref_start av_core" />
  <vehicle id="kept_route_ref" type="vehicle_passenger" depart="2.00" route="shared_keep" />
  <vehicle id="removed_unrelated" type="vehicle_passenger" depart="3.00">
    <route edges="unrelated" />
  </vehicle>
  <route id="av_route" edges="av_core" />
</routes>
""",
        encoding="utf-8",
    )


def test_filter_keeps_full_routes_that_touch_av_corridor(tmp_path):
    module = load_filter_module()
    routes_path = tmp_path / "vehicles.rou.xml"
    output_path = tmp_path / "vehicles.filtered.rou.xml"
    make_routes_xml(routes_path)

    fake_net = FakeNet(
        [
            FakeEdge("far_start", [(-100.0, 0.0), (-90.0, 0.0)]),
            FakeEdge("av_core", [(0.0, 0.0), (10.0, 0.0)]),
            FakeEdge("far_end", [(90.0, 0.0), (100.0, 0.0)]),
            FakeEdge("parallel", [(0.0, 3.0), (10.0, 3.0)]),
            FakeEdge("ref_start", [(-50.0, 0.0), (-40.0, 0.0)]),
            FakeEdge("unrelated", [(100.0, 100.0), (110.0, 100.0)]),
        ]
    )

    report = module.filter_routes_by_av_corridor(
        net=fake_net,
        routes_path=routes_path,
        output_path=output_path,
        av_route_id="av_route",
        radius=5.0,
        seed=1,
    )

    root = ET.parse(output_path).getroot()
    vehicle_ids = [vehicle.get("id") for vehicle in root.findall("vehicle")]

    assert vehicle_ids == ["kept_start_outside", "kept_parallel", "kept_route_ref"]
    assert root.find("vType[@id='vehicle_passenger']") is not None
    assert root.find("route[@id='av_route']") is not None
    assert report["total_vehicles"] == 4
    assert report["kept_by_corridor"] == 3
    assert report["removed_vehicles"] == 1


def test_keep_unmatched_probability_can_retain_unrelated_routes(tmp_path):
    module = load_filter_module()
    routes_path = tmp_path / "vehicles.rou.xml"
    output_path = tmp_path / "vehicles.filtered.rou.xml"
    make_routes_xml(routes_path)

    fake_net = FakeNet(
        [
            FakeEdge("far_start", [(-100.0, 0.0), (-90.0, 0.0)]),
            FakeEdge("av_core", [(0.0, 0.0), (10.0, 0.0)]),
            FakeEdge("far_end", [(90.0, 0.0), (100.0, 0.0)]),
            FakeEdge("parallel", [(0.0, 3.0), (10.0, 3.0)]),
            FakeEdge("ref_start", [(-50.0, 0.0), (-40.0, 0.0)]),
            FakeEdge("unrelated", [(100.0, 100.0), (110.0, 100.0)]),
        ]
    )

    report = module.filter_routes_by_av_corridor(
        net=fake_net,
        routes_path=routes_path,
        output_path=output_path,
        av_route_id="av_route",
        radius=5.0,
        keep_unmatched_prob=1.0,
        seed=1,
    )

    root = ET.parse(output_path).getroot()
    vehicle_ids = [vehicle.get("id") for vehicle in root.findall("vehicle")]

    assert "removed_unrelated" in vehicle_ids
    assert report["kept_by_random"] == 1
    assert report["removed_vehicles"] == 0


def test_missing_av_route_raises_clear_error(tmp_path):
    module = load_filter_module()
    routes_path = tmp_path / "vehicles.rou.xml"
    routes_path.write_text("<routes><vehicle id='v0'><route edges='edge_a' /></vehicle></routes>")

    fake_net = FakeNet([FakeEdge("edge_a", [(0.0, 0.0), (1.0, 0.0)])])

    with pytest.raises(ValueError, match="av_route"):
        module.filter_routes_by_av_corridor(
            net=fake_net,
            routes_path=routes_path,
            output_path=tmp_path / "out.rou.xml",
            av_route_id="av_route",
            radius=5.0,
        )
