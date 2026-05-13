import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("sumolib")


def load_generator_module():
    script_path = Path(__file__).parents[2] / "scripts" / "generate_sumo_artifacts_from_net.py"
    spec = importlib.util.spec_from_file_location("generate_sumo_artifacts_from_net", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeEdge:
    def __init__(self, edge_id, special=False):
        self.edge_id = edge_id
        self.special = special

    def getID(self):
        return self.edge_id

    def isSpecial(self):
        return self.special


def test_filter_regular_route_edges_removes_internal_special_and_consecutive_duplicates():
    module = load_generator_module()

    route_edges = [
        FakeEdge("edge_755"),
        FakeEdge(":node_1195_2"),
        FakeEdge("edge_760"),
        FakeEdge("edge_760"),
        FakeEdge("edge_special", special=True),
        FakeEdge("edge_2381"),
        FakeEdge(":node_673_1"),
        FakeEdge("edge_2381"),
        FakeEdge("edge_2456"),
    ]

    regular_edges = module.filter_regular_route_edges(route_edges)

    assert [edge.getID() for edge in regular_edges] == [
        "edge_755",
        "edge_760",
        "edge_2381",
        "edge_2456",
    ]


def test_ensure_vtypes_and_av_route_writes_only_regular_route_edges(tmp_path):
    module = load_generator_module()
    routes_path = tmp_path / "vehicles.rou.xml"

    module.ensure_vtypes_and_av_route(
        routes_path,
        ["edge_755", ":node_1195_2", "edge_760", "edge_760"],
    )

    route = ET.parse(routes_path).getroot().find("route[@id='av_route']")
    assert route is not None
    assert route.get("edges") == "edge_755 edge_760"
