from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError
from terasim_service.cosim_config import (
    CosimConfig,
    load_cosim_config,
    save_effective_cosim_config,
)


def test_defaults_enable_actor_scope_and_batching():
    config = load_cosim_config({})

    assert config == CosimConfig()
    assert config.actor_scope.enabled is True
    assert config.lane_relative_position is True
    assert config.batch.transform_updates is True
    assert config.batch.actor_spawns is True
    assert config.spawn.z_clearance_m == 5.0
    assert config.backoff.initial_seconds == 5.0
    assert config.backoff.max_seconds == 30.0
    assert config.idle_state_write_interval_seconds == 0.5


def test_yaml_is_typed_and_shared_between_sides():
    scenario = {
        "cosim": {
            "actor_scope": {"enabled": True, "center_id": "ego", "radius_m": 125},
            "lane_relative_position": True,
            "batch": {"transform_updates": True, "actor_spawns": True},
            "spawn": {"z_clearance_m": 2.5},
            "backoff": {"initial_seconds": 1, "max_seconds": 8},
            "idle_state_write_interval_seconds": 0.2,
        }
    }

    config = load_cosim_config(scenario)

    assert config == CosimConfig.model_validate(scenario["cosim"])
    assert config.actor_scope.radius_m == 125.0
    assert config.backoff.max_seconds == 8.0


def test_yaml_can_disable_defaults():
    config = load_cosim_config(
        {
            "cosim": {
                "actor_scope": {"enabled": False},
                "batch": {"transform_updates": False, "actor_spawns": False},
            }
        }
    )

    assert config.actor_scope.enabled is False
    assert config.batch.transform_updates is False
    assert config.batch.actor_spawns is False


@pytest.mark.parametrize(
    "cosim",
    [
        {"unknown": True},
        {"actor_scope": {"radius_m": -1}},
        {"spawn": {"z_clearance_m": -1}},
        {"idle_state_write_interval_seconds": -1},
    ],
)
def test_invalid_yaml_is_rejected(cosim):
    with pytest.raises(ValidationError):
        load_cosim_config({"cosim": cosim})


def test_effective_config_is_saved(tmp_path: Path):
    config = load_cosim_config({"cosim": {"batch": {"actor_spawns": True}}})

    path = save_effective_cosim_config(config, tmp_path)

    assert path == tmp_path / "cosim_effective_config.yaml"
    assert yaml.safe_load(path.read_text(encoding="utf-8")) == {
        "cosim": config.model_dump(mode="json")
    }
