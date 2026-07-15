"""Typed configuration shared by the TeraSim and CARLA co-sim processes."""

from __future__ import annotations

import json
import logging
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

LOGGER = logging.getLogger(__name__)


class ActorScopeConfig(BaseModel):
    """Limit synchronized vehicles to a radius around a center actor."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool = True
    center_id: str = "AV"
    radius_m: float = Field(default=300.0, ge=0.0)


class BatchConfig(BaseModel):
    """CARLA RPC batching switches."""

    model_config = ConfigDict(extra="forbid")

    transform_updates: bool = True
    actor_spawns: bool = True


class SpawnConfig(BaseModel):
    """CARLA actor spawn settings."""

    model_config = ConfigDict(extra="forbid")

    z_clearance_m: float = Field(default=5.0, ge=0.0)


class BackoffConfig(BaseModel):
    """Exponential backoff after a CARLA actor spawn failure."""

    model_config = ConfigDict(extra="forbid")

    initial_seconds: float = Field(default=5.0, ge=0.0)
    max_seconds: float = Field(default=30.0, ge=0.0)

    @model_validator(mode="after")
    def clamp_max_to_initial(self) -> "BackoffConfig":
        # Preserve the existing behavior when max is configured below the initial delay.
        if self.max_seconds < self.initial_seconds:
            self.max_seconds = self.initial_seconds
        return self


class CosimConfig(BaseModel):
    """Behavioral co-simulation settings from the scenario ``cosim`` section."""

    model_config = ConfigDict(extra="forbid")

    actor_scope: ActorScopeConfig = Field(default_factory=ActorScopeConfig)
    lane_relative_position: bool = True
    batch: BatchConfig = Field(default_factory=BatchConfig)
    spawn: SpawnConfig = Field(default_factory=SpawnConfig)
    backoff: BackoffConfig = Field(default_factory=BackoffConfig)
    idle_state_write_interval_seconds: float = Field(default=0.5, ge=0.0)


def load_cosim_config(scenario: Mapping[str, Any] | None) -> CosimConfig:
    """Load typed behavioral settings from the scenario YAML or model defaults."""

    raw_scenario = scenario or {}
    raw_cosim = raw_scenario.get("cosim", {})
    if raw_cosim is None:
        raw_cosim = {}
    if not isinstance(raw_cosim, Mapping):
        raise TypeError("scenario cosim section must be a mapping")
    return CosimConfig.model_validate(raw_cosim)


def log_effective_cosim_config(config: CosimConfig, logger: Any = LOGGER) -> None:
    """Log the fully resolved behavioral configuration as a single JSON object."""

    payload = json.dumps(config.model_dump(mode="json"), sort_keys=True)
    logger.info("Effective co-sim config: %s", payload)


def save_effective_cosim_config(config: CosimConfig, output_dir: str | Path) -> Path:
    """Persist the fully resolved configuration in a simulation output directory."""

    path = Path(output_dir) / "cosim_effective_config.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {"cosim": config.model_dump(mode="json")},
            stream,
            sort_keys=False,
        )
    return path
