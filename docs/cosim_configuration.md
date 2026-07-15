# CARLA/TeraSim co-simulation configuration

Co-simulation behavior is configured in the scenario YAML's top-level `cosim`
section. CARLA and TeraSim load this section through the same Pydantic model, so a
single scenario describes both sides of the synchronization.

Connection and deployment values remain outside the scenario. Continue to use CLI
arguments or environment variables for values such as the CARLA host/port, TeraSim
host/port, direct gRPC address/port, and scenario file path.

## Schema and defaults

The operational defaults enable actor filtering and CARLA transform/spawn batching.
They can be disabled explicitly in each scenario YAML.

```yaml
cosim:
  actor_scope:
    enabled: true
    center_id: AV
    radius_m: 300.0
  lane_relative_position: true
  batch:
    transform_updates: true
    actor_spawns: true
  spawn:
    z_clearance_m: 5.0
  backoff:
    initial_seconds: 5.0
    max_seconds: 30.0
  idle_state_write_interval_seconds: 0.5
```

Unknown fields, negative distances, and negative intervals are rejected. If
`backoff.max_seconds` is below `initial_seconds`, it is clamped to the initial
delay to preserve the previous runtime behavior.

For a dense scenario where CARLA should only mirror actors near the ego and use
batched RPCs, for example:

```yaml
cosim:
  actor_scope:
    enabled: true
    center_id: AV
    radius_m: 300.0
  lane_relative_position: true
  batch:
    transform_updates: true
    actor_spawns: true
```

## Effective configuration

Both processes log the fully resolved configuration after applying scenario YAML and
model defaults. The TeraSim plugin also saves it as
`cosim_effective_config.yaml` in the individual simulation output directory next
to `terasim_cosim_plugin.log`.
