# CARLA Co-Sim Actor Sync Bottleneck Investigation

Date: 2026-05-05

## Summary

Odaiba packaged CARLA co-simulation is not keeping up with the configured
10 Hz simulation step in wall-clock time. The main bottleneck is not CARLA
rendering, noVNC, socket buffer limits, SUMO stepping, or HTTP state retrieval.

The dominant cost is CARLA actor lookup inside `sync_cosim_actor_to_carla()`.
With roughly 353 vehicles, the co-sim loop performs one role-name lookup per
vehicle each tick. Each lookup calls `world.get_actors()` and scans actors,
making lookup cost about 0.59 s per tick by itself.

## Key Finding

Detailed actor sync profiler:

```text
state_get median             0.0121 s
vehicle_loop median          0.6118 s
lookup_total median          0.5869 s
cleanup_vehicle median       0.0015 s
cleanup_pedestrian median    0.0005 s
sumo_to_carla_total median   0.0013 s
total median                 0.6253 s
vehicle_count median       353
lookup_calls median        353
```

This means almost all of the actor sync time is spent in actor lookup:

```text
lookup_total / total ~= 0.5869 / 0.6253 = 94%
```

At 10 Hz, each tick must complete in less than 0.1 s. The current median actor
sync time is already about 0.63 s, so real-time 10 Hz execution is impossible
with the current lookup pattern.

## Observed Runtime Behavior

External CARLA actor sampler confirmed that simulation time advances at 10 Hz,
but wall-clock tick delivery is much slower.

Example run:

```text
World settings: synchronous_mode=True fixed_delta_seconds=0.1
wall_hz 1.632
sim_hz 10.0
wall_dt median 0.670 s
backward 0
```

Another profiled run:

```text
external wall_hz 3.366
sim_hz           10.0
```

So the simulation step is configured as 10 Hz, but CARLA frames/ticks are not
being produced at 10 Hz in wall-clock time.

## Bottleneck Breakdown

Tick profiler:

```text
status_wait median    0.003 s
sync_actor median     0.572 s
sync_tls median       0.013 s
tick_terasim median   0.002 s
world_tick median     0.031 s
total median          0.623 s
```

Interpretation:

- `status_wait` is small, so waiting for TeraSim status is not the bottleneck.
- `tick_terasim` is small, so advancing SUMO/TeraSim one step is not the bottleneck.
- `world_tick` is small, so CARLA synchronous tick/render is not the bottleneck in this measurement.
- `sync_actor` dominates the tick time.

Detailed actor profiler then showed that `sync_actor` is dominated by actor lookup.

## Code Path

The expensive path is in:

```text
packages/terasim-service/terasim_service/utils/carla/cosim.py
```

`sync_cosim_actor_to_carla()` loops through all SUMO vehicles:

```python
for veh_id in terasim_states["agent_details"]["vehicle"]:
    self._process_vehicle(...)
```

`_process_vehicle()` calls:

```python
vehicle_status, carla_id = get_actor_id_from_attribute(self.world, veh_id)
```

`get_actor_id_from_attribute()` scans CARLA actors:

```python
def get_actor_id_from_attribute(world, attribute):
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.attributes.get("role_name") == attribute:
            return True, actor.id
    return False, -1
```

With about 353 vehicles, this causes about 353 `world.get_actors()` calls and
actor-list scans per tick.

## Hypotheses Checked

### Vehicle Actually Moving Backward

Mostly no. The sampler usually recorded no backward movement:

```text
backward_events=0
```

One clean no-follow run did record four small backward steps:

```text
signed_delta ~= -0.072 m for 4 ticks
```

That appeared around a yaw discontinuity, so it is likely a separate route/angle
transition issue rather than the main performance bottleneck.

### noVNC / Browser FPS

Not the main bottleneck based on profiling. noVNC display can make the issue
more visible, but `world_tick` is not the dominant cost and `sync_actor` remains
large.

### CARLA Rendering

Unlikely to be the main bottleneck in the measured runs:

```text
world_tick median ~= 0.028-0.031 s
```

### SUMO / TeraSim Step

Unlikely to be the main bottleneck:

```text
tick_terasim median ~= 0.002 s
```

### HTTP State Retrieval

Unlikely to be the main bottleneck:

```text
state_get median ~= 0.012 s
```

### Socket Buffer / TCP Backlog

Current evidence is weak for kernel socket buffer saturation.

Observed `ss` queues:

```text
CARLA RPC :2000  Recv-Q=26 bytes, Send-Q=0
noVNC :6080      Recv-Q=0,  Send-Q=0
```

Observed `nstat` did not show backlog/listen overflow:

```text
TcpExtListenOverflows  0
TcpExtTCPBacklogDrop   0
TcpExtOfoPruned        0
```

Also, inside `carla-novnc-test`, defaults were already very large:

```text
net.core.wmem_default = 2147483647
net.core.rmem_default = 2147483647
```

Socket/RPC overhead can still contribute, but the measured dominant cost is the
per-vehicle actor lookup pattern.

## Recommended Fix

Cache CARLA actors by `role_name` once per tick instead of calling
`world.get_actors()` for every vehicle.

Current pattern:

```text
for each SUMO vehicle:
    world.get_actors()
    scan all actors for role_name
```

Recommended pattern:

```python
actors_by_role = {
    actor.attributes.get("role_name"): actor
    for actor in world.get_actors().filter("vehicle.*")
}
```

Then each vehicle can use a dictionary lookup:

```python
vehicle = actors_by_role.get(veh_id)
```

Expected impact:

- Remove most of `lookup_total median 0.5869 s`.
- Reduce `sync_actor` from about 0.62 s toward the remaining per-tick costs.
- If `set_transform` for 353 vehicles is still expensive, the next optimization
  should be batching transforms or limiting synchronized actors.

## Next Verification After Fix

Run co-sim with:

```bash
CARLA_COSIM_PROFILE_LOG=/app/outputs/fixed_tick_profile.csv \
CARLA_COSIM_ACTOR_PROFILE_LOG=/app/outputs/fixed_actor_profile.csv \
CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

External sampler:

```bash
ROLE_NAME=AV \
DURATION=45 \
OUTPUT_CSV=outputs/fixed_external_motion.csv \
./scripts/record_carla_actor_motion.sh
```

Success criteria:

```text
lookup_total median      near 0
sync_actor median        much lower than 0.6 s
total median             ideally < 0.1 s for real-time 10 Hz
external wall_hz         closer to 10 Hz
backward_events          0, or separately investigated if yaw discontinuity recurs
```
