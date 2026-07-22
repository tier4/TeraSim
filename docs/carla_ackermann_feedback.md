# CARLA Ackermann feedback co-simulation

This mode keeps selected CARLA vehicles under physics-based Ackermann control and writes their
observed pose and speed back to TeraSim before the next SUMO step.

## Enable the loop

Use synchronous CARLA mode and set:

```bash
export CARLA_COSIM_VEHICLE_CONTROL_MODE=ackermann_physics
export CARLA_COSIM_ACKERMANN_FEEDBACK_MODE=apply
export CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='*,AV'
```

`*` selects all background vehicles; `AV` must be listed explicitly. `shadow` may be used instead
of `apply` to observe and log feedback without changing SUMO state.

The normal HTTP/Redis path queues one ordered command batch before requesting the next TeraSim
tick. For the direct gRPC path, pass `--direct_addr HOST:PORT`; the same ordered `set_state`
commands are carried in the next `Tick` request.

## Constraints

- CARLA must be synchronous.
- The CARLA client must own `world.tick()`; `passive_tick` is unsupported.
- `control_av` cannot be combined with feedback ownership of the AV.
- Feedback mode requires `ackermann_physics`.

Each feedback command contains the CARLA frame number. TeraSim exposes the accepted observed
speed and frame in the next simulation state, allowing the controller to detect missing or stale
feedback and brake safely.
