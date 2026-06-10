# Private CARLA noVNC Desktop

This is the self-contained route that does not require `sudo apt` on the host.
noVNC, VNC, Xvfb, Openbox, and the Python dependencies for `manual_control.py` are
built into the CARLA image itself.

For the full Odaiba packaged-map plus `odaiba_osmlike_network3.net.xml` to TeraSim-CARLA co-sim
workflow, see [odaiba_packaged_carla_cosim.md](odaiba_packaged_carla_cosim.md).

## Version note

The noVNC image uses `carlasim/carla:0.9.16`, which is aligned with the
`carla==0.9.16` Python client used by the repo's co-simulation image.

Note: CARLA's official docs for 0.9.16 treat Docker execution differently from older
versions, and CARLA 0.9.12+ runs on the Vulkan-only Unreal Engine 4.26 line.
For that reason this setup no longer passes `-opengl`.

## File layout

Place the OpenDRIVE file under:

```text
examples/maps/odaiba_ll2/odaiba_carla.xodr
```

That directory is mounted into the container as:

```text
/home/carla/workspace
```

## Build

```bash
docker compose -f docker-compose.carla-novnc.yml build --no-cache
```

This build now compiles `python3.10` inside the CARLA image so that the bundled
`carla-0.9.16` wheel can be imported from the same container. Expect the first
build to take noticeably longer than before.

## Start

This exposes:

- noVNC: `6092`
- VNC: `5912`
- CARLA RPC/streaming: `2010-2012`

```bash
docker rm -f carla-novnc-test 2>/dev/null || true
docker compose -f docker-compose.carla-novnc.yml up -d
```

Check that it is up:

```bash
docker ps --filter name=carla-novnc-test
docker logs --tail=30 carla-novnc-test
```

If rendering is unexpectedly slow, verify that CARLA is not falling back to
`lavapipe` software Vulkan:

```bash
docker exec carla-novnc-test nvidia-smi
docker logs --tail=100 carla-novnc-test | grep -i lavapipe || true
docker exec carla-novnc-test bash -lc 'echo "$VK_ICD_FILENAMES"; ls -l /usr/share/vulkan/icd.d/nvidia_icd.json'
```

If `docker logs` prints `lavapipe`, CARLA is not using the NVIDIA Vulkan ICD.
This Compose file now mounts the host's NVIDIA Vulkan JSON and sets
`VK_ICD_FILENAMES` explicitly to avoid that fallback.

## View from your laptop

```bash
ssh -N -L 6092:localhost:6092 <user>@<server>
```

Then open:

```text
http://localhost:6092/vnc.html
```

Password:

```text
headless
```

## Generate an OpenDRIVE world

The image already contains `numpy` and `pygame`, so `manual_control.py` is usable.
This container also installs a compatible `python3.10` and the bundled
`carla-0.9.16` wheel during image build, so use `python3.10` directly:

```bash
C=carla-novnc-test
docker exec -it $C bash -lc '
python3.10 - <<PY
import pathlib, carla
xodr = pathlib.Path("/home/carla/workspace/odaiba_carla.xodr").read_text()

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(600.0)

params = carla.OpendriveGenerationParameters(
    vertex_distance=1.0,
    max_road_length=20.0,
    wall_height=0.0,
    additional_width=1.0,
    smooth_junctions=False,
    enable_mesh_visibility=True,
    enable_pedestrian_navigation=False,
)

world = client.generate_opendrive_world(xodr, params)
print("generated map:", world.get_map().name)
print("spawn points:", len(world.get_map().get_spawn_points()))
PY
'
```

There is also a simpler fallback:

```bash
C=carla-novnc-test
docker exec -it $C bash -lc '
python3.10 - <<PY
import pathlib, carla
xodr = pathlib.Path("/home/carla/workspace/odaiba_carla.xodr").read_text()

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(600.0)

world = client.generate_opendrive_world(xodr)
print("generated map:", world.get_map().name)
print("spawn points:", len(world.get_map().get_spawn_points()))
PY
'
```

## Visual check with manual control

```bash
C=carla-novnc-test
docker exec -e DISPLAY=:1 -it $C bash -lc '
cd /workspace/PythonAPI/examples
python3.10 manual_control.py --host 127.0.0.1 --port 2000
'
```

or:

```bash
C=carla-novnc-test
docker exec -e DISPLAY=:1 -it $C bash -lc '
cd /workspace/PythonAPI/examples
python3.10 manual_control.py --host 127.0.0.1 --port 2000 --filter "vehicle.ford.crown" --generation 2
'
```

## Optional: start TeraSim afterward

If you still want to try the repo's co-simulation container against this CARLA server:

```bash
CARLA_PORT=2010 docker compose -f docker-compose.cosim-odaiba-ll2.yml up --build
```

This now uses the same CARLA version as the co-simulation image, but the visual noVNC
path itself is still worth smoke-testing on your server before relying on it.
