#!/bin/bash
set -e
umask 000

# Maximum simulation time in seconds (warmup + run_time + buffer)
MAX_SIM_TIME=${MAX_SIM_TIME:-300}

echo "=========================================="
echo " TeraSim-CARLA Odaiba LL2 Co-Simulation"
echo "  Max time: ${MAX_SIM_TIME}s"
echo "=========================================="

# ─── Step 1: Start Redis ─────────────────────────────────────────────────────
echo "[1/4] Starting Redis server..."
redis-server &

# ─── Step 2: Start TeraSim Server ────────────────────────────────────────────
echo "[2/4] Starting TeraSim server..."
python -m terasim_service &
TERASIM_PID=$!

# ─── Step 3: Wait for server ─────────────────────────────────────────────────
echo "[3/4] Waiting 30s for TeraSim server to be ready..."
sleep 30

# ─── Step 4: Check CARLA and start co-simulation ─────────────────────────────
echo "[4/4] Starting simulation..."
echo "------------------------------------------"

# Check if CARLA is reachable
CARLA_AVAILABLE=false
python3 -c "
import carla
client = carla.Client('localhost', ${CARLA_PORT:-2010})
client.set_timeout(5.0)
version = client.get_server_version()
print(f'CARLA server version: {version}')
" 2>/dev/null && CARLA_AVAILABLE=true || true

if [ "$CARLA_AVAILABLE" = true ]; then
    echo "  CARLA is available! Running CARLA co-simulation..."

    # Check if OpenDRIVE file exists for CARLA
    if [ -f /app/examples/maps/odaiba_ll2/odaiba_ll2_to_xodr.xodr ]; then
        echo "  Loading Odaiba LL2 OpenDRIVE map into CARLA..."

        # Load OpenDRIVE map into CARLA using generate_opendrive_world
        # Note: Large xodr files can take 10+ minutes to load.
        # generate_opendrive_world may fail with "failed to connect to newly created map"
        # because CARLA is still building the mesh. We retry the connection after waiting.
        python3 -c "
import carla
import time
import sys

CARLA_PORT = ${CARLA_PORT:-2010}
XODR_PATH = '/app/examples/maps/odaiba_ll2/odaiba_ll2_to_xodr.xodr'

client = carla.Client('localhost', CARLA_PORT)
client.set_timeout(600.0)

# Check if the map is already loaded (e.g., from a previous run)
try:
    world = client.get_world()
    current_map = world.get_map()
    if 'OpenDrive' in current_map.name or 'opendrive' in current_map.name.lower():
        print('OpenDRIVE map is already loaded in CARLA. Skipping reload.')
        sys.exit(0)
except Exception:
    pass

# Read the OpenDRIVE file
with open(XODR_PATH, 'r') as f:
    xodr_data = f.read()

params = carla.OpendriveGenerationParameters(
    vertex_distance=10.0,
    max_road_length=500.0,
    wall_height=0.0,
    additional_width=0.6,
    smooth_junctions=True,
    enable_mesh_visibility=True,
)

print('Sending OpenDRIVE to CARLA (this may take 10+ minutes for large maps)...')
try:
    world = client.generate_opendrive_world(xodr_data, params)
    print('Odaiba LL2 OpenDRIVE map loaded into CARLA!')
except RuntimeError as e:
    print(f'Initial load returned error: {e}')
    print('CARLA may still be generating the map. Waiting and retrying connection...')
    # Wait for CARLA to finish building the map, retrying periodically
    for attempt in range(20):  # up to ~10 minutes (20 x 30s)
        time.sleep(30)
        try:
            client2 = carla.Client('localhost', CARLA_PORT)
            client2.set_timeout(30.0)
            world = client2.get_world()
            map_name = world.get_map().name
            print(f'Connected! Map: {map_name} (attempt {attempt+1})')
            break
        except Exception as retry_e:
            print(f'  Retry {attempt+1}/20: CARLA not ready yet ({retry_e})')
    else:
        print('ERROR: Could not connect to CARLA after waiting. Continuing without CARLA map.')
        sys.exit(1)
" 2>&1 || echo "  Warning: Failed to load OpenDRIVE into CARLA."

        echo "  Waiting for CARLA world to stabilize..."
        sleep 10
    fi

    echo "  Config: /app/examples/scenarios/cosim_odaiba_ll2.yaml"
    echo "------------------------------------------"

    # Run CARLA co-simulation (timeout=0 disables timeout for large xodr maps)
    python3 /app/examples/scripts/carla_cosim_main.py \
        --terasim_config /app/examples/scenarios/cosim_odaiba_ll2.yaml \
        --carla_port ${CARLA_PORT:-2010} \
        --carla_timeout 600 \
        --async_mode || true
    CARLA_EXIT=$?

    echo "=========================================="
    echo " CARLA client exited (code: $CARLA_EXIT)"
else
    echo "  CARLA is NOT available. Running TeraSim-only simulation..."
    echo "  Config: /app/examples/scenarios/cosim_odaiba_ll2.yaml"
    echo "------------------------------------------"

    # Run TeraSim-only simulation via HTTP API
    python3 -c "
import requests
import time
import json

TERASIM_HOST = 'localhost'
TERASIM_PORT = 8000
BASE_URL = f'http://{TERASIM_HOST}:{TERASIM_PORT}'

print('Starting TeraSim-only simulation...')

# Initialize simulation
init_data = {
    'config_file': '/app/examples/scenarios/cosim_odaiba_ll2.yaml',
    'auto_run': False,
}

try:
    resp = requests.post(f'{BASE_URL}/start_simulation', json=init_data, timeout=30)
    sim_info = resp.json()
    sim_id = sim_info.get('simulation_id', 'default')
    print(f'Simulation started: {sim_id}')
except Exception as e:
    print(f'Failed to start simulation: {e}')
    import sys; sys.exit(1)

# Wait for simulation to be ready
time.sleep(5)

# Tick simulation
MAX_STEPS = 6000  # 600s at 0.1s per step
for step in range(MAX_STEPS):
    try:
        # Check status
        status_resp = requests.get(f'{BASE_URL}/simulation_status/{sim_id}', timeout=10)
        status = status_resp.json()

        if status.get('status') == 'finished':
            print(f'Simulation finished at step {step}')
            break

        if status.get('status') in ['wait_for_tick', 'ticked', 'started', 'running']:
            # Tick
            tick_resp = requests.post(f'{BASE_URL}/simulation_tick/{sim_id}', timeout=30)

            if step % 100 == 0:
                print(f'Step {step}/{MAX_STEPS}')
        else:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('Interrupted by user.')
        break
    except Exception as e:
        if step % 100 == 0:
            print(f'Step {step}: {e}')
        time.sleep(0.1)

# Stop simulation
try:
    requests.post(f'{BASE_URL}/simulation_control/{sim_id}', json={'command': 'stop'}, timeout=10)
    print('Simulation stopped.')
except:
    pass

print('TeraSim-only simulation complete.')
" 2>&1
fi

echo " Stopping TeraSim server..."
echo "=========================================="

kill $TERASIM_PID 2>/dev/null || true
wait $TERASIM_PID 2>/dev/null || true

echo "=========================================="
echo " Simulation complete. Container exiting."
echo "=========================================="
