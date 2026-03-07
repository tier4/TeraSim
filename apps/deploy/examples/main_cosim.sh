#!/bin/bash
set -e
umask 000

# Maximum simulation time in seconds (warmup + run_time + buffer)
MAX_SIM_TIME=${MAX_SIM_TIME:-300}

echo "=========================================="
echo " TeraSim-CARLA Mcity Co-Simulation"
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
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
version = client.get_server_version()
print(f'CARLA server version: {version}')
" 2>/dev/null && CARLA_AVAILABLE=true || true

if [ "$CARLA_AVAILABLE" = true ]; then
    echo "  CARLA is available! Running CARLA co-simulation..."
    echo "  Config: /app/examples/simulation_Mcity_carla_config.yaml"
    echo "------------------------------------------"

    # Run CARLA co-simulation
    python3 /app/examples/scripts/carla_cosim_main.py \
        --terasim_config /app/examples/simulation_Mcity_carla_config.yaml \
        --map_name=McityMap_Main || true
    CARLA_EXIT=$?

    echo "=========================================="
    echo " CARLA client exited (code: $CARLA_EXIT)"
else
    echo "  CARLA is NOT available. Running TeraSim-only simulation..."
    echo "  Config: /app/examples/simulation_Mcity_carla_config.yaml"
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
    'config_file': '/app/examples/simulation_Mcity_carla_config.yaml',
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
