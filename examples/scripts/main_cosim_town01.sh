#!/bin/bash
set -e
umask 000

# CARLA host (override with the CARLA_HOST env var if the server is not on localhost).
CARLA_HOST="${CARLA_HOST:-localhost}"

# Maximum simulation time in seconds (warmup + run_time + buffer)
MAX_SIM_TIME=${MAX_SIM_TIME:-300}

echo "=========================================="
echo " TeraSim-CARLA Town01 Co-Simulation"
echo "  CARLA host: ${CARLA_HOST}"
echo "  Max time:   ${MAX_SIM_TIME}s"
echo "=========================================="

# ─── Step 0: Verify Town01 map files ─────────────────────────────────────────
echo "[0/5] Verifying Town01 map files..."
if [ ! -f /app/examples/maps/town01/Town01.net.xml ]; then
    echo "FATAL: /app/examples/maps/town01/Town01.net.xml not found."
    echo "  Generate it via netconvert from Town01.xodr."
    exit 1
fi
echo "  Map files OK."

# Update AV route and traffic_scale dynamically based on the network
echo "  Updating simulation config with network routes..."
python3 -c "
import xml.etree.ElementTree as ET
import yaml
import random
import math

random.seed(42)

# Parse network to get edge IDs and build connectivity graph
tree = ET.parse('/app/examples/maps/town01/Town01.net.xml')
root = tree.getroot()
edges = [e.get('id') for e in root.findall('.//edge') if not e.get('id', '').startswith(':')]

# Build connectivity graph from connections
connections = {}
for conn in root.findall('.//connection'):
    from_e = conn.get('from', '')
    to_e = conn.get('to', '')
    if from_e.startswith(':') or to_e.startswith(':'):
        continue
    if from_e not in connections:
        connections[from_e] = set()
    connections[from_e].add(to_e)

# Find a valid connected route by walking the graph
def find_route(start, max_len=15):
    visited = set()
    route = [start]
    current = start
    while len(route) < max_len:
        visited.add(current)
        neighbors = connections.get(current, set()) - visited
        if not neighbors:
            break
        current = sorted(neighbors)[0]
        route.append(current)
    return route

best_route = []
for start_edge in sorted(connections.keys()):
    route = find_route(start_edge)
    if len(route) > len(best_route):
        best_route = route

route_edges = best_route if len(best_route) >= 3 else edges[:5]

with open('/app/examples/scenarios/cosim_town01.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['environment']['parameters']['AV_cfg']['route'] = route_edges
print(f'  Updated AV route with {len(route_edges)} edges')

# Town01 uses righthand traffic (US-style street grid)
config['environment']['parameters']['drive_rule'] = 'righthand'

# ─── Calculate traffic_scale from num_cars ────────────────────────────────
num_cars = config.get('parameters', {}).get('num_cars', 30)

# Town01.trips.xml uses <trip> tags (randomTrips.py output), not <flow>.
# Estimate flow rate from the trip count.
trips_file = '/app/examples/maps/town01/Town01.trips.xml'
try:
    trips_tree = ET.parse(trips_file)
    trips_root = trips_tree.getroot()
    trip_count = len(trips_root.findall('trip'))
    # trip_count is over simulation_time (default 3600s), convert to veh/hour
    total_flow_rate = int(trip_count * 3600.0 / 3600.0)
except Exception:
    total_flow_rate = 2400  # fallback (Town01 randomTrips with period=1.5)

# Estimate network size from location element
loc = root.find('.//location')
if loc is not None:
    bounds = loc.get('convBoundary', '0,0,400,330').split(',')
    net_width = float(bounds[2]) - float(bounds[0])
    net_height = float(bounds[3]) - float(bounds[1])
else:
    net_width, net_height = 400, 330

net_diag = math.sqrt(net_width**2 + net_height**2)
avg_trip_length = net_diag * 0.4
avg_speed = 10.0
avg_trip_duration = avg_trip_length / avg_speed

base_equilibrium = total_flow_rate * avg_trip_duration / 3600.0

if base_equilibrium > 0:
    traffic_scale = max(1.0, round(num_cars / base_equilibrium, 1))
else:
    traffic_scale = 3.0

print(f'  num_cars={num_cars}, total_flow={total_flow_rate} veh/hr, '
      f'est_equilibrium={base_equilibrium:.0f}, traffic_scale={traffic_scale}')

if 'simulator' not in config:
    config['simulator'] = {}
if 'parameters' not in config['simulator']:
    config['simulator']['parameters'] = {}
config['simulator']['parameters']['traffic_scale'] = traffic_scale

sumo_cfg_path = '/app/examples/maps/town01/Town01.sumocfg'
try:
    sumo_tree = ET.parse(sumo_cfg_path)
    sumo_root = sumo_tree.getroot()
    for scale_elem in sumo_root.iter('scale'):
        scale_elem.set('value', '1.0')
    sumo_tree.write(sumo_cfg_path, xml_declaration=True, encoding='UTF-8')
    print('  Updated SUMO config: scale=1.0 (traffic_scale handles scaling)')
except Exception as e:
    print(f'  Warning: could not update SUMO config: {e}')

with open('/app/examples/scenarios/cosim_town01.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

print('  Config updated successfully.')
" 2>&1 || echo "  Warning: Config update failed, using defaults."

# ─── Step 1: Start Redis ─────────────────────────────────────────────────────
echo "[1/5] Starting Redis server..."
redis-server &

# ─── Step 2: Start TeraSim Server ────────────────────────────────────────────
echo "[2/5] Starting TeraSim server..."
python -m terasim_service &
TERASIM_PID=$!

# ─── Step 3: Wait for server ─────────────────────────────────────────────────
echo "[3/5] Waiting 30s for TeraSim server to be ready..."
sleep 30

# ─── Step 4: Check CARLA and start co-simulation ─────────────────────────────
echo "[4/5] Starting simulation..."
echo "------------------------------------------"

# Check if CARLA is reachable via CARLA_HOST
CARLA_AVAILABLE=false
python3 -c "
import carla
client = carla.Client('${CARLA_HOST}', 2000)
client.set_timeout(5.0)
version = client.get_server_version()
print(f'CARLA server version: {version}')
" 2>/dev/null && CARLA_AVAILABLE=true || true

if [ "$CARLA_AVAILABLE" = true ]; then
    echo "  CARLA is available! Running CARLA co-simulation..."

    # Town01 is a built-in CARLA map — just load it by name, no OpenDRIVE generation needed
    echo "  Loading Town01 in CARLA..."
    python3 -c "
import carla
client = carla.Client('${CARLA_HOST}', 2000)
client.set_timeout(30.0)
world = client.load_world('Town01')
print(f'Loaded: {world.get_map().name}')
" 2>&1 || echo "  Warning: Failed to load Town01 in CARLA."

    echo "  Config: /app/examples/scenarios/cosim_town01.yaml"
    echo "------------------------------------------"

    # Run CARLA co-simulation
    python3 /app/examples/scripts/carla_cosim_main.py \
        --carla_host ${CARLA_HOST} \
        --terasim_config /app/examples/scenarios/cosim_town01.yaml || true
    CARLA_EXIT=$?

    echo "=========================================="
    echo " CARLA client exited (code: $CARLA_EXIT)"
else
    echo "  CARLA is NOT available at ${CARLA_HOST}:2000. Running TeraSim-only simulation..."
    echo "  Config: /app/examples/scenarios/cosim_town01.yaml"
    echo "------------------------------------------"

    python3 -c "
import requests
import time

TERASIM_HOST = 'localhost'
TERASIM_PORT = 8000
BASE_URL = f'http://{TERASIM_HOST}:{TERASIM_PORT}'

print('Starting TeraSim-only simulation...')

init_data = {
    'config_file': '/app/examples/scenarios/cosim_town01.yaml',
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

time.sleep(5)

MAX_STEPS = 6000
for step in range(MAX_STEPS):
    try:
        status_resp = requests.get(f'{BASE_URL}/simulation_status/{sim_id}', timeout=10)
        status = status_resp.json()

        if status.get('status') == 'finished':
            print(f'Simulation finished at step {step}')
            break

        if status.get('status') in ['wait_for_tick', 'ticked', 'started', 'running']:
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
