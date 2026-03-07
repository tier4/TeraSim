#!/bin/bash
set -e
umask 000

# Maximum simulation time in seconds (warmup + run_time + buffer)
MAX_SIM_TIME=${MAX_SIM_TIME:-300}

echo "=========================================="
echo " TeraSim-CARLA Odaiba Co-Simulation"
echo "  Max time: ${MAX_SIM_TIME}s"
echo "=========================================="

# ─── Step 0: Setup Odaiba Map ────────────────────────────────────────────────
echo "[0/5] Setting up Odaiba map..."
if [ ! -f /app/examples/maps/odaiba/odaiba.net.xml ]; then
    echo "  Map files not found, running setup..."
    python3 /app/examples/scripts/setup_odaiba_map.py

    if [ ! -f /app/examples/maps/odaiba/odaiba.net.xml ]; then
        echo "FATAL: Map setup failed. Exiting."
        exit 1
    fi
else
    echo "  Map files already exist, skipping setup."
fi

# Update config with actual routes from the network and calculate traffic_scale
echo "  Updating simulation config with network routes..."
python3 -c "
import xml.etree.ElementTree as ET
import yaml
import random
import math

random.seed(42)

# Parse network to get edge IDs and build connectivity graph
tree = ET.parse('/app/examples/maps/odaiba/odaiba.net.xml')
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

# Update the YAML config
with open('/app/examples/simulation_odaiba_config.yaml', 'r') as f:
    config = yaml.safe_load(f)

config['environment']['parameters']['AV_cfg']['route'] = route_edges
print(f'  Updated AV route with {len(route_edges)} edges')

# Japan drives on the left but SUMO OSM import keeps it righthand by default
config['environment']['parameters']['drive_rule'] = 'righthand'

# ─── Calculate traffic_scale from num_cars ────────────────────────────────
num_cars = config.get('parameters', {}).get('num_cars', 50)

# Parse trips file to get total flow rate (veh/hour)
trips_file = '/app/examples/maps/odaiba/odaiba.trips.xml'
try:
    trips_tree = ET.parse(trips_file)
    trips_root = trips_tree.getroot()
    total_flow_rate = sum(
        int(flow.get('vehsPerHour', '0'))
        for flow in trips_root.findall('flow')
    )
except Exception:
    total_flow_rate = 1000  # fallback estimate

# Estimate network size from location element
loc = root.find('.//location')
if loc is not None:
    bounds = loc.get('convBoundary', '0,0,1000,1000').split(',')
    net_width = float(bounds[2]) - float(bounds[0])
    net_height = float(bounds[3]) - float(bounds[1])
else:
    net_width, net_height = 1900, 2600

# Estimate average trip duration:
# avg trip length ~ half of network diagonal, avg speed ~ 10 m/s (accounting for stops)
net_diag = math.sqrt(net_width**2 + net_height**2)
avg_trip_length = net_diag * 0.4
avg_speed = 10.0  # m/s, lower than max to account for intersections/stops
avg_trip_duration = avg_trip_length / avg_speed

# Equilibrium vehicle count at scale=1.0:
#   equilibrium = total_flow_rate * avg_trip_duration / 3600
base_equilibrium = total_flow_rate * avg_trip_duration / 3600.0

# Calculate traffic_scale to achieve num_cars
if base_equilibrium > 0:
    traffic_scale = max(1.0, round(num_cars / base_equilibrium, 1))
else:
    traffic_scale = 3.0

print(f'  num_cars={num_cars}, total_flow={total_flow_rate} veh/hr, '
      f'est_equilibrium={base_equilibrium:.0f}, traffic_scale={traffic_scale}')

# Set traffic_scale in simulator parameters (source build supports this!)
if 'simulator' not in config:
    config['simulator'] = {}
if 'parameters' not in config['simulator']:
    config['simulator']['parameters'] = {}
config['simulator']['parameters']['traffic_scale'] = traffic_scale

# Also update SUMO config to set scale=1.0 (traffic_scale handles scaling via --scale)
sumo_cfg_path = '/app/examples/maps/odaiba/odaiba.sumocfg'
try:
    sumo_tree = ET.parse(sumo_cfg_path)
    sumo_root = sumo_tree.getroot()
    # Update scale in input section
    for scale_elem in sumo_root.iter('scale'):
        scale_elem.set('value', '1.0')
    sumo_tree.write(sumo_cfg_path, xml_declaration=True, encoding='UTF-8')
    print('  Updated SUMO config: scale=1.0 (traffic_scale handles scaling)')
except Exception as e:
    print(f'  Warning: could not update SUMO config: {e}')

with open('/app/examples/simulation_odaiba_config.yaml', 'w') as f:
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

    # Check if OpenDRIVE file exists for CARLA
    if [ -f /app/examples/maps/odaiba/odaiba_carla.xodr ]; then
        echo "  Loading Odaiba OpenDRIVE map into CARLA..."

        # Load OpenDRIVE map into CARLA using generate_opendrive_world
        python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)

# Read the OpenDRIVE file
with open('/app/examples/maps/odaiba/odaiba_carla.xodr', 'r') as f:
    xodr_data = f.read()

# Load the OpenDRIVE map
params = carla.OpendriveGenerationParameters(
    vertex_distance=2.0,
    max_road_length=500.0,
    wall_height=0.0,
    additional_width=0.6,
    smooth_junctions=True,
    enable_mesh_visibility=True,
)
world = client.generate_opendrive_world(xodr_data, params)
print('Odaiba OpenDRIVE map loaded into CARLA!')
" 2>&1 || echo "  Warning: Failed to load OpenDRIVE into CARLA."
    fi

    echo "  Config: /app/examples/simulation_odaiba_config.yaml"
    echo "------------------------------------------"

    # Run CARLA co-simulation
    python3 /app/examples/scripts/carla_cosim_main.py \
        --terasim_config /app/examples/simulation_odaiba_config.yaml || true
    CARLA_EXIT=$?

    echo "=========================================="
    echo " CARLA client exited (code: $CARLA_EXIT)"
else
    echo "  CARLA is NOT available. Running TeraSim-only simulation..."
    echo "  Config: /app/examples/simulation_odaiba_config.yaml"
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
    'config_file': '/app/examples/simulation_odaiba_config.yaml',
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
