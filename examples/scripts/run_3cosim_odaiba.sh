#!/bin/bash
# =============================================================================
# Odaiba 3-way co-simulation, TeraSim stage: inject SUMO background traffic
# into a CARLA server that is owned and ticked by an external client (e.g. an
# Autoware bridge running in synchronous mode). Called as the container
# entrypoint by docker-compose.cosim-odaiba-3cosim.yml.
# =============================================================================
# Prerequisites:
#   - A CARLA server (default :2013) with the Odaiba OpenDRIVE map loaded and
#     an external tick master attached (this script never calls world.tick()).
#   - A host redis on localhost:6379 (terasim_service connects to it).
# Design notes:
#   - Uses the passive client (carla_cosim_3cosim.py), which follows the
#     external ticks and never reloads the world. The active launcher
#     (main_cosim_odaiba.sh) reloads the CARLA world -- do not use it here.
#   - Scenario: cosim_odaiba_osmlike.yaml (UTM zone 54 SUMO net,
#     coordinate-aligned with the Odaiba OpenDRIVE map).
#   - The AV route keep-alive in plugins/cosim.py prevents SUMO from retiring
#     the externally-driven AV on dense maps ("AV_left").
#   - Startup is idempotent: leftovers from a previous force-removed run
#     (stale redis keys, orphaned CARLA vehicles) are cleaned up first.
# =============================================================================
set -u
cd /app

CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2013}"
TERASIM_PORT="${TERASIM_PORT:-8100}"
SCENARIO="${SCENARIO:-/app/examples/scenarios/cosim_odaiba_osmlike.yaml}"

echo "=========================================="
echo " TeraSim 3-way cosim (passive), Odaiba"
echo "  CARLA :${CARLA_PORT}  /  TeraSim service :${TERASIM_PORT}"
echo "  scenario: ${SCENARIO}"
echo "=========================================="

# -- Step 1: flush stale redis keys --
#    simulation:* keys left over from a previous run block the next
#    simulation start.
echo "[1/5] redis flush (dbsize before: $(redis-cli dbsize 2>/dev/null))"
redis-cli flushdb >/dev/null 2>&1 || true
echo "      dbsize after:  $(redis-cli dbsize 2>/dev/null)"

# -- Step 2: destroy CARLA vehicles except the ego (idempotency) --
#    If a previous run was force-removed, its SUMO background vehicles stay
#    behind in CARLA (owned by the external client, a different lifecycle)
#    and the next injection gets stuck with "collision at spawn position".
#    Note: in synchronous mode the first get_actors() can return 0 -- retry.
#    The ego (role_name ego_vehicle/hero) is protected.
echo "[2/5] destroy non-ego vehicles in CARLA :${CARLA_PORT} (cleanup of leftovers)"
python - <<PY
import carla, time
c = carla.Client("${CARLA_HOST}", ${CARLA_PORT}); c.set_timeout(60.0)
w = c.get_world()
vs = []
for i in range(12):
    vs = list(w.get_actors().filter("*vehicle*"))
    if len(vs) > 0:
        break
    time.sleep(0.5)
keep = ("ego_vehicle", "hero")
victims = [v for v in vs if v.attributes.get("role_name") not in keep]
print("      CARLA total=%d ego=%d destroy=%d" % (len(vs), len(vs) - len(victims), len(victims)))
if victims:
    c.apply_batch([carla.command.DestroyActor(v.id) for v in victims])
    time.sleep(2)
    print("      after cleanup=%d" % len(list(w.get_actors().filter("*vehicle*"))))
PY

# -- Step 3: start the TeraSim service --
#    __main__.py hardcodes port 8000, so call uvicorn directly to use
#    ${TERASIM_PORT}.
echo "[3/5] start TeraSim service on :${TERASIM_PORT}"
python -c "import uvicorn; from terasim_service.api import create_app; uvicorn.run(create_app(), host='0.0.0.0', port=${TERASIM_PORT})" \
  > /tmp/terasim_service.log 2>&1 &
SERVICE_PID=$!

# -- Step 4: wait for the service to listen (max 40 s) --
echo "[4/5] wait for service :${TERASIM_PORT} ..."
for i in $(seq 1 40); do
  if python -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',${TERASIM_PORT}));s.close()" 2>/dev/null; then
    echo "      service ready (${i}s)"; break
  fi
  sleep 1
done

# -- Step 5: run the passive client (injects background vehicles, never ticks) --
echo "[5/5] run passive client (carla_cosim_3cosim.py)"
echo "------------------------------------------"
python examples/scripts/carla_cosim_3cosim.py \
  --terasim_config "${SCENARIO}" \
  --carla_port "${CARLA_PORT}" \
  --terasim_port "${TERASIM_PORT}"
CLIENT_EXIT=$?

echo "------------------------------------------"
echo " passive client exited (code: ${CLIENT_EXIT})"
kill "${SERVICE_PID}" 2>/dev/null || true
echo " container exiting."
