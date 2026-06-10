#!/bin/bash
# =============================================================================
# 3者 cosim お台場: TeraSim 背景車両を psim の CARLA(:2013) に passive 注入する。
# docker-compose.cosim-odaiba-3cosim.yml の command から呼ばれる(コンテナ内 entrypoint)。
# =============================================================================
# 前提:
#   - 第1段階 start_3cosim_odaiba_psim.sh で CARLA :2013(odaibatest5) + psim + bridge 稼働済み
#   - redis は host のシステム redis(localhost:6379) を共有(terasim_service が固定接続)
# 設計(お台場固有の落とし穴に対応):
#   - passive client(carla_cosim_3cosim.py)を使い world.tick() しない → psim の CARLA を壊さない
#     (active 版 main_cosim_odaiba.sh は generate_opendrive_world で世界を再ロードするので3者厳禁)
#   - scenario = cosim_odaiba_osmlike.yaml (cosim_odaiba_ll2.yaml は net.xml 不在で使えない)
#   - plugins/cosim.py の (D) route 動的継ぎ足し(commit 6d48579)で AV_left を防ぐ(bind mount)
#   - 冪等化: 前回 run が CARLA に残した背景車両を起動時に掃除(ego は保護)してから注入
# =============================================================================
set -u
cd /app

CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2013}"
TERASIM_PORT="${TERASIM_PORT:-8100}"
SCENARIO="${SCENARIO:-/app/examples/scenarios/cosim_odaiba_osmlike.yaml}"

echo "=========================================="
echo " TeraSim 3-cosim (passive) お台場"
echo "  CARLA :${CARLA_PORT}  /  TeraSim service :${TERASIM_PORT}"
echo "  scenario: ${SCENARIO}"
echo "=========================================="

# ── Step 1: redis 残骸を掃除(過去 sim の simulation:* key が初回起動を阻害) ──
echo "[1/5] redis flush (dbsize before: $(redis-cli dbsize 2>/dev/null))"
redis-cli flushdb >/dev/null 2>&1 || true
echo "      dbsize after:  $(redis-cli dbsize 2>/dev/null)"

# ── Step 2: CARLA の ego 以外の車両を掃除(冪等化) ──
#   前回 run をコンテナ強制削除すると SUMO 背景車両が CARLA(=psim 所有)に取り残され、
#   次回 SUMO 注入が "collision at spawn position" で詰まる。起動時に掃除して防ぐ。
#   sync mode の初回 get_actors() は 0 を返すのでリトライ必須。ego(ego_vehicle/hero)は保護。
echo "[2/5] CARLA :${CARLA_PORT} の ego 以外を掃除(前回残骸の除去、冪等化)"
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

# ── Step 3: TeraSim service を :${TERASIM_PORT} で起動 ──
#   __main__.py は port=8000 ハードコード(他コンテナ占有)なので uvicorn を直接叩いて 8100 に
echo "[3/5] start TeraSim service on :${TERASIM_PORT}"
python -c "import uvicorn; from terasim_service.api import create_app; uvicorn.run(create_app(), host='0.0.0.0', port=${TERASIM_PORT})" \
  > /tmp/terasim_service.log 2>&1 &
SERVICE_PID=$!

# ── Step 4: service の listen を待つ(最大 40s) ──
echo "[4/5] wait for service :${TERASIM_PORT} ..."
for i in $(seq 1 40); do
  if python -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',${TERASIM_PORT}));s.close()" 2>/dev/null; then
    echo "      service ready (${i}s)"; break
  fi
  sleep 1
done

# ── Step 5: passive client で背景車両を CARLA に注入(world.tick しない) ──
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
