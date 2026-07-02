#!/bin/bash
# =============================================================================
# 3者 cosim お台場(direct 版): TeraSim 背景車両を psim の CARLA(:2013) に passive 注入する。
# docker-compose.cosim-odaiba-3cosim-direct.yml の command から呼ばれる(コンテナ内 entrypoint)。
# =============================================================================
# redis 版 run_3cosim_odaiba.sh との違い(= 第1段階の中継撤去):
#   - redis flush なし、TeraSim Service(FastAPI)起動なし
#   - TeraSim は terasim_service.run_direct(gRPC サーバ内蔵 plugin)として起動
#   - client は --direct_addr で gRPC 直結(1 step = 1 RPC、ポーリングなし)
# 前提:
#   - 第1段階 start_3cosim_odaiba_psim.sh で CARLA :2013(odaibatest5) + psim + bridge 稼働済み
# =============================================================================
set -u
cd /app

CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2013}"
GRPC_PORT="${GRPC_PORT:-8200}"
SCENARIO="${SCENARIO:-/app/examples/scenarios/cosim_odaiba_osmlike.yaml}"

echo "=========================================="
echo " TeraSim 3-cosim (passive, direct link) お台場"
echo "  CARLA :${CARLA_PORT}  /  gRPC :${GRPC_PORT}"
echo "  scenario: ${SCENARIO}"
echo "=========================================="

# ── Step 1: CARLA の ego 以外の車両を掃除(冪等化、redis 版と同一) ──
#   前回 run の SUMO 背景車両が CARLA(=psim 所有、コンテナと別ライフサイクル)に残ると
#   次回注入が "collision at spawn position" で詰まる。ego(ego_vehicle/hero)は保護。
echo "[1/3] CARLA :${CARLA_PORT} の ego 以外を掃除(前回残骸の除去、冪等化)"
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

# ── Step 2: TeraSim direct runner(gRPC サーバ内蔵)を起動 ──
echo "[2/3] start TeraSim direct runner (gRPC :${GRPC_PORT})"
python -m terasim_service.run_direct --config "${SCENARIO}" --grpc_port "${GRPC_PORT}" \
  > /tmp/terasim_runner.log 2>&1 &
RUNNER_PID=$!

# ── Step 3: passive client(gRPC 直結)──
#   DirectLink が runner の起動〜wait_for_tick を最大 600s 待つので、ここでの待機は不要
echo "[3/3] run passive client (carla_cosim_3cosim.py --direct_addr 127.0.0.1:${GRPC_PORT})"
echo "------------------------------------------"
python examples/scripts/carla_cosim_3cosim.py \
  --terasim_config "${SCENARIO}" \
  --carla_port "${CARLA_PORT}" \
  --direct_addr "127.0.0.1:${GRPC_PORT}"
CLIENT_EXIT=$?

echo "------------------------------------------"
echo " passive client exited (code: ${CLIENT_EXIT})"
kill "${RUNNER_PID}" 2>/dev/null || true
echo " container exiting."
