#!/bin/bash
# =============================================================================
# 3者 cosim お台場(単一プロセス版): TeraSim 背景車両を psim の CARLA(:2013) に passive 注入する。
# docker-compose.cosim-odaiba-3cosim-inprocess.yml の command から呼ばれる(コンテナ内 entrypoint)。
# =============================================================================
# direct(gRPC)版 run_3cosim_odaiba_direct.sh との違い(= 第2段階の同一プロセス化):
#   - TeraSim runner + CARLA client の 2 プロセス+gRPC が terasim_service.run_cosim の
#     1 プロセスに統合(状態・コマンドは Python オブジェクトのまま受け渡し)
# 前提:
#   - 第1段階 start_3cosim_odaiba_psim.sh で CARLA :2013(odaibatest5) + psim + bridge 稼働済み
# =============================================================================
set -u
cd /app

CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2013}"
SCENARIO="${SCENARIO:-/app/examples/scenarios/cosim_odaiba_osmlike.yaml}"

echo "=========================================="
echo " TeraSim 3-cosim (passive, in-process) お台場"
echo "  CARLA :${CARLA_PORT}"
echo "  scenario: ${SCENARIO}"
echo "=========================================="

# ── Step 1: CARLA の ego 以外の車両を掃除(冪等化、redis 版と同一) ──
#   前回 run の SUMO 背景車両が CARLA(=psim 所有、コンテナと別ライフサイクル)に残ると
#   次回注入が "collision at spawn position" で詰まる。ego(ego_vehicle/hero)は保護。
echo "[1/2] CARLA :${CARLA_PORT} の ego 以外を掃除(前回残骸の除去、冪等化)"
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

# ── Step 2: 単一プロセス runner(TeraSim + CARLA クライアント)──
#   exec で bash を置き換え、docker stop の SIGTERM が直接 python に届くようにする
#   (runner は SIGTERM を受けて TeraSim 停止 + CARLA 背景車両の掃除まで行う)。
echo "[2/2] run terasim_service.run_cosim (single process)"
echo "------------------------------------------"
exec python -m terasim_service.run_cosim \
  --config "${SCENARIO}" \
  --carla_host "${CARLA_HOST}" \
  --carla_port "${CARLA_PORT}"
