# Odaiba Packaged CARLA Co-Simulation

この手順は、手元に次の 2 つしかない状態から始める想定です。

- CARLA custom map の配布 package
- SUMO の `network.net.xml`

今回の Odaiba LL2 では、入力ファイルは次の配置を前提にします。

```text
examples/maps/odaiba_ll2/
  export_odaibatest5_294096eb1-dirty.tar.gz
  network.net.xml
```

この packaged-map 方式では、`.xodr` は co-sim 実行に必須ではありません。
CARLA 側は package を import して `load_world()` で地図を開きます。

詳しい noVNC のみの説明は [private_carla_novnc.md](private_carla_novnc.md) を参照してください。

## 1. 事前に分かっていること

- CARLA 側の map 名は `odaibatest5`
- SUMO 側は `examples/maps/odaiba_ll2/network.net.xml` を使う
- Odaiba の OpenDRIVE 変換では次の offset を使っている

```yaml
mgrs_grid: 54SUE
offset:
  x: 92008.5
  y: 45335.1
  z: 0
```

このため co-sim wrapper は既定で次の `SUMO -> CARLA` offset を使います。

- `SUMO_TO_CARLA_OFFSET_X=-92008.5`
- `SUMO_TO_CARLA_OFFSET_Y=45335.1`
- `SUMO_TO_CARLA_OFFSET_Z=0.0`

package を別 offset で作っている場合だけ、実行時にこの env を上書きしてください。

## 2. 使う helper

今回の workflow では次の helper を使います。

- `scripts/prepare_odaiba_ll2_sumo_artifacts.sh`
- `scripts/run_cosim_odaiba_ll2_packaged_generated.sh`
- `scripts/follow_carla_actor_novnc.sh`

これらは次の生成済み scenario を前提にします。

- `examples/scenarios/cosim_odaiba_ll2_generated.yaml`

## 3. Docker image を build する

co-sim image と CARLA noVNC image の両方を build します。

```bash
cd /home/h-kawai/TeraSim

docker compose -f docker-compose.cosim-odaiba-ll2.yml build
docker compose -f docker-compose.carla-novnc.yml build
```

初回の CARLA build は `python3.10` を image 内で build するので時間がかかります。

## 4. CARLA noVNC を起動する

```bash
cd /home/h-kawai/TeraSim

docker rm -f carla-novnc-test 2>/dev/null || true
docker compose -f docker-compose.carla-novnc.yml up -d
```

確認:

```bash
docker ps --filter name=carla-novnc-test
docker logs --tail=30 carla-novnc-test
docker exec carla-novnc-test nvidia-smi
```

ローカル PC から noVNC を見る場合:

```bash
ssh -N -L 6092:localhost:6092 <user>@<server>
```

ブラウザ:

```text
http://localhost:6092/vnc.html
```

パスワード:

```text
headless
```

## 5. CARLA package を import する

package を CARLA container の `Import` ディレクトリへ入れて import します。

```bash
cd /home/h-kawai/TeraSim

PKG=examples/maps/odaiba_ll2/export_odaibatest5_294096eb1-dirty.tar.gz
C=carla-novnc-test

docker exec -u root $C bash -lc 'mkdir -p /workspace/Import && chown -R carla:carla /workspace/Import'
docker cp "$PKG" $C:/workspace/Import/
docker exec -it $C bash -lc 'cd /workspace && ./ImportAssets.sh'
docker compose -f docker-compose.carla-novnc.yml restart carla_novnc
```

map 一覧確認:

```bash
C=carla-novnc-test
docker exec -it $C bash -lc '
python3.10 - <<PY
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(60.0)
for name in client.get_available_maps():
    print(name)
PY
'
```

`odaibatest5` が見えたら OK です。

必要なら明示的に load して確認します。

```bash
C=carla-novnc-test
docker exec -it $C bash -lc '
python3.10 - <<PY
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(60.0)
world = client.load_world("odaibatest5")
print("loaded:", world.get_map().name)
PY
'
```

## 6. `network.net.xml` から SUMO 側の不足ファイルを生成する

手元に `network.net.xml` しかなくても、この helper が次を自動生成します。

- `examples/maps/odaiba_ll2/metadata.json`
- `examples/maps/odaiba_ll2/trips.trips.xml`
- `examples/maps/odaiba_ll2/vehicles.rou.xml`
- `examples/maps/odaiba_ll2/simulation.sumocfg`

さらに `metadata.json` の `av_route_edge_ids` をもとに
`examples/scenarios/cosim_odaiba_ll2_generated.yaml` の `AV_cfg.route`
も更新します。

```bash
cd /home/h-kawai/TeraSim
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

交通量を増やしたい場合:

```bash
cd /home/h-kawai/TeraSim
PERIOD=1.0 ./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

AV route を毎回変えたい場合:

```bash
cd /home/h-kawai/TeraSim
PERIOD=0.5 AV_ROUTE_SEED=$(date +%s) FORCE_NEW_AV_ROUTE=1 \
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

補足:

- `SEED` は背景交通の randomTrips 用です
- `AV_ROUTE_SEED` は AV route だけを変える seed です
- `FORCE_NEW_AV_ROUTE=1` を付けると、既存 `metadata.json` に保存済みの route を無視して新しく作ります

補足:

- `metadata.json` が無ければ空ファイルから自動生成されます
- `metadata.json` に AV route が無ければ、`network.net.xml` から fallback route を自動生成します

## 7. TeraSim-CARLA co-sim を起動する

通常はこれだけで実行できます。

```bash
cd /home/h-kawai/TeraSim
CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

この wrapper は次を自動でやります。

- `carla-novnc-test` の container IP を自動検出
- container 間接続なら `CARLA_PORT=2000` を使用
- TeraSim port を空き port から自動選択
- `cosim_odaiba_ll2_generated.yaml` を使用
- Odaiba 用の `SUMO_TO_CARLA_OFFSET_*` を注入

host 側の公開 port `2010` を明示的に使いたい場合だけ、こちらです。

```bash
cd /home/h-kawai/TeraSim
CARLA_HOST=localhost CARLA_PORT=2010 CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

重要:

- 既定は `CARLA co-sim async mode: 0` です
- この workflow では sync mode を使ってください
- `CARLA_COSIM_ASYNC_MODE=1` を付けると、現状の `auto_run=false` 経路では co-sim が停止することがあります

## 8. noVNC で AV を追尾する

co-sim が動いたら、spectator を AV に追従させると確認しやすいです。

```bash
cd /home/h-kawai/TeraSim
./scripts/follow_carla_actor_novnc.sh
```

真上視点:

```bash
cd /home/h-kawai/TeraSim
CAMERA_MODE=topdown ./scripts/follow_carla_actor_novnc.sh
```

少し近い chase view:

```bash
cd /home/h-kawai/TeraSim
FOLLOW_DISTANCE=8 FOLLOW_HEIGHT=3.5 ./scripts/follow_carla_actor_novnc.sh
```

## 9. 終了理由を確認する

各 run の出力は次に入ります。

```text
outputs/odaiba_ll2_generated_test/raw_data/0_0/<simulation_id>/
```

まず確認したいファイル:

- `monitor.json`
- `run.log`
- `terasim_cosim_plugin.log`
- `collision.xml`
- `conflict_info.jsonl`

終了理由の最短確認:

```bash
SIM_ID=<simulation_id>
BASE=/home/h-kawai/TeraSim/outputs/odaiba_ll2_generated_test/raw_data/0_0/$SIM_ID

python3 -m json.tool "$BASE/monitor.json" | grep -E '"finish_reason"|"collider"|"victim"|"final_time"'
tail -n 20 "$BASE/run.log"
```

見方:

- `finish_reason: timeout` なら規定時間終了
- `finish_reason: collision` なら衝突終了
- `finish_reason: AV_left` なら AV 離脱終了

実行中か終了直後なら API でも見られます。

```bash
curl -s http://127.0.0.1:<TERASIM_PORT>/simulation_result/<simulation_id> | python3 -m json.tool
```

## 10. よくある詰まりどころ

### `Map '...' not found`

package import が終わっていないか、CARLA restart 前です。

```bash
docker exec -it carla-novnc-test bash -lc '
python3.10 - <<PY
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(60.0)
print(client.get_available_maps())
PY
'
```

### `CARLA map probe not ready yet`

CARLA の world 切り替え中です。少し待って再実行してください。
`run_cosim_odaiba_ll2_packaged_generated.sh` は既定で待機込みです。

### `Redis ... bind: Address already in use`

今回の packaged runner では既存 Redis を再利用するので、多くの場合はそのままで問題ありません。

### noVNC で AV しか見えない

総車両数が少ないとは限りません。局所的に周囲が空いている場合があります。
まず AV 追尾をかけてから、必要なら `PERIOD` を小さくして交通量を増やしてください。

### AV が map 外に飛ぶ

`SUMO_TO_CARLA_OFFSET_*` が package 作成時の offset と合っていない可能性があります。
今回の Odaiba LL2 では既定値は次です。

```bash
SUMO_TO_CARLA_OFFSET_X=-92008.5
SUMO_TO_CARLA_OFFSET_Y=45335.1
SUMO_TO_CARLA_OFFSET_Z=0.0
```

## 11. まとめ

今回の Odaiba packaged co-sim の最短ルートは次です。

1. `docker compose -f docker-compose.cosim-odaiba-ll2.yml build`
2. `docker compose -f docker-compose.carla-novnc.yml build`
3. `docker compose -f docker-compose.carla-novnc.yml up -d`
4. package を `ImportAssets.sh` で import
5. `./scripts/prepare_odaiba_ll2_sumo_artifacts.sh`
6. `CARLA_PACKAGE_MAP_NAME=odaibatest5 ./scripts/run_cosim_odaiba_ll2_packaged_generated.sh`
7. `./scripts/follow_carla_actor_novnc.sh`
