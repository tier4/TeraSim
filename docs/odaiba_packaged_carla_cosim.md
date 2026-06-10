# Odaiba Packaged CARLA Co-Simulation

この手順は、手元に次の 2 つしかない状態から始める想定です。

- CARLA custom map の配布 package
- SUMO の `odaiba_osmlike_network3.net.xml`

今回の Odaiba LL2 では、入力ファイルは次の配置を前提にします。

```text
examples/maps/odaiba_ll2/
  export_odaibatest5_294096eb1-dirty.tar.gz
  odaiba_osmlike_network3.net.xml
```

SUMO net を別ファイル名で試す場合も同じディレクトリに置けば使えます。
別の SUMO net を試す場合は、後述の `SUMO_NET_FILE` で指定します。

この packaged-map 方式では、`.xodr` は co-sim 実行に必須ではありません。
CARLA 側は package を import して `load_world()` で地図を開きます。

詳しい noVNC のみの説明は [private_carla_novnc.md](private_carla_novnc.md) を参照してください。

## 1. 事前に分かっていること

- CARLA 側の map 名は `odaibatest5`
- SUMO 側の既定は `examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml`
- 別の SUMO net を使う場合は `SUMO_NET_FILE=examples/maps/odaiba_ll2/<file>.net.xml` で切り替える
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
- `scripts/show_carla_vehicle_camera_novnc.sh`

これらは次の生成済み scenario を前提にします。

- `examples/scenarios/cosim_odaiba_ll2_generated.yaml`

## 3. Docker image を build する

co-sim image と CARLA noVNC image の両方を build します。

```bash
cd /path/to/TeraSim

docker compose -f docker-compose.cosim-odaiba-ll2.yml build
docker compose -f docker-compose.carla-novnc.yml build
```

初回の CARLA build は `python3.10` を image 内で build するので時間がかかります。

## 4. CARLA noVNC を起動する

```bash
cd /path/to/TeraSim

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

後で co-sim を起動すると、SUMO 側の noVNC も既定で `6093` に出ます。
CARLA と SUMO を同時に見る場合は、最初から両方 forward しておくと便利です。

```bash
ssh -N -L 6092:localhost:6092 -L 6093:localhost:6093 <user>@<server>
```

## 5. CARLA package を import する

package を CARLA container の `Import` ディレクトリへ入れて import します。

```bash
cd /path/to/TeraSim

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

## 6. `odaiba_osmlike_network3.net.xml` から SUMO 側の不足ファイルを生成する

手元に `odaiba_osmlike_network3.net.xml` があれば、この helper が次を自動生成します。

- `examples/maps/odaiba_ll2/metadata.json`
- `examples/maps/odaiba_ll2/trips.trips.xml`
- `examples/maps/odaiba_ll2/vehicles.rou.xml`
- `examples/maps/odaiba_ll2/simulation.sumocfg`

さらに `metadata.json` の `av_route_edge_ids` をもとに
`examples/scenarios/cosim_odaiba_ll2_generated.yaml` の `AV_cfg.route`
も更新します。

```bash
cd /path/to/TeraSim
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

別の SUMO net を使いたい場合は `SUMO_NET_FILE` を指定します。この指定は
`simulation.sumocfg` と `examples/scenarios/cosim_odaiba_ll2_generated.yaml` の
`sumo_net_file_path` / `input.sumo_net_file` にも反映されます。

```bash
cd /path/to/TeraSim
SUMO_NET_FILE=examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml \
PERIOD=1.0 FORCE_NEW_AV_ROUTE=1 \
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

交通量を増やしたい場合:

```bash
cd /path/to/TeraSim
PERIOD=1.0 ./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

AV route を毎回変えたい場合:

```bash
cd /path/to/TeraSim
PERIOD=0.5 AV_ROUTE_SEED=$(date +%s) FORCE_NEW_AV_ROUTE=1 \
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

AV route file を明示して固定したい場合:

```bash
cd /path/to/TeraSim
SUMO_NET_FILE=examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml \
PERIOD=0.5 \
AV_ROUTE_FILE=examples/maps/odaiba_ll2/teleport-mirai-loop.rou.xml \
./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
```

route file に複数の `<route>` がある場合は `AV_ROUTE_ID=<route id>` も指定できます。
`AV_ROUTE_FILE` を指定しない場合は、従来どおり `metadata.json` の AV route、
または fallback route を使います。

co-sim 起動時にまとめて指定することもできます。

```bash
cd /path/to/TeraSim
PERIOD=0.5 \
AV_ROUTE_FILE=examples/maps/odaiba_ll2/teleport-mirai-loop.rou.xml \
CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

補足:

- `SEED` は背景交通の randomTrips 用です
- `AV_ROUTE_SEED` は AV route だけを変える seed です
- `FORCE_NEW_AV_ROUTE=1` を付けると、既存 `metadata.json` に保存済みの route を無視して新しく作ります
- `SUMO_NET_FILE` は使う SUMO net です。旧名の `NET_PATH` も互換用に使えますが、通常は `SUMO_NET_FILE` を使ってください

補足:

- `metadata.json` が無ければ空ファイルから自動生成されます
- `metadata.json` に AV route が無ければ、指定した SUMO net から fallback route を自動生成します

## 7. TeraSim-CARLA co-sim を起動する

通常はこれだけで実行できます。

```bash
cd /path/to/TeraSim
CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

この runner は既定で SUMO GUI も noVNC 上に起動します。
SUMO GUI は AV を追尾する view に自動設定されます。

```text
CARLA noVNC: http://localhost:6092/vnc.html
SUMO noVNC:  http://localhost:6093/vnc.html
Password:    headless
```

SUMO の寄り具合を変えたい場合は `SUMO_GUI_TRACK_ZOOM` を調整します。
値を大きくすると近く、小さくすると広く見えます。

```bash
cd /path/to/TeraSim
SUMO_GUI_TRACK_ZOOM=650 CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

追尾対象を変える場合は `SUMO_GUI_TRACK_VEHICLE` を指定します。

SUMO GUI が不要な場合だけ、次のように無効化します。

```bash
cd /path/to/TeraSim
ENABLE_SUMO_GUI=0 CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

port が衝突する場合は `SUMO_NOVNC_PORT` と `SUMO_VNC_PORT` を変更してください。

```bash
cd /path/to/TeraSim
SUMO_NOVNC_PORT=6094 SUMO_VNC_PORT=5914 CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

この wrapper は次を自動でやります。

- `carla-novnc-test` の container IP を自動検出
- container 間接続なら `CARLA_PORT=2000` を使用
- TeraSim port を空き port から自動選択
- `cosim_odaiba_ll2_generated.yaml` を使用
- Odaiba 用の `SUMO_TO_CARLA_OFFSET_*` を注入
- SUMO GUI/noVNC 用の runtime config を `/tmp` に生成
- SUMO GUI を `SUMO_GUI_TRACK_VEHICLE` に追従させる

host 側の公開 port `2010` を明示的に使いたい場合だけ、こちらです。

```bash
cd /path/to/TeraSim
CARLA_HOST=localhost CARLA_PORT=2010 CARLA_PACKAGE_MAP_NAME=odaibatest5 \
./scripts/run_cosim_odaiba_ll2_packaged_generated.sh
```

重要:

- 既定は `CARLA co-sim async mode: 0` です
- この workflow では sync mode を使ってください
- `CARLA_COSIM_ASYNC_MODE=1` を付けると、現状の `auto_run=false` 経路では co-sim が停止することがあります

## 8. co-sim を安全に止める

通常は、co-sim を起動した terminal で Ctrl-C を押すと Python client が
`CarlaCosim.close()` を実行します。この正常終了では次が行われます。

- CARLA の synchronous mode を解除
- `fixed_delta_seconds` を解除
- co-sim が作った CARLA actor を削除
- TeraSim service に stop command を送信

Ctrl-C で止まらない場合は、すぐに `docker rm -f` せず、別 terminal から
TeraSim service に stop command を送ります。

まず co-sim container と TeraSim port を取得します。

```bash
C=$(docker ps --filter ancestor=terasim-service:cosim --format '{{.Names}}' | head -1)
PORT=$(docker exec "$C" bash -lc 'echo ${TERASIM_PORT:-8000}')
echo "container=$C port=$PORT"
```

`simulation_id` は Redis から取得できます。

```bash
SIM_ID=$(docker exec "$C" python3 - <<'PY'
import redis

r = redis.Redis()
simulation_ids = []
for key in r.scan_iter("simulation:*:status"):
    parts = key.decode().split(":")
    if len(parts) >= 3:
        simulation_ids.append(parts[1])

print(simulation_ids[-1] if simulation_ids else "")
PY
)
echo "simulation_id=$SIM_ID"
```

stop command を送ります。

```bash
curl -s -X POST "http://127.0.0.1:${PORT}/simulation_control/${SIM_ID}" \
  -H "Content-Type: application/json" \
  -d '{"command":"stop"}'
```

この後、元の co-sim terminal が `Cleaning synchronization` や
`Simulation complete` まで進めば成功です。

それでも止まらない場合は、次善策として timeout 付きで container に SIGTERM を送ります。

```bash
docker stop -t 30 "$C"
```

最後の手段だけ `docker rm -f` を使います。

```bash
docker rm -f "$C"
```

強制停止後に CARLA 側の actor や sync 設定が残っているように見える場合は、
CARLA noVNC container を再起動します。

```bash
docker compose -f docker-compose.carla-novnc.yml restart -t 30 carla_novnc
```

推奨順は次です。

1. 元 terminal で Ctrl-C
2. 別 terminal から TeraSim stop API
3. `docker stop -t 30`
4. 最後の手段として `docker rm -f`

## 9. noVNC で AV camera を表示する

AV に CARLA camera sensor を attach して、自車目線の RGB camera window を
noVNC desktop 上に出す場合:

```bash
cd /path/to/TeraSim
./scripts/show_carla_vehicle_camera_novnc.sh
```

ボンネット視点や後方追従風の sensor view:

```bash
CAMERA_PRESET=hood ./scripts/show_carla_vehicle_camera_novnc.sh
CAMERA_PRESET=chase ATTACHMENT_TYPE=SpringArmGhost ./scripts/show_carla_vehicle_camera_novnc.sh
```

Ctrl-C や pygame window close で終了しない場合は、container 内の
`python3.10 -` を止めます。このコマンドは attached camera viewer 用の Python を止めるためのものです。

```bash
docker exec carla-novnc-test bash -lc "pkill -TERM -f 'python3.10 -' || true"
```

残っているか確認したい場合:

```bash
docker exec carla-novnc-test bash -lc "pgrep -af 'python3.10 -' || true"
```

## 10. 終了理由を確認する

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
BASE=/path/to/TeraSim/outputs/odaiba_ll2_generated_test/raw_data/0_0/$SIM_ID

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

## 11. よくある詰まりどころ

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
SUMO GUI の AV 追従表示を確認してから、必要なら `PERIOD` を小さくして交通量を増やしてください。

### AV が map 外に飛ぶ

`SUMO_TO_CARLA_OFFSET_*` が package 作成時の offset と合っていない可能性があります。
今回の Odaiba LL2 では既定値は次です。

```bash
SUMO_TO_CARLA_OFFSET_X=-92008.5
SUMO_TO_CARLA_OFFSET_Y=45335.1
SUMO_TO_CARLA_OFFSET_Z=0.0
```

## 12. まとめ

今回の Odaiba packaged co-sim の最短ルートは次です。

1. `docker compose -f docker-compose.cosim-odaiba-ll2.yml build`
2. `docker compose -f docker-compose.carla-novnc.yml build`
3. `docker compose -f docker-compose.carla-novnc.yml up -d`
4. package を `ImportAssets.sh` で import
5. `SUMO_NET_FILE=examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml ./scripts/prepare_odaiba_ll2_sumo_artifacts.sh`
6. `CARLA_PACKAGE_MAP_NAME=odaibatest5 ./scripts/run_cosim_odaiba_ll2_packaged_generated.sh`
7. `./scripts/show_carla_vehicle_camera_novnc.sh`
