# お台場 Ackermann feedback GUI runner

CARLAを描画ありで起動し、TeraSim/SUMOの背景車両を物理ONのAckermann制御で走行させ、CARLAの実位置・速度をSUMOへfeedbackします。TeraSim側はFastAPI/Redisを使わず、`run_direct`のgRPC Tick経路を使用します。

## 起動

```bash
./scripts/run_ackermann_odaiba_feedback_gui.sh
```

起動後の表示先は次の通りです（初期パスワードは `headless`）。

- CARLA AV chase camera: `http://localhost:6092/vnc.html`
- SUMO GUI（AV自動追従）: `http://localhost:6093/vnc.html`

終了はrunnerを実行した端末で `Ctrl-C` です。runner終了時にはchase cameraを停止し、CARLA内のvehicle/sensor actorを削除します。終了時のactor削除を無効化する場合は `CLEAN_CARLA_ACTORS_ON_EXIT=0` を指定します。CARLA表示コンテナも終了する場合は次を実行します。

```bash
docker compose -f docker-compose.carla-novnc.yml down
```

## 既定条件

- map: `odaiba_tl_mapping`
- CARLA: synchronous、`fixed_delta_seconds=0.1`、rendering ON
- vehicle control: `ackermann_physics`
- feedback: `apply`、対象 `*`（AV以外の背景車両）
- CARLA actor / TeraSim state filter: AV中心300 m
- SUMO: `sumo-gui`（`USE_LIBSUMO=0`）
- chase camera: CARLA actorの `role_name=AV`

主な上書き例です。

```bash
VNC_PASSWORD=secret \
SUMO_GUI_REALTIME=0 \
CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='*,AV' \
./scripts/run_ackermann_odaiba_feedback_gui.sh
```

`SUMO_GUI_REALTIME=0` はGUIを表示したまま実時間待ちを無効化します。`CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='*,AV'` はAVもfeedback対象に加えます。

CARLA map packageが未導入の場合、runnerは `examples/maps/odaiba_ll2/tlmappings_0708/odaiba_tl_mapping_294096eb1-dirty.tar.gz` を自動importしてCARLAを再起動します。初回のTeraSim GUI image buildとmap importには時間がかかります。
