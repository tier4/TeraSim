# CARLA AV Visual Jitter With Follow Camera

Date: 2026-05-05

## Summary

Odaiba packaged CARLA co-simulation で `./scripts/follow_carla_actor_novnc.sh`
を使って AV を追尾すると、CARLA の noVNC 画面上では AV が前後に揺れているように見える。

調査結果として、この現象は主に可視化上の問題であり、CARLA API 上の AV transform が継続的に前後へ戻っているわけではない。

外部 sampler では、修正後の co-sim 実行中に AV の後退 step は検出されなかった。

```text
outputs/fix_external_motion.csv
samples=430
moving=358
distance=371.15 m
signed=371.08 m
backward_events=0
max_step=2.01 m
```

45 秒で 430 samples なので、外部観測ではおよそ 9.56 Hz で CARLA tick を受け取れている。
これは設定値の 10 Hz にかなり近い。

## Visual Symptom

noVNC 上で chase camera を使うと、次のように見えることがある。

- AV が前後に震える、または一瞬戻ったように見える
- `AV [actor_id]` の文字が 2 個または 3 個重なって見える
- 一部 frame で画面が詰まり、そのあと車両や camera が飛ぶように見える

ただし、`outputs/fix_external_motion.csv` の結果では `backward_events=0` であり、
API 上の AV transform は後退していない。

## Cause

主な原因は `follow_carla_actor_novnc.sh` による追尾 camera と debug 描画の見え方である。

### 1. Debug text の残像

旧 `follow_carla_actor_novnc.sh` は毎 `0.05 s` ごとに
`world.debug.draw_string()` を呼び、`life_time=0.1` で label を描画していた。

```text
UPDATE_INTERVAL=0.05 s
label life_time=0.1 s
```

つまり label の寿命が更新間隔より長いため、仕様上、同じ `AV [actor_id]` が
2 個以上残って見える。noVNC や browser 側の描画遅延があると 3 個程度に見えることもある。

この文字の多重表示は、actor が複数存在することや AV が実際に前後していることを意味しない。

### 2. Chase camera が 10 Hz の離散更新を強調する

SUMO/CARLA co-sim は `fixed_delta_seconds=0.1` なので、AV の CARLA transform は基本的に 10 Hz の離散更新である。
`follow_carla_actor_novnc.sh` は AV の transform を読んで `spectator.set_transform()` で camera を追従させる。

chase camera では camera が AV の後方に固定されるため、車両の 10 Hz 離散移動、描画 frame の欠落、
一部 tick の stall が視覚的に増幅される。

その結果、実際には前進している AV が、画面上では一瞬前後に揺れているように見える。

### 3. 一部 frame の stall

co-sim の actor lookup bottleneck 修正後、通常 tick は大きく改善した。

```text
lookup_calls median: 353 -> 0
lookup_total median: 0.5869 s -> 0.0 s
sync_actor median:   0.572 s  -> 0.025 s
tick total median:   0.623 s  -> 0.115 s
```

一方で、まれに数秒級の重い frame が残っている。

```text
outputs/fix_actor_profile.csv
spawn_total max 7.0736 s
sync_actor max 9.6883 s

outputs/fix_tick_profile.csv
total max 9.7362 s
```

これは通常時の連続的な重さではなく、actor spawn や CARLA/Unreal 側の一時的な stall と考えられる。
この stall が発生すると noVNC 上では frame が飛び、追尾 camera では AV が一瞬戻る、または飛ぶように見える。

## What The Follow Script Does

`follow_carla_actor_novnc.sh` は AV の車両 transform を変更しない。

実施しているのは次の 2 つだけである。

- `world.debug.draw_string()` による label 表示
- `spectator.set_transform()` による camera 位置更新

したがって、この script 自体が AV actor に `set_transform()` をかけて前後に動かしているわけではない。

## Script Fix

可視化 artifact を減らすため、`scripts/follow_carla_actor_novnc.sh` を修正した。

変更内容:

- debug label を default OFF にした
- `WAIT_FOR_TICK=1` を追加し、spectator 更新を CARLA tick に同期できるようにした
- label を使う場合の寿命を短くできる `LABEL_LIFE_TIME` を追加した

推奨実行:

```bash
cd /home/h-kawai/TeraSim

WAIT_FOR_TICK=1 \
DRAW_LABEL=0 \
CAMERA_MODE=chase \
./scripts/follow_carla_actor_novnc.sh
```

真上から確認する場合:

```bash
cd /home/h-kawai/TeraSim

WAIT_FOR_TICK=1 \
DRAW_LABEL=0 \
CAMERA_MODE=topdown \
TOPDOWN_HEIGHT=45 \
./scripts/follow_carla_actor_novnc.sh
```

label を出したい場合:

```bash
cd /home/h-kawai/TeraSim

WAIT_FOR_TICK=1 \
DRAW_LABEL=1 \
LABEL_LIFE_TIME=0.03 \
./scripts/follow_carla_actor_novnc.sh
```

## How To Stop Follow Process

CARLA noVNC container 内の follow process だけ止める。

```bash
docker exec carla-novnc-test bash -lc \
"pkill -TERM -f 'python3.10 -' || true"
```

確認:

```bash
docker exec carla-novnc-test bash -lc \
"ps -eo pid,ppid,etime,cmd | grep 'python3.10 -' | grep -v grep || true"
```

何も表示されなければ follow process は停止済み。

## Conclusion

AV が前後に動いて見える現象は、今回の計測では CARLA actor transform の後退ではなく、
可視化上の問題と判断する。

特に chase camera による追尾、debug label の残像、noVNC/browser の描画遅延、
およびまれな重い frame が組み合わさることで、AV が前後しているように見える。

今後さらに改善する場合は、次を切り分け対象にする。

- actor spawn 発生時の stall 削減
- noVNC/browser ではなく native display または録画 sensor で確認
- spectator follow ではなく topdown 固定視点で確認
- chase camera の位置を補間し、camera だけを滑らかにする
