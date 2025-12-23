#!/usr/bin/env python3
"""
TeraSim + CARLA Co-Simulation with Recording
CPU環境で実行してデータ保存 → GPU環境で可視化再生
"""

import sys
import json
import time
from pathlib import Path

sys.path.insert(0, "/app/terasim/packages/terasim")

import carla
import traci

def detect_gpu_mode():
    """GPU/CPUモードを自動検出"""
    try:
        import torch
        if torch.cuda.is_available():
            return "gpu"
    except:
        pass
    return "cpu"

class CosimRecorder:
    """Co-Simulationの記録・再生クラス"""
    
    def __init__(self, output_file):
        self.output_file = Path(output_file)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.data = {
            "metadata": {
                "sumo_net": None,
                "carla_map": None,
                "step_length": 0.1,
                "total_steps": 0,
            },
            "frames": []
        }
    
    def set_metadata(self, sumo_net, carla_map, step_length):
        """メタデータを設定"""
        self.data["metadata"]["sumo_net"] = str(sumo_net)
        self.data["metadata"]["carla_map"] = carla_map
        self.data["metadata"]["step_length"] = step_length
    
    def record_frame(self, step, vehicles):
        """1フレームを記録"""
        frame = {
            "step": step,
            "vehicles": vehicles
        }
        self.data["frames"].append(frame)
    
    def save(self):
        """ファイルに保存"""
        self.data["metadata"]["total_steps"] = len(self.data["frames"])
        with open(self.output_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        print(f"📁 記録ファイル保存: {self.output_file} ({self.output_file.stat().st_size / 1024:.1f} KB)")
    
    @classmethod
    def load(cls, input_file):
        """ファイルから読み込み"""
        with open(input_file, 'r') as f:
            return json.load(f)


def run_cosim_with_recording(max_steps=500, output_file="/tmp/cosim_recording.json"):
    """Co-Simulationを実行してデータを記録"""
    
    print("=" * 70)
    print("🚗 TeraSim + CARLA Co-Simulation (記録モード)")
    print("=" * 70)
    print()
    
    mode = detect_gpu_mode()
    print(f"🖥️  検出モード: {mode.upper()}")
    print()
    
    # 設定
    sumo_net = "/app/terasim/examples/maps/odaiba/standard_odaiba.net.xml"
    sumo_cfg = "/app/terasim/examples/maps/odaiba/standard_odaiba.sumocfg"
    carla_host = "localhost"
    carla_port = 2000
    carla_map = "Town03"
    step_length = 0.1
    
    # 記録用オブジェクト
    recorder = CosimRecorder(output_file)
    recorder.set_metadata(sumo_net, carla_map, step_length)
    
    # CARLA接続
    print("⚙️  CARLA接続中...")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    print(f"✅ CARLA接続成功: {client.get_server_version()}")
    
    # CARLAマップロード
    print(f"⚙️  CARLAマップ '{carla_map}' 読み込み中...")
    world = client.load_world(carla_map)
    print("✅ マップ読み込み完了")
    
    # CARLA設定
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = step_length
    
    if mode == "gpu":
        # GPU環境：レンダリング有効
        settings.no_rendering_mode = False
        print("🎨 GPUモード: レンダリング有効")
    else:
        # CPU環境：レンダリング無効（高速化）
        settings.no_rendering_mode = True
        print("⚡ CPUモード: レンダリング無効（高速化）")
    
    world.apply_settings(settings)
    print("✅ CARLA同期モード有効化")
    
    # 車両ブループリント
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')[0]
    
    # SUMO起動
    print()
    print("⚙️  SUMO起動中...")
    sumo_cmd = [
        "sumo",
        "-c", sumo_cfg,
        "--step-length", str(step_length),
        "--no-warnings",
        "--no-step-log",
    ]
    
    traci.start(sumo_cmd)
    print("✅ SUMO起動完了")
    
    # アクター管理
    carla_actors = {}
    
    print()
    print("🚀 Co-Simulation開始...")
    print(f"   最大ステップ数: {max_steps}")
    print(f"   記録ファイル: {output_file}")
    print()
    
    try:
        for step in range(max_steps):
            # SUMOを1ステップ進める
            traci.simulationStep()
            
            # SUMO車両情報を取得
            sumo_vehicles = traci.vehicle.getIDList()
            vehicles_data = []
            
            for veh_id in sumo_vehicles:
                try:
                    x, y = traci.vehicle.getPosition(veh_id)
                    angle = traci.vehicle.getAngle(veh_id)
                    speed = traci.vehicle.getSpeed(veh_id)
                    
                    # 記録用データ
                    vehicles_data.append({
                        "id": veh_id,
                        "x": x,
                        "y": y,
                        "angle": angle,
                        "speed": speed,
                    })
                    
                    # CARLA座標に変換
                    carla_x = x * 0.1
                    carla_y = -y * 0.1
                    carla_yaw = -angle
                    
                    # 新規車両をスポーン
                    if veh_id not in carla_actors:
                        transform = carla.Transform(
                            carla.Location(x=carla_x, y=carla_y, z=1.0),
                            carla.Rotation(yaw=carla_yaw)
                        )
                        actor = world.try_spawn_actor(vehicle_bp, transform)
                        if actor:
                            carla_actors[veh_id] = actor
                    else:
                        # 既存車両の位置を更新
                        transform = carla.Transform(
                            carla.Location(x=carla_x, y=carla_y, z=1.0),
                            carla.Rotation(yaw=carla_yaw)
                        )
                        carla_actors[veh_id].set_transform(transform)
                
                except:
                    pass
            
            # 消えた車両を削除
            for veh_id in list(carla_actors.keys()):
                if veh_id not in sumo_vehicles:
                    try:
                        carla_actors[veh_id].destroy()
                    except:
                        pass
                    del carla_actors[veh_id]
            
            # フレームを記録
            recorder.record_frame(step, vehicles_data)
            
            # CARLAを更新
            world.tick()
            
            # 進捗表示
            if step % 50 == 0:
                print(f"  ステップ {step:3d}/{max_steps}: SUMO={len(sumo_vehicles)}台, CARLA={len(carla_actors)}台")
        
        print()
        print("=" * 70)
        print("✅ Co-Simulation完了！")
        print("=" * 70)
        print()
        
        # 記録を保存
        recorder.save()
        
        print()
        print("📊 結果:")
        print(f"  総ステップ数: {max_steps}")
        print(f"  シミュレーション時間: {max_steps * step_length:.1f}秒")
        print(f"  最終車両数: SUMO={len(sumo_vehicles)}, CARLA={len(carla_actors)}")
        print(f"  記録フレーム数: {len(recorder.data['frames'])}")
        print()
        
        if mode == "cpu":
            print("💡 次のステップ:")
            print(f"  1. 記録ファイルをコピー:")
            print(f"     docker cp terasim-kang:{output_file} ./")
            print()
            print(f"  2. GPU環境で再生:")
            print(f"     python3 run_cosim_replay.py --input {output_file}")
            print()
        else:
            print("🎨 GPU環境で実行されました。")
            print("   CARLAの3Dビジュアライゼーションが利用可能です。")
            print()
        
    except KeyboardInterrupt:
        print("\n⚠️  ユーザーによる中断")
    except Exception as e:
        print(f"\n❌ エラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # クリーンアップ
        print("🧹 クリーンアップ中...")
        
        for actor in carla_actors.values():
            try:
                actor.destroy()
            except:
                pass
        
        try:
            traci.close()
        except:
            pass
        
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except:
            pass
        
        print("✅ クリーンアップ完了")
    
    return 0


def replay_recording(input_file, playback_speed=1.0):
    """記録データをCARLAで再生（GPU環境用）"""
    
    print("=" * 70)
    print("🎬 Co-Simulation 再生モード (GPU環境)")
    print("=" * 70)
    print()
    
    # 記録データを読み込み
    print(f"📂 記録ファイル読み込み: {input_file}")
    data = CosimRecorder.load(input_file)
    
    metadata = data["metadata"]
    frames = data["frames"]
    
    print(f"✅ 記録データ読み込み完了")
    print(f"   総フレーム数: {len(frames)}")
    print(f"   シミュレーション時間: {metadata['total_steps'] * metadata['step_length']:.1f}秒")
    print()
    
    # CARLA接続
    print("⚙️  CARLA接続中...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    print(f"✅ CARLA接続成功: {client.get_server_version()}")
    
    # マップロード
    carla_map = metadata["carla_map"]
    print(f"⚙️  CARLAマップ '{carla_map}' 読み込み中...")
    world = client.load_world(carla_map)
    print("✅ マップ読み込み完了")
    
    # 設定
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = metadata["step_length"]
    settings.no_rendering_mode = False  # GPU: レンダリング有効
    world.apply_settings(settings)
    print("🎨 レンダリング有効化")
    
    # 車両ブループリント
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')[0]
    
    carla_actors = {}
    
    print()
    print("🎬 再生開始...")
    print(f"   再生速度: {playback_speed}x")
    print()
    
    try:
        for frame in frames:
            step = frame["step"]
            vehicles = frame["vehicles"]
            
            # 車両を配置
            vehicle_ids = {v["id"] for v in vehicles}
            
            for veh_data in vehicles:
                veh_id = veh_data["id"]
                x, y = veh_data["x"], veh_data["y"]
                angle = veh_data["angle"]
                
                # CARLA座標
                carla_x = x * 0.1
                carla_y = -y * 0.1
                carla_yaw = -angle
                
                transform = carla.Transform(
                    carla.Location(x=carla_x, y=carla_y, z=1.0),
                    carla.Rotation(yaw=carla_yaw)
                )
                
                if veh_id not in carla_actors:
                    actor = world.try_spawn_actor(vehicle_bp, transform)
                    if actor:
                        carla_actors[veh_id] = actor
                else:
                    carla_actors[veh_id].set_transform(transform)
            
            # 消えた車両を削除
            for veh_id in list(carla_actors.keys()):
                if veh_id not in vehicle_ids:
                    try:
                        carla_actors[veh_id].destroy()
                    except:
                        pass
                    del carla_actors[veh_id]
            
            # 更新
            world.tick()
            
            if step % 50 == 0:
                print(f"  フレーム {step}/{len(frames)}: 車両数={len(vehicles)}")
            
            # 再生速度調整
            if playback_speed < 999:  # 999 = 最高速
                time.sleep(metadata["step_length"] / playback_speed)
        
        print()
        print("=" * 70)
        print("✅ 再生完了！")
        print("=" * 70)
        print()
        
    except KeyboardInterrupt:
        print("\n⚠️  ユーザーによる中断")
    finally:
        # クリーンアップ
        print("🧹 クリーンアップ中...")
        for actor in carla_actors.values():
            try:
                actor.destroy()
            except:
                pass
        
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except:
            pass
        
        print("✅ クリーンアップ完了")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="TeraSim + CARLA Co-Simulation with Recording/Replay")
    parser.add_argument("--mode", choices=["record", "replay"], default="record",
                        help="record: Co-Simulation実行して記録, replay: 記録を再生")
    parser.add_argument("--max-steps", type=int, default=500,
                        help="最大ステップ数（recordモード）")
    parser.add_argument("--output", default="/tmp/cosim_recording.json",
                        help="記録ファイルのパス（recordモード）")
    parser.add_argument("--input", default="/tmp/cosim_recording.json",
                        help="再生する記録ファイル（replayモード）")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="再生速度（replayモード、999=最高速）")
    
    args = parser.parse_args()
    
    if args.mode == "record":
        return run_cosim_with_recording(args.max_steps, args.output)
    else:
        return replay_recording(args.input, args.speed)


if __name__ == "__main__":
    sys.exit(main())

