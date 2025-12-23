#!/usr/bin/env python3
"""
お台場マップでTeraSim + CARLA Co-Simulation（正確な位置同期版）
- SUMOマップ: standard_odaiba.net.xml
- CARLAマップ: odaiba_carla.xodr (同じお台場データから生成)
- 座標変換: 正確な座標マッピング（同一座標系）
"""

import sys
import carla
import traci
import time
import random
from pathlib import Path
import json

def detect_gpu_mode():
    """GPU/CPUモードを自動検出"""
    try:
        import torch
        if torch.cuda.is_available():
            return "gpu"
    except:
        pass
    return "cpu"

class AccurateCosimRecorder:
    """正確な座標同期でのCo-Simulation記録"""
    
    def __init__(self, output_file):
        self.output_file = Path(output_file)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.data = {
            "metadata": {
                "sumo_net": None,
                "carla_xodr": None,
                "coordinate_system": "same_origin",  # 同じ座標系
                "step_length": 0.1,
                "total_steps": 0,
            },
            "frames": []
        }
    
    def set_metadata(self, sumo_net, carla_xodr, step_length):
        self.data["metadata"]["sumo_net"] = str(sumo_net)
        self.data["metadata"]["carla_xodr"] = str(carla_xodr)
        self.data["metadata"]["step_length"] = step_length
    
    def record_frame(self, step, vehicles):
        self.data["frames"].append({"step": step, "vehicles": vehicles})
    
    def save(self):
        self.data["metadata"]["total_steps"] = len(self.data["frames"])
        with open(self.output_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        file_size = self.output_file.stat().st_size
        if file_size > 1024 * 1024:
            print(f"📁 記録ファイル保存: {self.output_file} ({file_size / 1024 / 1024:.1f} MB)")
        else:
            print(f"📁 記録ファイル保存: {self.output_file} ({file_size / 1024:.1f} KB)")

def sumo_to_carla_transform_accurate(sumo_x, sumo_y, sumo_angle):
    """
    正確な座標変換（同じOSMデータから生成されたマップなので座標系は同じ）
    - SUMOとCARLAは同じUTM座標を使用
    - 軸の向きだけ調整
    """
    # CARLAはUnreal Engine座標系（左手座標系、Z-up）
    # SUMOは通常のXY座標系
    
    # 座標変換（スケールは1:1、軸の向きだけ調整）
    carla_x = sumo_x
    carla_y = -sumo_y  # Y軸反転
    carla_z = 0.5  # 地面の上
    
    # 角度変換（SUMOは北が0度時計回り、CARLAはYawで表現）
    carla_yaw = -sumo_angle  # 反転
    
    return carla.Transform(
        carla.Location(x=carla_x, y=carla_y, z=carla_z),
        carla.Rotation(yaw=carla_yaw)
    )

def main():
    print("=" * 70)
    print("🚗 TeraSim + CARLA Co-Simulation（正確な位置同期版）")
    print("   SUMOマップ: お台場（standard_odaiba）")
    print("   CARLAマップ: お台場（odaiba_carla.xodr）")
    print("=" * 70)
    print()
    
    mode = detect_gpu_mode()
    print(f"🖥️  検出モード: {mode.upper()}")
    print()
    
    # パス設定
    maps_dir = Path("/app/terasim/examples/maps/odaiba")
    sumo_net = maps_dir / "standard_odaiba.net.xml"
    sumo_cfg = maps_dir / "standard_odaiba.sumocfg"
    carla_xodr = maps_dir / "odaiba_carla.xodr"
    output_file = "/tmp/odaiba_accurate_cosim.json"
    
    max_steps = 500
    step_length = 0.1
    
    # 記録用
    recorder = AccurateCosimRecorder(output_file)
    recorder.set_metadata(sumo_net, carla_xodr, step_length)
    
    # CARLA接続
    print("⚙️  CARLA接続中...")
    carla_host = "localhost"
    carla_port = 2000
    
    try:
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(10.0)
        print(f"✅ CARLA接続成功: {client.get_server_version()}")
    except Exception as e:
        print(f"❌ CARLA接続失敗: {e}")
        return 1
    
    # お台場OpenDRIVEマップをCARLAにロード
    print()
    print("⚙️  お台場OpenDRIVEマップをCARLAにロード中...")
    print(f"   ファイル: {carla_xodr}")
    print(f"   サイズ: {carla_xodr.stat().st_size / 1024 / 1024:.1f} MB")
    
    try:
        with open(carla_xodr, 'r', encoding='utf-8') as f:
            xodr_data = f.read()
        
        # OpenDRIVEからワールドを生成
        vertex_distance = 2.0  # メートル単位での頂点間隔
        max_road_length = 500.0  # 最大道路長
        wall_height = 0.0  # 壁の高さ（0=壁なし）
        extra_width = 0.6  # 余分な幅
        
        world = client.generate_opendrive_world(
            xodr_data,
            carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True
            )
        )
        
        print("✅ お台場マップロード成功！")
        
    except Exception as e:
        print(f"❌ マップロード失敗: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # CARLA設定
    print()
    print("⚙️  CARLA設定中...")
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = step_length
    
    if mode == "gpu":
        settings.no_rendering_mode = False
        print("🎨 GPUモード: レンダリング有効")
    else:
        settings.no_rendering_mode = True
        print("⚡ CPUモード: レンダリング無効（高速化）")
    
    world.apply_settings(settings)
    print("✅ CARLA同期モード有効化")
    
    # 車両ブループリント
    blueprint_library = world.get_blueprint_library()
    vehicle_bps = blueprint_library.filter('vehicle.*')
    
    # SUMO起動
    print()
    print("⚙️  SUMO起動中...")
    print(f"   マップ: {sumo_cfg}")
    
    sumo_port = random.randint(30000, 40000)
    sumo_cmd = [
        "sumo",
        "-c", str(sumo_cfg),
        "--step-length", str(step_length),
        "--no-warnings",
        "--no-step-log",
    ]
    
    try:
        traci.start(sumo_cmd, port=sumo_port)
        print("✅ SUMO起動完了")
    except Exception as e:
        print(f"❌ SUMO起動失敗: {e}")
        return 1
    
    # Co-Simulationループ
    print()
    print("🚀 Co-Simulation開始...")
    print(f"   最大ステップ数: {max_steps}")
    print(f"   座標系: 同一（正確な位置同期）")
    print()
    
    carla_actors = {}
    
    try:
        for step in range(max_steps):
            # SUMOステップ
            traci.simulationStep()
            
            # SUMO車両情報取得
            sumo_vehicles = traci.vehicle.getIDList()
            vehicles_data = []
            
            for veh_id in sumo_vehicles:
                try:
                    x, y = traci.vehicle.getPosition(veh_id)
                    angle = traci.vehicle.getAngle(veh_id)
                    speed = traci.vehicle.getSpeed(veh_id)
                    
                    vehicles_data.append({
                        "id": veh_id,
                        "x": x,
                        "y": y,
                        "angle": angle,
                        "speed": speed,
                    })
                    
                    # CARLA座標変換（正確な変換）
                    carla_transform = sumo_to_carla_transform_accurate(x, y, angle)
                    
                    # 新規車両をスポーン
                    if veh_id not in carla_actors:
                        bp = random.choice(vehicle_bps)
                        actor = world.try_spawn_actor(bp, carla_transform)
                        if actor:
                            carla_actors[veh_id] = actor
                    else:
                        # 既存車両の位置更新
                        carla_actors[veh_id].set_transform(carla_transform)
                
                except Exception as e:
                    pass
            
            # 消えた車両を削除
            for veh_id in list(carla_actors.keys()):
                if veh_id not in sumo_vehicles:
                    try:
                        carla_actors[veh_id].destroy()
                    except:
                        pass
                    del carla_actors[veh_id]
            
            # フレーム記録
            recorder.record_frame(step, vehicles_data)
            
            # CARLAステップ
            world.tick()
            
            # 進捗表示
            if step % 50 == 0:
                print(f"  ステップ {step:3d}/{max_steps}: SUMO={len(sumo_vehicles)}台, CARLA={len(carla_actors)}台")
        
        print()
        print("=" * 70)
        print("✅ Co-Simulation完了！")
        print("=" * 70)
        print()
        
        # 記録保存
        recorder.save()
        
        print()
        print("📊 結果:")
        print(f"  総ステップ数: {max_steps}")
        print(f"  シミュレーション時間: {max_steps * step_length:.1f}秒")
        print(f"  最終車両数: SUMO={len(sumo_vehicles)}, CARLA={len(carla_actors)}")
        print(f"  記録フレーム数: {len(recorder.data['frames'])}")
        print()
        print("✅ 正確な位置同期:")
        print("   SUMOとCARLAで同じお台場マップを使用")
        print("   座標系は同一（1:1マッピング）")
        print()
        
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️  ユーザーによる中断")
        return 1
    except Exception as e:
        print(f"\n❌ エラー: {e}")
        import traceback
        traceback.print_exc()
        return 1
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

if __name__ == "__main__":
    sys.exit(main())

