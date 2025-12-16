#!/usr/bin/env python3
"""Odaiba マップでのTeraSim デモ実行スクリプト"""

from pathlib import Path

from terasim.envs.template import EnvTemplate
from terasim.logger.infoextractor import InfoExtractor
from terasim.simulator import Simulator
from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from terasim.vehicle.decision_models.idm_model import IDMModel
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.vehicle import Vehicle


class ExampleVehicleFactory(VehicleFactory):
    """サンプル車両ファクトリー"""
    
    def create_vehicle(self, veh_id: str, simulator) -> Vehicle:
        """車両を生成する
        
        Args:
            veh_id: 車両ID
            simulator: シミュレーター（SUMO）
            
        Returns:
            Vehicle: 構築された車両オブジェクト
        """
        sensor_list = [EgoSensor(), LocalSensor(obs_range=40)]
        decision_model = IDMModel(MOBIL_lc_flag=False, stochastic_acc_flag=True)
        controller = HighEfficiencyController(simulator)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )


def main():
    """メイン処理"""
    current_path = Path(__file__).parent
    maps_path = current_path.parent / "maps" / "odaiba"
    output_path = current_path / "output" / "odaiba"
    
    print(f"マップディレクトリ: {maps_path}")
    print(f"出力ディレクトリ: {output_path}")
    
    # ファイルの存在確認
    net_file = maps_path / "odaiba.net.xml"
    config_file = maps_path / "odaiba.sumocfg"
    
    if not net_file.exists():
        raise FileNotFoundError(f"ネットワークファイルが見つかりません: {net_file}")
    if not config_file.exists():
        raise FileNotFoundError(f"設定ファイルが見つかりません: {config_file}")
    
    print(f"✅ ネットワークファイル: {net_file}")
    print(f"✅ 設定ファイル: {config_file}")
    print()
    print("⚙️  テレポート設定:")
    print("  - 通常道路: 300秒で立ち往生車両を削除")
    print("  - 高速道路: 120秒で削除")
    print("  - 接続なし: 60秒で削除")
    print()
    print("シミュレーションを開始します...")
    print("（実行時間: 約5-10分 - 5分間のシミュレーション）")
    print()
    
    # 環境とシミュレーターの設定
    env = EnvTemplate(vehicle_factory=ExampleVehicleFactory(), info_extractor=InfoExtractor)
    
    # テレポート設定を追加（立ち往生した車両を自動削除）
    sumo_args = [
        "--time-to-teleport", "300",            # 300秒（5分）立ち往生したら削除
        "--time-to-teleport.highways", "120",    # 高速道路では120秒（2分）で削除
        "--time-to-teleport.disconnected", "60", # 接続のないエッジでは60秒で削除
        "--max-depart-delay", "600",             # 出発遅延の最大値（10分）
    ]
    
    sim = Simulator(
        sumo_net_file_path=net_file,
        sumo_config_file_path=config_file,
        num_tries=10,
        gui_flag=False,
        output_path=output_path,
        sumo_output_file_types=["fcd_all"],
        additional_sumo_args=sumo_args,  # テレポート設定を追加
    )
    
    # シミュレーション実行
    sim.bind_env(env)
    sim.run()
    
    print()
    print("✅ シミュレーション完了！")
    print(f"出力ファイル: {output_path}")


if __name__ == "__main__":
    main()

