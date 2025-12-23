#!/usr/bin/env python3
"""
お台場マップをOpenDRIVE形式に変換してCARLAで使えるようにする
netconvert を直接使用
"""

import sys
import os
import subprocess
from pathlib import Path

def main():
    print("=" * 70)
    print("🗺️  お台場マップ変換: OSM → OpenDRIVE (.xodr)")
    print("=" * 70)
    print()
    
    # パス設定
    maps_dir = Path("/app/terasim/examples/maps/odaiba")
    osm_file = maps_dir / "standard_odaiba_map.osm"
    net_file = maps_dir / "odaiba_for_carla.net.xml"
    xodr_file = maps_dir / "odaiba_carla.xodr"
    
    if not osm_file.exists():
        print(f"❌ OSMファイルが見つかりません: {osm_file}")
        return 1
    
    print(f"📂 入力OSMファイル: {osm_file}")
    print(f"   サイズ: {osm_file.stat().st_size / 1024 / 1024:.1f} MB")
    print()
    
    # ステップ1: OSM → SUMO network
    print("⚙️  ステップ1: OSM → SUMO network (.net.xml)")
    cmd_osm_to_net = [
        "netconvert",
        "--osm-files", str(osm_file),
        "--output-file", str(net_file),
        "--geometry.remove",
        "--ramps.guess",
        "--junctions.join",
        "--tls.guess-signals",
        "--tls.discard-simple",
        "--tls.join",
        "--no-internal-links",
        "--verbose",
    ]
    
    try:
        print(f"   コマンド: {' '.join(cmd_osm_to_net[:5])} ...")
        result = subprocess.run(
            cmd_osm_to_net,
            capture_output=True,
            text=True,
            timeout=300
        )
        
        if result.returncode == 0:
            print(f"✅ SUMO network生成成功: {net_file}")
            if net_file.exists():
                print(f"   サイズ: {net_file.stat().st_size / 1024:.1f} KB")
        else:
            print(f"❌ エラー: {result.stderr[:500]}")
            return 1
            
    except subprocess.TimeoutExpired:
        print("❌ タイムアウト（300秒）")
        return 1
    except Exception as e:
        print(f"❌ エラー: {e}")
        return 1
    
    print()
    
    # ステップ2: SUMO network → OpenDRIVE
    print("⚙️  ステップ2: SUMO network → OpenDRIVE (.xodr)")
    cmd_net_to_xodr = [
        "netconvert",
        "--sumo-net-file", str(net_file),
        "--opendrive-output", str(xodr_file),
        "--verbose",
    ]
    
    try:
        print(f"   コマンド: {' '.join(cmd_net_to_xodr[:4])} ...")
        result = subprocess.run(
            cmd_net_to_xodr,
            capture_output=True,
            text=True,
            timeout=300
        )
        
        if result.returncode == 0:
            print(f"✅ OpenDRIVE生成成功: {xodr_file}")
            if xodr_file.exists():
                file_size_kb = xodr_file.stat().st_size / 1024
                if file_size_kb > 1024:
                    print(f"   サイズ: {file_size_kb / 1024:.1f} MB")
                else:
                    print(f"   サイズ: {file_size_kb:.1f} KB")
        else:
            print(f"❌ エラー: {result.stderr[:500]}")
            return 1
            
    except subprocess.TimeoutExpired:
        print("❌ タイムアウト（300秒）")
        return 1
    except Exception as e:
        print(f"❌ エラー: {e}")
        return 1
    
    print()
    print("=" * 70)
    print("✅ 変換完了！")
    print("=" * 70)
    print()
    print(f"📁 生成ファイル:")
    print(f"   1. SUMO network:  {net_file}")
    print(f"   2. OpenDRIVE:     {xodr_file}")
    print()
    print("🎯 次のステップ:")
    print("   1. CARLAでお台場マップをロード:")
    print(f"      python3 -c \"")
    print(f"import carla")
    print(f"client = carla.Client('localhost', 2000)")
    print(f"client.set_timeout(10.0)")
    print(f"with open('{xodr_file}', 'r') as f:")
    print(f"    xodr_data = f.read()")
    print(f"world = client.generate_opendrive_world(xodr_data)")
    print(f"print('お台場マップロード成功！')")
    print(f"      \"")
    print()
    print("   2. お台場SUMO + お台場CARLAでCo-Simulation実行")
    print("      → 正確な位置同期が可能！")
    print()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
