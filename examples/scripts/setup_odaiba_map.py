#!/usr/bin/env python3
"""
Odaiba Map Setup Script for TeraSim
====================================
Downloads standard OSM data for the Odaiba area, converts it to SUMO format,
and generates OpenDRIVE (.xodr) file for CARLA co-simulation.

This script should be run inside the TeraSim Docker container where SUMO tools
(netconvert, etc.) are available.

Usage:
    python3 /app/examples/scripts/setup_odaiba_map.py
"""

import os
import subprocess
import sys
import urllib.request
import urllib.parse
import xml.etree.ElementTree as ET

# ─── Configuration ───────────────────────────────────────────────────────────

# Odaiba area bounds (from Lanelet2 map analysis)
# Focused area for a manageable simulation
SOUTH = 35.615
NORTH = 35.630
WEST = 139.770
EAST = 139.785

# Output directory
OUTPUT_DIR = "/app/examples/maps/odaiba"

# File names
OSM_FILE = os.path.join(OUTPUT_DIR, "odaiba.osm")
NET_FILE = os.path.join(OUTPUT_DIR, "odaiba.net.xml")
ROU_FILE = os.path.join(OUTPUT_DIR, "odaiba.rou.xml")
CFG_FILE = os.path.join(OUTPUT_DIR, "odaiba.sumocfg")
ADD_FILE = os.path.join(OUTPUT_DIR, "odaiba.add.xml")
XODR_FILE = os.path.join(OUTPUT_DIR, "odaiba_carla.xodr")


def download_osm():
    """Download standard OSM data from OpenStreetMap API."""
    print(f"[1/5] Downloading OSM data for Odaiba area...")
    print(f"  Bounds: S={SOUTH}, W={WEST}, N={NORTH}, E={EAST}")
    
    # Use the main OSM API (more reliable than Overpass for bounded queries)
    url = f"https://api.openstreetmap.org/api/0.6/map?bbox={WEST},{SOUTH},{EAST},{NORTH}"
    
    try:
        req = urllib.request.Request(url)
        req.add_header('User-Agent', 'TeraSim/1.0')
        with urllib.request.urlopen(req, timeout=120) as response:
            osm_data = response.read()
        
        with open(OSM_FILE, "wb") as f:
            f.write(osm_data)
        
        size_kb = len(osm_data) / 1024
        print(f"  ✅ Downloaded OSM data: {size_kb:.1f} KB -> {OSM_FILE}")
        return True
    except Exception as e:
        print(f"  ❌ Failed to download from main API: {e}")
        print(f"  Trying Overpass API...")
        return download_osm_overpass()


def download_osm_overpass():
    """Fallback: Download OSM data from Overpass API."""
    
    query = f"""
    [out:xml][timeout:180];
    (
      way["highway"~"motorway|trunk|primary|secondary|tertiary|residential|service|unclassified|living_street"]({SOUTH},{WEST},{NORTH},{EAST});
    );
    (._;>;);
    out body;
    """
    
    url = "https://overpass-api.de/api/interpreter"
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    
    try:
        req = urllib.request.Request(url, data=data)
        with urllib.request.urlopen(req, timeout=180) as response:
            osm_data = response.read()
        
        with open(OSM_FILE, "wb") as f:
            f.write(osm_data)
        
        size_kb = len(osm_data) / 1024
        print(f"  ✅ Downloaded OSM data: {size_kb:.1f} KB -> {OSM_FILE}")
        return True
    except Exception as e:
        print(f"  ❌ Overpass download also failed: {e}")
        return False


def convert_osm_to_sumo():
    """Convert OSM to SUMO network using netconvert."""
    print(f"\n[2/5] Converting OSM to SUMO network...")
    
    cmd = [
        "netconvert",
        "--osm-files", OSM_FILE,
        "--output-file", NET_FILE,
        # Geometry options
        "--geometry.remove", "true",
        "--roundabouts.guess", "true",
        "--ramps.guess", "true",
        # Junction options
        "--junctions.join", "true",
        "--tls.guess", "true",
        "--tls.join", "true",
        # Edge options
        "--osm.all-attributes", "true",
        "--osm.speedlimit-none", "13.89",  # 50 km/h default
        # Keep right-hand drive (Japan)
        "--lefthand", "false",
        # Remove unused edges to keep network manageable
        "--remove-edges.isolated", "true",
        "--keep-edges.by-vclass", "passenger",
        # Output options
        "--output.street-names", "true",
        "--output.original-names", "true",
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if result.returncode == 0:
            size_mb = os.path.getsize(NET_FILE) / (1024 * 1024)
            print(f"  ✅ Created SUMO network: {size_mb:.1f} MB -> {NET_FILE}")
            if result.stderr:
                warnings = [l for l in result.stderr.split("\n") if "Warning" in l]
                if warnings:
                    print(f"  ⚠ {len(warnings)} warnings during conversion")
            return True
        else:
            print(f"  ❌ netconvert failed:")
            print(f"  stdout: {result.stdout[:500]}")
            print(f"  stderr: {result.stderr[:500]}")
            return False
    except Exception as e:
        print(f"  ❌ Error running netconvert: {e}")
        return False


def generate_routes():
    """Generate random traffic routes for the SUMO network."""
    print(f"\n[3/5] Generating traffic routes...")
    
    # Parse network to get edge IDs
    try:
        tree = ET.parse(NET_FILE)
        root = tree.getroot()
        
        # Get all non-internal edges that allow passenger vehicles
        edges = []
        for edge in root.findall(".//edge"):
            edge_id = edge.get("id", "")
            if edge_id.startswith(":"):  # Skip internal edges
                continue
            # Check if edge allows passenger vehicles
            for lane in edge.findall("lane"):
                allow = lane.get("allow", "")
                disallow = lane.get("disallow", "")
                if "passenger" not in disallow:
                    edges.append(edge_id)
                    break
        
        print(f"  Found {len(edges)} edges in network")
        
        if len(edges) < 2:
            print(f"  ❌ Not enough edges to create routes")
            return False
        
        # Create vehicle types and random routes
        routes_xml = ET.Element("routes")
        routes_xml.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
        routes_xml.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")
        
        # Vehicle types similar to Mcity
        vtype_urban = ET.SubElement(routes_xml, "vType")
        vtype_urban.set("id", "NDE_URBAN")
        vtype_urban.set("length", "5.00")
        vtype_urban.set("width", "1.85")
        vtype_urban.set("minGap", "3.28")
        vtype_urban.set("maxSpeed", "13.89")
        vtype_urban.set("carFollowModel", "IDM")
        vtype_urban.set("accel", "1.84")
        vtype_urban.set("decel", "1.29")
        vtype_urban.set("tau", "1.17")
        vtype_urban.set("emergencyDecel", "7.06")
        vtype_urban.set("lcSpeedGain", "0")
        vtype_urban.set("lcCooperative", "0")
        vtype_urban.set("lcKeepRight", "1")
        vtype_urban.set("speedFactor", "normc(1,0.1,0.8,1.2)")
        
        vtype_highway = ET.SubElement(routes_xml, "vType")
        vtype_highway.set("id", "NDE_HIGHWAY")
        vtype_highway.set("length", "5.00")
        vtype_highway.set("width", "1.85")
        vtype_highway.set("minGap", "5.92")
        vtype_highway.set("maxSpeed", "28.31")
        vtype_highway.set("carFollowModel", "IDM")
        vtype_highway.set("accel", "5.95")
        vtype_highway.set("decel", "5.96")
        vtype_highway.set("tau", "1.72")
        vtype_highway.set("emergencyDecel", "7.06")
        vtype_highway.set("lcSpeedGain", "0")
        vtype_highway.set("lcCooperative", "0")
        vtype_highway.set("lcKeepRight", "1")
        vtype_highway.set("speedFactor", "normc(1,0.1,0.8,1.2)")
        
        # Generate random flows using randomTrips approach
        # We'll use SUMO's randomTrips.py tool
        random_trips_cmd = [
            "python3", "-c",
            f"""
import subprocess, sys
result = subprocess.run([
    'python3', '/usr/local/share/sumo/tools/randomTrips.py',
    '-n', '{NET_FILE}',
    '-o', '{ROU_FILE}',
    '-b', '0',
    '-e', '3600',
    '-p', '3.0',
    '--vehicle-class', 'passenger',
    '--trip-attributes', 'type="NDE_URBAN"',
    '--validate',
    '--route-file', '{ROU_FILE}',
    '--additional-file', '{ROU_FILE.replace(".rou.xml", ".vtype.xml")}',
], capture_output=True, text=True)
print(result.stdout)
if result.returncode != 0:
    print(result.stderr, file=sys.stderr)
    sys.exit(result.returncode)
"""
        ]
        
        # Alternative: generate flows directly
        # Create flows for random trips through the network
        import random
        random.seed(42)
        
        num_routes = min(20, len(edges) // 2)
        route_edges_list = []
        
        for i in range(num_routes):
            from_edge = random.choice(edges)
            to_edge = random.choice(edges)
            while to_edge == from_edge:
                to_edge = random.choice(edges)
            route_edges_list.append((from_edge, to_edge))
        
        # Write a trips file for duarouter to compute actual routes
        trips_file = os.path.join(OUTPUT_DIR, "odaiba.trips.xml")
        trips_xml = ET.Element("routes")
        
        # Add vehicle types
        for vtype in [vtype_urban, vtype_highway]:
            trips_xml.append(vtype)
        
        for i, (from_e, to_e) in enumerate(route_edges_list):
            trip = ET.SubElement(trips_xml, "flow")
            trip.set("id", f"BV_{i}")
            trip.set("type", "NDE_URBAN")
            trip.set("begin", "0.00")
            trip.set("from", from_e)
            trip.set("to", to_e)
            trip.set("departLane", "best")
            trip.set("vehsPerHour", str(random.randint(20, 100)))
        
        tree = ET.ElementTree(trips_xml)
        ET.indent(tree, space="    ")
        tree.write(trips_file, xml_declaration=True, encoding="UTF-8")
        
        # Use duarouter to compute actual routes when possible
        duarouter_cmd = [
            "duarouter",
            "--net-file", NET_FILE,
            "--route-files", trips_file,
            "--output-file", ROU_FILE,
            "--ignore-errors", "true",
            "--no-step-log", "true",
        ]
        
        try:
            result = subprocess.run(duarouter_cmd, capture_output=True, text=True, timeout=60)
            if result.returncode == 0 and os.path.exists(ROU_FILE):
                print(f"  ✅ Generated routes with duarouter: {ROU_FILE}")
                return True
            else:
                print(f"  ⚠ duarouter failed, using trips file directly")
                # Fall back: just use the trips file as routes
                import shutil
                shutil.copy(trips_file, ROU_FILE)
                print(f"  ✅ Using trips as routes: {ROU_FILE}")
                return True
        except FileNotFoundError:
            print(f"  ⚠ duarouter not found, using trips file directly")
            import shutil
            shutil.copy(trips_file, ROU_FILE)
            print(f"  ✅ Using trips as routes: {ROU_FILE}")
            return True
            
    except Exception as e:
        print(f"  ❌ Error generating routes: {e}")
        import traceback
        traceback.print_exc()
        return False


def generate_additional_file():
    """Generate SUMO additional file (empty for now)."""
    print(f"\n[3.5/5] Generating additional file...")
    
    add_xml = '<?xml version="1.0" encoding="UTF-8"?>\n'
    add_xml += '<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
    add_xml += 'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">\n'
    add_xml += '</additional>\n'
    
    with open(ADD_FILE, "w") as f:
        f.write(add_xml)
    
    print(f"  ✅ Created additional file: {ADD_FILE}")
    return True


def generate_sumocfg():
    """Generate SUMO configuration file."""
    print(f"\n[4/5] Generating SUMO configuration...")
    
    cfg_xml = f"""<?xml version="1.0" encoding="UTF-8"?>

<configuration>

    <input>
        <net-file value="odaiba.net.xml"/>
        <route-files value="odaiba.rou.xml"/>
        <additional-files value="odaiba.add.xml"/>
        <step-length value="0.1"/>
        <scale value="0.8"/>
    </input>

    <time>
        <begin value="0"/>
        <end value="3600"/>
    </time>

    <random_number>
        <random value="true"/>
    </random_number>

    <processing>
        <lateral-resolution value="0.5"/>
        <time-to-teleport value="-1"/>
        <collision.action value="warn"/>
        <collision.check-junctions value="true"/>
        <collision.mingap-factor value="0"/>
    </processing>
</configuration>
"""
    
    with open(CFG_FILE, "w") as f:
        f.write(cfg_xml)
    
    print(f"  ✅ Created SUMO config: {CFG_FILE}")
    return True


def generate_xodr():
    """Generate OpenDRIVE (.xodr) from SUMO network for CARLA."""
    print(f"\n[5/5] Generating OpenDRIVE for CARLA...")
    
    cmd = [
        "netconvert",
        "--sumo-net-file", NET_FILE,
        "--opendrive-output", XODR_FILE,
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        if result.returncode == 0 and os.path.exists(XODR_FILE):
            size_mb = os.path.getsize(XODR_FILE) / (1024 * 1024)
            print(f"  ✅ Created OpenDRIVE: {size_mb:.1f} MB -> {XODR_FILE}")
            return True
        else:
            print(f"  ❌ OpenDRIVE generation failed:")
            print(f"  stderr: {result.stderr[:500]}")
            return False
    except Exception as e:
        print(f"  ❌ Error generating OpenDRIVE: {e}")
        return False


def get_network_info():
    """Print network statistics."""
    try:
        tree = ET.parse(NET_FILE)
        root = tree.getroot()
        
        edges = [e for e in root.findall(".//edge") if not e.get("id", "").startswith(":")]
        junctions = root.findall(".//junction")
        tls = root.findall(".//tlLogic")
        
        print(f"\n═══════════════════════════════════════")
        print(f"   Odaiba Network Summary")
        print(f"═══════════════════════════════════════")
        print(f"  Edges:     {len(edges)}")
        print(f"  Junctions: {len(junctions)}")
        print(f"  Traffic Lights: {len(tls)}")
        
        # Get some edge IDs for route configuration
        edge_ids = [e.get("id") for e in edges[:10]]
        print(f"  Sample edges: {edge_ids}")
        print(f"═══════════════════════════════════════\n")
        
        return edge_ids
    except Exception as e:
        print(f"  Error reading network info: {e}")
        return []


def main():
    print("=" * 50)
    print("  Odaiba Map Setup for TeraSim")
    print("=" * 50)
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Step 1: Download OSM (or use existing)
    if os.path.exists(OSM_FILE):
        size_kb = os.path.getsize(OSM_FILE) / 1024
        print(f"[1/5] Using existing OSM file: {OSM_FILE} ({size_kb:.1f} KB)")
    elif not download_osm():
        print("\n❌ FATAL: Could not download OSM data. Exiting.")
        sys.exit(1)
    
    # Step 2: Convert to SUMO
    if not convert_osm_to_sumo():
        print("\n❌ FATAL: Could not convert to SUMO network. Exiting.")
        sys.exit(1)
    
    # Step 3: Generate routes
    if not generate_routes():
        print("\n⚠ Warning: Could not generate routes.")
    
    # Step 3.5: Generate additional file
    generate_additional_file()
    
    # Step 4: Generate SUMO config
    generate_sumocfg()
    
    # Step 5: Generate OpenDRIVE
    xodr_ok = generate_xodr()
    
    # Print network info
    get_network_info()
    
    print("\n✅ Setup complete!")
    print(f"  SUMO files: {OUTPUT_DIR}/")
    if xodr_ok:
        print(f"  CARLA OpenDRIVE: {XODR_FILE}")
    else:
        print(f"  ⚠ CARLA OpenDRIVE not available - will run TeraSim only")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
