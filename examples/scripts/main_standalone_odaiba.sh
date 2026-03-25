#!/bin/bash
set -e
umask 000

CONFIG="/app/examples/scenarios/standalone_odaiba_nade.yaml"

echo "=========================================="
echo " TeraSim Odaiba Standalone NADE Simulation"
echo "  Config: ${CONFIG}"
echo "  Mode: SUMO only (no CARLA)"
echo "=========================================="

echo "[1/1] Running NADE simulation directly..."
python3 /app/scripts/run_experiments_debug.py --config "${CONFIG}"

echo "=========================================="
echo " Simulation complete. Container exiting."
echo "=========================================="
