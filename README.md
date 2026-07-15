<div align="center">
<p align="center">

<img src="docs/figure/logo.png" height="100px">

</p>
</div>

<p align="center">
<strong>TeraSim (tier4 fork) — naturalistic & adversarial traffic simulation with a single-process CARLA co-simulation link</strong>
</p>

---

## Overview

This is the [tier4](https://github.com/tier4/TeraSim) fork of [mcity/TeraSim](https://github.com/mcity/TeraSim), reduced to the minimum needed for **3-way co-simulation (Autoware × CARLA × TeraSim)**:

- **NDE/NADE traffic simulation** on SUMO: naturalistic background traffic with adversarial maneuvers (`terasim`, `terasim-nde-nade`)
- **A single-process CARLA co-simulation link** (`terasim-service`): the TeraSim simulation loop and the CARLA-facing client run as two threads of one process, exchanging states and commands as plain Python objects. TeraSim background traffic is mirrored into a running CARLA server; an externally driven ego (e.g. Autoware via `autoware_carla_interface`) is fed back into SUMO so background traffic reacts to it.
- **FCD visualization** (`terasim-vis` + `scripts/visualize_fcd.py`) and an **OpenDRIVE→SUMO map conversion pipeline** (`scripts/xodr_to_sumo_converter.py`, sample maps in `examples/xodr_sumo_maps/`)

The upstream extras (TeraSim-World/Cosmos generative sensor simulation, environment generation, dataset tooling, the Redis/FastAPI service and its clients) have been removed from this fork; see the upstream repository for those.

## Repository Layout

```
TeraSim/
├── packages/
│   ├── terasim/            # Core simulation engine (SUMO integration, agents, pipeline)
│   ├── terasim-nde-nade/   # Naturalistic & adversarial environment algorithms (Cython)
│   ├── terasim-service/    # Single-process CARLA co-simulation link
│   └── terasim-vis/        # Visualization tools (FCD plots/videos)
├── examples/
│   ├── maps/               # SUMO nets used by the example scenarios (Town01, ...)
│   ├── scenarios/          # Scenario YAMLs (cosim_town01.yaml, ...)
│   ├── scripts/            # Co-simulation launcher script
│   └── xodr_sumo_maps/     # OpenDRIVE→SUMO converter test maps
├── configs/visulation/     # visualize_fcd.py config example
├── scripts/                # Runners and tooling (see below)
├── docs/                   # OpenDRIVE / SUMO plain-XML format notes
└── tests/                  # Test suites (core, NDE-NADE)
```

## Installation

### Native (conda)

```bash
git clone https://github.com/tier4/TeraSim.git
cd TeraSim
conda create -n terasim python=3.10 -y
conda activate terasim
./setup_environment.sh
```

### Docker (co-simulation image)

```bash
docker build -f Dockerfile.cosim -t terasim-service:latest .
```

**Requirements**: Python 3.10–3.12, SUMO 1.23.1 (installed by the setup script), gcc/g++ (Cython extensions).

## Running

```bash
# CARLA co-simulation, single process (a CARLA server must be running)
python -m terasim_service.run_cosim \
    --config examples/scenarios/cosim_town01.yaml --carla_port 2013

# Standalone NADE run (no CARLA), GUI controlled by the scenario yaml
python scripts/run_experiments_debug.py --config <scenario yaml>

# FCD trajectory visualization (fcd_all output -> plots/video)
python scripts/visualize_fcd.py configs/visulation/example.yaml

# OpenDRIVE -> SUMO net conversion
python scripts/xodr_to_sumo_converter.py --help
```

For the full 3-way setup (Autoware + CARLA + TeraSim in passive mode) see
`docker-compose.cosim-odaiba-3cosim-inprocess.yml` and
`examples/scripts/run_3cosim_odaiba_inprocess.sh`.

## Publications

TeraSim builds on the following research:

* **NDE** – Learning naturalistic driving environment with statistical realism
  [Paper](https://doi.org/10.1038/s41467-023-37677-5) | [Code](https://github.com/michigan-traffic-lab/Learning-Naturalistic-Driving-Environment)

* **NADE** – Intelligent driving intelligence test with naturalistic and adversarial environment
  [Paper](https://doi.org/10.1038/s41467-021-21007-8) | [Code](https://github.com/michigan-traffic-lab/Naturalistic-and-Adversarial-Driving-Environment)

* **D2RL** – Dense deep reinforcement learning for AV safety validation
  [Paper](https://doi.org/10.1038/s41586-023-05732-2) | [Code](https://github.com/michigan-traffic-lab/Dense-Deep-Reinforcement-Learning)

## **📄 License**

- **TeraSim Core and other packages**: Apache 2.0 License
- **Visualization Tools**: MIT License

This project includes modified code from [SumoNetVis](https://github.com/patmalcolm91/SumoNetVis) licensed under the MIT License.
