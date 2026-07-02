"""3-cosim CARLA client (passive mode) for TeraSim.

Connects to the psim's CARLA server where the autoware_carla_interface bridge owns
world.tick(). This client does NOT tick the world; it follows the psim tick via
world.wait_for_tick() and only mirrors TeraSim/SUMO background vehicles into CARLA.
The psim ego (role_name="ego_vehicle") is protected from cleanup.

Difference from carla_cosim_main.py:
  - passive_tick=True : never call world.tick(); wait for the psim tick instead
  - no world settings applied here (psim owns synchronous_mode / fixed_delta_seconds)
  - map_name empty : do NOT reload the world (psim already loaded the map)
  - skip_tls / control_av off by default : one-way TeraSim -> CARLA, background vehicles only
  - protected_roles includes "ego_vehicle" so the psim ego is never destroyed

Prereqs:
  - psim running (CARLA + autoware_carla_interface bridge) on --carla_port; sync mode owned by psim
  - TeraSim service running (Redis + `python -m terasim_service`) on --terasim_port

Usage:
  python carla_cosim_3cosim.py \
      --terasim_config examples/scenarios/cosim_town01.yaml \
      --carla_port 2013
"""
import argparse

from terasim_service.utils.carla import CarlaCosim


def main():
    p = argparse.ArgumentParser(description="3-cosim CARLA passive client for TeraSim")
    p.add_argument('--carla_host', default='127.0.0.1')
    p.add_argument('--carla_port', default=2013, type=int,
                   help='psim CARLA RPC port (default 2013)')
    p.add_argument('--carla_timeout', default=600.0, type=float,
                   help='CARLA client timeout; large to tolerate procedurally-generated worlds where get_map().to_opendrive() may take time')
    p.add_argument('-s', '--step_length', default=0.05, type=float)
    p.add_argument('--map_name', default='', type=str,
                   help='leave empty: psim already loaded the world; do NOT reload')
    p.add_argument('--terasim_host', default='localhost')
    p.add_argument('--terasim_port', default=8000, type=int)
    p.add_argument('--direct_addr', default=None,
                   help='host:port of a terasim_service.run_direct gRPC server. When set, talk '
                        'to TeraSim directly (no Redis/FastAPI); terasim_host/port are unused')
    p.add_argument('--terasim_config', required=True,
                   help='TeraSim scenario yaml (e.g. examples/scenarios/cosim_town01.yaml)')
    p.add_argument('--control_av', action='store_true',
                   help='also sync CARLA ego -> SUMO (off by default; one-way TeraSim->CARLA)')
    p.add_argument('--sync_tls', action='store_true',
                   help='also sync SUMO traffic lights -> CARLA (off by default)')
    p.add_argument('--protected_roles', nargs='+', default=['AV', 'ego_vehicle'],
                   help='CARLA role_names never destroyed by cleanup (psim ego = ego_vehicle)')
    p.add_argument('--av_carla_role', default='ego_vehicle',
                   help='CARLA role_name whose pose is fed back to the SUMO AV so SUMO traffic '
                        'avoids it (psim ego = ego_vehicle). Defaults to "AV" reproduces the '
                        'original single-AV behavior.')
    args = p.parse_args()

    # 3-cosim passive mode: fixed flags consumed by CarlaCosim.tick()/_cleanup_actors()/close().
    args.passive_tick = True       # follow psim's world.tick() via wait_for_tick(); never tick here
    args.async_mode = False        # use the sync branch (which now carries the passive path)
    args.skip_tls = not args.sync_tls
    # 3-cosim: feed the Autoware ego (role given by --av_carla_role) pose back to the SUMO AV so
    # SUMO background traffic sees the ego and avoids it. Without this, SUMO does not know the ego
    # exists and background vehicles drive through / shove the (physics-on) ego out of the road.
    args.control_av = True

    carla_cosim = CarlaCosim(args)
    # IMPORTANT: do NOT apply world settings here. The psim bridge owns synchronous_mode and
    # fixed_delta_seconds; touching them would break the psim sync loop.

    try:
        tick_flag = True
        while tick_flag:
            tick_flag = carla_cosim.tick()
    except KeyboardInterrupt:
        print("Cancelled by user.")
    finally:
        print("Cleaning up (passive: keep ego, do not reset CARLA settings).")
        carla_cosim.close()


if __name__ == "__main__":
    main()
