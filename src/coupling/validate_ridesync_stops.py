import os
import sys
import csv
import argparse
import logging
from typing import Set
from pathlib import Path

import pandas as pd

# Ensure 'src' package is importable when running this script directly
_this_file = Path(__file__).resolve()
_fleetpy_root = _this_file.parents[2]  # .../FleetPy
if str(_fleetpy_root) not in sys.path:
    sys.path.append(str(_fleetpy_root))

from src.coupling.misc import create_fleetpy_network_from_matsim


LOG = logging.getLogger(__name__)


def _load_stop_nodes(all_stops_csv: str) -> Set[int]:
    try:
        df = pd.read_csv(all_stops_csv, delimiter=';')
    except Exception:
        df = pd.read_csv(all_stops_csv)
    if 'node_index' not in df.columns:
        raise RuntimeError(f"CSV {all_stops_csv} missing required column 'node_index'")
    return set(int(x) for x in df['node_index'].tolist() if pd.notna(x))


def validate(all_stops_csv: str, matsim_network_path: str, fleetpy_data_path: str, network_name: str) -> int:
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    LOG.info(f"Validating RideSync stops against network...\n  stops={all_stops_csv}\n  matsim_network={matsim_network_path}\n  data_dir={fleetpy_data_path}\n  network_name={network_name}")

    stop_nodes = _load_stop_nodes(all_stops_csv)

    matsim_edge_to_fp_edge, fp_edge_to_matsim_edge = create_fleetpy_network_from_matsim(
        matsim_network_path, fleetpy_data_path, network_name
    )

    outgoing_nodes = set(fp_edge_to_matsim_edge.keys())
    incoming_nodes = set()
    for _from, to_map in fp_edge_to_matsim_edge.items():
        try:
            incoming_nodes.update(to_map.keys())
        except Exception:
            pass

    missing = sorted([n for n in stop_nodes if (n not in outgoing_nodes and n not in incoming_nodes)])
    isolated = []
    for n in stop_nodes:
        has_out = n in outgoing_nodes and bool(fp_edge_to_matsim_edge.get(n))
        has_in = any((n in to_map) for to_map in fp_edge_to_matsim_edge.values())
        if not has_out and not has_in:
            isolated.append(n)

    LOG.info(f"RideSync stops: total={len(stop_nodes)} missing={len(missing)} isolated={len(isolated)}")
    if missing:
        LOG.warning(f"Missing nodes (not present in network): sample={missing[:20]}")
    if isolated:
        LOG.warning(f"Isolated nodes (no incoming/outgoing edges): sample={isolated[:20]}")

    return 0 if (not missing and not isolated) else 1


def main():
    p = argparse.ArgumentParser(description='Validate RideSync all_stops.csv against MATSim network')
    p.add_argument('--all-stops', required=True, help='Path to all_stops.csv')
    p.add_argument('--network', required=True, help='Path to MATSim network XML(.gz)')
    p.add_argument('--data-dir', required=True, help='FleetPy data directory (will write/read networks/<name>/base)')
    p.add_argument('--network-name', default='matsim_bridge', help='FleetPy network name (default: matsim_bridge)')
    args = p.parse_args()

    rc = validate(args.all_stops, args.network, args.data_dir, args.network_name)
    sys.exit(rc)


if __name__ == '__main__':
    main()


