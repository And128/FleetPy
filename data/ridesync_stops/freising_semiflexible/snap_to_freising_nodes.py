#!/usr/bin/env python3
import os
import sys
import csv
import math
from typing import List, Tuple

import pandas as pd
import numpy as np


def load_nodes(nodes_path: str) -> pd.DataFrame:
    if not os.path.exists(nodes_path):
        raise FileNotFoundError(f"Cannot find nodes.csv at {nodes_path}")
    df = pd.read_csv(nodes_path)
    # expected columns: node_index,pos_x,pos_y,(optional)is_stop_only
    if 'node_index' not in df.columns or 'pos_x' not in df.columns or 'pos_y' not in df.columns:
        raise RuntimeError("nodes.csv must have columns: node_index,pos_x,pos_y")
    return df


def snap_to_nodes(stops_df: pd.DataFrame, nodes_df: pd.DataFrame) -> Tuple[List[int], List[float]]:
    node_x = nodes_df['pos_x'].to_numpy()
    node_y = nodes_df['pos_y'].to_numpy()
    node_idx = nodes_df['node_index'].to_numpy()

    idxs: List[int] = []
    dists: List[float] = []

    for _, r in stops_df.iterrows():
        x = float(r['pos_x'])
        y = float(r['pos_y'])
        dx = node_x - x
        dy = node_y - y
        dist2 = dx * dx + dy * dy
        i = int(np.argmin(dist2))
        idxs.append(int(node_idx[i]))
        dists.append(float(math.sqrt(dist2[i])))
    return idxs, dists


def main():
    if len(sys.argv) < 3:
        print("Usage: snap_to_freising_nodes.py <freising_all_stops.csv> <output_csv>")
        sys.exit(1)

    in_csv = sys.argv[1]
    out_csv = sys.argv[2]

    network_nodes = os.path.join("data", "networks", "freising_network", "base", "nodes.csv")

    stops_df = pd.read_csv(in_csv, sep=';')
    nodes_df = load_nodes(network_nodes)

    node_indices, distances = snap_to_nodes(stops_df, nodes_df)

    stops_df['node_index'] = node_indices
    stops_df['distance_to_node'] = distances

    # enforce sequential stop_order starting from 1
    stops_df = stops_df.reset_index(drop=True)
    stops_df['stop_order'] = stops_df.index + 1

    stops_df.to_csv(out_csv, sep=';', index=False)
    print(f"Wrote snapped stops to {out_csv}")


if __name__ == '__main__':
    main()
