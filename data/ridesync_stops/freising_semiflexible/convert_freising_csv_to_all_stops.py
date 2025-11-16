#!/usr/bin/env python3
import sys
import csv
import math
from typing import List, Tuple

try:
    from pyproj import Transformer, CRS  # type: ignore
except Exception:
    Transformer = None  # type: ignore
    CRS = None  # type: ignore


def to_epsg_32632(lons: List[float], lats: List[float]) -> Tuple[List[float], List[float]]:
    if Transformer is not None and CRS is not None:
        transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32632), always_xy=True)
        xs, ys = [], []
        for lon, lat in zip(lons, lats):
            x, y = transformer.transform(lon, lat)
            xs.append(x)
            ys.append(y)
        return xs, ys
    # Fallback simplified UTM-32 approximation
    xs, ys = [], []
    a = 6378137.0
    e2 = 0.00669437999014
    k0 = 0.9996
    central_meridian = math.radians(9.0)
    for lon, lat in zip(lons, lats):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
        T = math.tan(lat_rad) ** 2
        C = e2 * math.cos(lat_rad) ** 2 / (1 - e2)
        A = math.cos(lat_rad) * (lon_rad - central_meridian)
        x = k0 * N * (A + (1 - T + C) * A ** 3 / 6 + (5 - 18 * T + T ** 2 + 72 * C - 58 * e2) * A ** 5 / 120)
        x += 500000
        M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * lat_rad
                 - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * lat_rad)
                 + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * lat_rad)
                 - (35 * e2 ** 3 / 3072) * math.sin(6 * lat_rad))
        y = k0 * (M + N * math.tan(lat_rad) * (A ** 2 / 2 + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
                                               + (61 - 58 * T + T ** 2 + 600 * C - 330 * e2) * A ** 6 / 720))
        xs.append(x)
        ys.append(y)
    return xs, ys


def sniff_delimiter_and_encoding(path: str) -> Tuple[str, str]:
    encodings = ['utf-8', 'cp1252', 'iso-8859-1']
    for enc in encodings:
        try:
            with open(path, 'r', encoding=enc, errors='strict') as f:
                sample = f.read(4096)
            sniffer = csv.Sniffer()
            dialect = sniffer.sniff(sample, delimiters=",;\t")
            return dialect.delimiter, enc
        except Exception:
            continue
    # Fallback
    return ';', 'cp1252'


def main():
    if len(sys.argv) < 3:
        print("Usage: convert_freising_csv_to_all_stops.py <input_csv> <output_csv>")
        sys.exit(1)
    in_csv = sys.argv[1]
    out_csv = sys.argv[2]

    delimiter_in, encoding_in = sniff_delimiter_and_encoding(in_csv)

    rows_in = []
    with open(in_csv, 'r', encoding=encoding_in, errors='replace') as f:
        reader = csv.DictReader(f, delimiter=delimiter_in)
        for r in reader:
            rows_in.append(r)

    lons, lats = [], []
    for r in rows_in:
        lats.append(float(r['lat']))
        lons.append(float(r['long']))

    xs, ys = to_epsg_32632(lons, lats)

    fieldnames = [
        "stop_id","stop_name","lat","long","fixed_stop","stop_order","pos_x","pos_y",
        "utm_zone_number","utm_zone_letter","node_index","distance_to_node"
    ]
    with open(out_csv, 'w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
        writer.writeheader()
        for idx, r in enumerate(rows_in):
            try:
                sid = int(r['stop_id'])
                stop_order = sid
            except Exception:
                sid = idx + 1
                stop_order = idx + 1
            fixed_raw = str(r.get('fixed_stop', 'False')).strip()
            fixed_norm = 'True' if fixed_raw.lower() in ('true','1','yes') else 'False'
            writer.writerow({
                'stop_id': sid,
                'stop_name': r['stop_name'],
                'lat': float(r['lat']),
                'long': float(r['long']),
                'fixed_stop': fixed_norm,
                'stop_order': stop_order,
                'pos_x': xs[idx],
                'pos_y': ys[idx],
                'utm_zone_number': 32,
                'utm_zone_letter': 'U',
                'node_index': -1,
                'distance_to_node': -1,
            })

    print(f"Wrote {len(rows_in)} rows to {out_csv}")


if __name__ == '__main__':
    main()


