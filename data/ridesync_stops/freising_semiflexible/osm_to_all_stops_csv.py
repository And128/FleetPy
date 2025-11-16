#!/usr/bin/env python3
import sys
import json
import csv
from typing import Dict, Any, List

try:
    from pyproj import Transformer, CRS
except ImportError:
    Transformer = None


def to_epsg_32632(coords: List[List[float]]):
    if Transformer is None:
        raise RuntimeError("pyproj is required: pip install pyproj")
    transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32632), always_xy=True)
    xs, ys = [], []
    for lon, lat in coords:
        x, y = transformer.transform(lon, lat)
        xs.append(x)
        ys.append(y)
    return xs, ys


def parse_elements(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    elements = data.get("elements", [])
    rows: List[Dict[str, Any]] = []
    # Collect candidate points (lon,lat) and names
    pts: List[List[float]] = []
    names: List[str] = []
    for el in elements:
        if el.get("type") == "node":
            lon = el.get("lon")
            lat = el.get("lat")
            if lon is None or lat is None:
                continue
            name = el.get("tags", {}).get("name") or el.get("tags", {}).get("ref") or "bus_stop"
            pts.append([lon, lat])
            names.append(name)
        elif el.get("type") in ("way", "relation"):
            center = el.get("center")
            if not center:
                continue
            lon = center.get("lon")
            lat = center.get("lat")
            if lon is None or lat is None:
                continue
            name = el.get("tags", {}).get("name") or el.get("tags", {}).get("ref") or "platform"
            pts.append([lon, lat])
            names.append(name)

    xs, ys = to_epsg_32632(pts) if pts else ([], [])
    for idx, (name, lonlat) in enumerate(zip(names, pts)):
        x = xs[idx]
        y = ys[idx]
        rows.append({
            "stop_id": idx + 1,
            "stop_name": name,
            "lat": lonlat[1],
            "long": lonlat[0],
            "fixed_stop": "False",
            "stop_order": idx + 1,
            "pos_x": x,
            "pos_y": y,
            "utm_zone_number": 32,
            "utm_zone_letter": "U",
            "node_index": -1,
            "distance_to_node": -1,
        })
    return rows


def write_all_stops_like(rows: List[Dict[str, Any]], out_csv: str):
    fieldnames = [
        "stop_id","stop_name","lat","long","fixed_stop","stop_order","pos_x","pos_y",
        "utm_zone_number","utm_zone_letter","node_index","distance_to_node"
    ]
    with open(out_csv, "w", newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def main():
    if len(sys.argv) < 3:
        print("Usage: osm_to_all_stops_csv.py <overpass_json_file> <output_csv>")
        sys.exit(1)
    in_json = sys.argv[1]
    out_csv = sys.argv[2]
    with open(in_json, "r", encoding="utf-8") as f:
        data = json.load(f)
    rows = parse_elements(data)
    write_all_stops_like(rows, out_csv)
    print(f"Wrote {len(rows)} stops to {out_csv}")


if __name__ == "__main__":
    main()


