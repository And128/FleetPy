#!/usr/bin/env python3
import csv
import sys


FREISING_CSV = 'data/ridesync_stops/ridesync_freising/freising_ridesync621_allstops.csv'
OUT_CSV = 'data/ridesync_stops/ridesync_freising/bus_schedule_freising.csv'


def load_stop_ids(path: str):
    name_to_id = {}
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            name_to_id[row['stop_name']] = int(row['stop_id'])
    return name_to_id


def main():
    name_to_id = load_stop_ids(FREISING_CSV)

    fixed_order = [
        'Freising Bahnhof Start',
        'Landratsamt',
        'Steinpark',
        'Berufsschule',
        'Freising Bahnhof Ende',
    ]
    missing = [n for n in fixed_order if n not in name_to_id]
    if missing:
        print(f"ERROR: Missing fixed stops in source CSV: {missing}")
        sys.exit(1)

    stop_ids = [name_to_id[n] for n in fixed_order]
    base_times = [30, 510, 1170, 1590, 2250]
    headway = 2400  # 40 minutes in seconds
    bus_id = 101

    with open(OUT_CSV, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(["bus_id", "stop_id", "departure_time", "stop_name", "route-id"])
        for route_id in range(101, 116):  # 101..115 inclusive
            offset = (route_id - 101) * headway
            for sid, name, t in zip(stop_ids, fixed_order, base_times):
                writer.writerow([bus_id, sid, t + offset, name, route_id])

    print(f"Wrote {OUT_CSV}")


if __name__ == '__main__':
    main()


