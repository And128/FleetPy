import csv
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional


@dataclass
class RideSyncStop:
    stop_id: int
    stop_name: str
    pos_x: float
    pos_y: float
    fixed: bool
    stop_order: int
    node_index: int


@dataclass
class RideSyncFixedDeparture:
    bus_id: str
    stop_id: int
    departure_time: int
    stop_name: str
    route_id: int


class RideSyncData:
    """Loader and accessor for RideSync semi-flex route data.

    Reads all_stops.csv and bus_schedule.csv and provides convenient indices:
    - stops_by_id
    - fixed_departures_by_route: route_id -> ordered list of fixed departures
    - fixed_stop_ids_by_route: route_id -> ordered list of fixed stop_ids (ascending by stop_order)
    - stop_order_by_id: stop_id -> stop_order
    """

    def __init__(self, all_stops_csv: str, bus_schedule_csv: str):
        self.all_stops_csv = all_stops_csv
        self.bus_schedule_csv = bus_schedule_csv

        self.stops_by_id: Dict[int, RideSyncStop] = {}
        self.node_to_stop_id: Dict[int, int] = {}
        self.stop_order_by_id: Dict[int, int] = {}
        self.fixed_departures_by_route: Dict[int, List[RideSyncFixedDeparture]] = {}
        self.fixed_stop_ids_by_route: Dict[int, List[int]] = {}
        self._dep_lookup: Dict[Tuple[int, int], int] = {}
        self._route_stop_dep: Dict[int, Dict[int, int]] = {}

        self._load_all_stops()
        self._load_bus_schedule()

    def _load_all_stops(self):
        with open(self.all_stops_csv, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f, delimiter=';')
            for row in reader:
                stop_id = int(row['stop_id'])
                stop = RideSyncStop(
                    stop_id=stop_id,
                    stop_name=row['stop_name'],
                    pos_x=float(row['pos_x']),
                    pos_y=float(row['pos_y']),
                    fixed=row['fixed_stop'].strip().lower() in ('true', '1', 'yes'),
                    stop_order=int(row['stop_order']),
                    node_index=int(row['node_index'])
                )
                self.stops_by_id[stop_id] = stop
                self.stop_order_by_id[stop_id] = stop.stop_order
                self.node_to_stop_id[stop.node_index] = stop_id

    def _load_bus_schedule(self):
        with open(self.bus_schedule_csv, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                route_id = int(row['route-id'])
                dep = RideSyncFixedDeparture(
                    bus_id=row['bus_id'],
                    stop_id=int(row['stop_id']),
                    departure_time=int(row['departure_time']),
                    stop_name=row['stop_name'],
                    route_id=route_id,
                )
                self.fixed_departures_by_route.setdefault(route_id, []).append(dep)
                self._dep_lookup[(route_id, dep.stop_id)] = dep.departure_time
                try:
                    self._route_stop_dep[route_id][dep.stop_id] = dep.departure_time
                except KeyError:
                    self._route_stop_dep[route_id] = {dep.stop_id: dep.departure_time}
        # sort fixed departures by stop_order per route
        for route_id, deps in self.fixed_departures_by_route.items():
            deps.sort(key=lambda d: self.stop_order_by_id.get(d.stop_id, 1_000_000))
            self.fixed_stop_ids_by_route[route_id] = [d.stop_id for d in deps]

    # Utilities

    def route_bus_id(self, route_id: int) -> str:
        deps = self.fixed_departures_by_route[route_id]
        return deps[0].bus_id

    def get_departure_time(self, route_id: int, fixed_stop_id: int) -> Optional[int]:
        return self._dep_lookup.get((route_id, fixed_stop_id))

    def get_prev_fixed_stop_id(self, pickup_stop_id: int) -> Optional[int]:
        pu_order = self.stop_order_by_id[pickup_stop_id]
        # All fixed stop ids (consistent across routes)
        fixed_ids = set()
        for v in self.fixed_stop_ids_by_route.values():
            fixed_ids.update(v)
        fixed_orders = sorted([(sid, self.stop_order_by_id[sid]) for sid in fixed_ids], key=lambda x: x[1])
        prev = None
        for sid, so in fixed_orders:
            if so < pu_order:
                prev = sid
            else:
                break
        return prev

    def select_route_by_prev_fixed_and_access(self, prev_fixed_stop_id: int, access_time: int) -> Optional[Tuple[int, str, int]]:
        """Return (route_id, bus_id, dep_time) where departure_time(prev_fixed_stop) - 30 > access_time and minimal.
        If none, return None.
        """
        best = None
        best_val = None
        for route_id, stop_dep in self._route_stop_dep.items():
            dep = stop_dep.get(prev_fixed_stop_id)
            if dep is None:
                continue
            key_time = dep - 30
            if key_time > access_time:
                if best_val is None or key_time < best_val:
                    best_val = key_time
                    best = route_id
        if best is None:
            return None
        return best, self.route_bus_id(best), self._route_stop_dep[best][prev_fixed_stop_id]