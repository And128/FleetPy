import logging
from typing import Any, Dict, Tuple, Optional

from src.fleetctrl.FleetControlBase import FleetControlBase
from src.fleetctrl.planning.PlanRequest import PlanRequest
from src.simulation.Offers import TravellerOffer
from src.fleetctrl.pooling.objectives import return_pooling_objective_function
import os
import csv
from src.misc.globals import *
from src.routing.NetworkBase import return_node_position
import json

from .RideSyncData import RideSyncData
from .RideSyncPlanner import RideSyncPlanner

LOG = logging.getLogger(__name__)


INPUT_PARAMETERS_RideSyncSemiFlexibleFleetControl = {
    "doc": "RideSync semi-flexible bus controller integrating fixed-time stops and optional stops",
    "inherit": "FleetControlBase",
    "input_parameters_mandatory": [],
    "input_parameters_optional": [],
    "mandatory_modules": [],
    "optional_modules": []
}


class RideSyncSemiFlexibleFleetControl(FleetControlBase):
    def __init__(self, op_id: int, operator_attributes: Dict, list_vehicles, routing_engine, zone_system,
                 scenario_parameters: Dict, dir_names: Dict, op_charge_depot_infra=None, list_pub_charging_infra=[]):
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names=dir_names, op_charge_depot_infra=op_charge_depot_infra,
                         list_pub_charging_infra=list_pub_charging_infra)
        
        # Store scenario_parameters for later use
        self.scenario_parameters = scenario_parameters

        # Load RideSync inputs from scenario config
        all_stops = dir_names[G_DIR_DATA] + "/" + scenario_parameters.get("ridesync_all_stops_file")
        bus_sched = dir_names[G_DIR_DATA] + "/" + scenario_parameters.get("ridesync_route_config_file", "ridesync_stops/bus_schedule.csv")
        # Fallback to default path if route_config_file key points elsewhere; user specified bus_schedule explicitly earlier
        if bus_sched.endswith("route_config.json"):
            bus_sched = dir_names[G_DIR_DATA] + "/ridesync_stops/bus_schedule.csv"
        self.rs_data = RideSyncData(all_stops, bus_sched)
        self.planner = RideSyncPlanner(self.rs_data, routing_engine)

        self.sim_time = scenario_parameters[G_SIM_START_TIME]
        self.const_bt = operator_attributes.get(G_OP_CONST_BT, 30)
        # QoS: easy cutoff for waiting time (seconds)
        self.rs_max_wait_cutoff = 3600
        # QoS: cutoff for walking time (seconds) start->pickup and dropoff->end
        self.rs_max_walk_cutoff = 1800
        # temporary assignments awaiting confirmation
        self.tmp_assignment = {}
        # track pending offers to detect declines
        self._pending_offers: Dict[Any, int] = {}
        self._init_dynamic_fleetcontrol_output_key(G_FCTRL_CT_RQU)
        # set objective function for plan utility
        self.vr_ctrl_f = return_pooling_objective_function(operator_attributes[G_OP_VR_CTRL_F])

        # Per-vehicle, per-route_id plans (persist across bookings within that route_id only)
        # vid -> { route_id -> VehiclePlan }
        self.veh_route_plans: Dict[int, Dict[int, Any]] = {veh.vid: {} for veh in self.sim_vehicles}
        # Track which (vid, route_id) combinations have been assigned to avoid re-assigning every time step
        self._assigned_routes: set = set()
        # bus usage recording
        self._bus_usage_current: Dict[int, Dict] = {}
        self._bus_usage_f = os.path.join(dir_names[G_DIR_OUTPUT], "bus_usage.csv")
        self._bus_usage_seen = set()
        self._bus_usage_seen_pickups: Dict[int, set] = {veh.vid: set() for veh in self.sim_vehicles}
        self._bus_usage_seen_dropoffs: Dict[int, set] = {veh.vid: set() for veh in self.sim_vehicles}
        # MATSim feed logger
        self._matsim_feed_path = os.path.join(dir_names[G_DIR_OUTPUT], "matsim_feed.jsonl")
        self._matsim_bus_id_cache: Dict[int, str] = {}
        if not os.path.isfile(self._bus_usage_f):
            with open(self._bus_usage_f, "w", newline='', encoding='utf-8') as fh:
                w = csv.writer(fh)
                w.writerow(["iteration", "bus_id", "route_id", "stop_id", "arrival_time", "departure_time", "rq_id_pick_up", "rq_id_drop_off"])
        # final route vehicle plan snapshot per route-id
        self._vehplan_f = os.path.join(dir_names[G_DIR_OUTPUT], "vehicle_plans.csv")
        self._vp_written_routes = set()  # (vid, route_id)
        if not os.path.isfile(self._vehplan_f):
            with open(self._vehplan_f, "w", newline='', encoding='utf-8') as fh:
                w = csv.writer(fh)
                w.writerow(["iteration", "bus_id", "vehicle_id", "route_id", "seq", "stop_id", "node_id", "fixed_stop", "planned_arrival", "planned_departure", "rq_ids_pick", "rq_ids_drop"]) 
        # cache minimal offer metadata for recovery on late confirm
        self._offer_cache: Dict[Any, Dict] = {}

    def _matsim_emit(self, event: Dict):
        try:
            with open(self._matsim_feed_path, "a", encoding='utf-8') as fh:
                fh.write(json.dumps(event, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def _matsim_plan_update(self, bus_id: str, route_id: int, route_plan):
        try:
            seq_list = []
            seq = 0
            for ps in route_plan.list_plan_stops:
                node_id = ps.get_pos()[0]
                stop_id = self.rs_data.node_to_stop_id.get(node_id)
                if stop_id is None:
                    continue
                fixed = bool(ps.is_fixed_stop()) if hasattr(ps, 'is_fixed_stop') else False
                arr, dep = ps.get_planned_arrival_and_departure_time()
                bd = getattr(ps, 'boarding_dict', {}) or {}
                seq_list.append({
                    "seq": seq,
                    "stop_id": int(stop_id),
                    "fixed_stop": int(fixed),
                    "planned_arrival": int(arr) if arr is not None else None,
                    "planned_departure": int(dep) if dep is not None else None,
                    "pick_rids": list(map(int, bd.get(1, []))) if bd.get(1) else [],
                    "drop_rids": list(map(int, bd.get(-1, []))) if bd.get(-1) else [],
                })
                seq += 1
            self._matsim_emit({
                "type": "plan_update",
                "ts": int(self.sim_time),
                "bus_id": bus_id,
                "route_id": int(route_id),
                "sequence": seq_list,
            })
        except Exception:
            pass

    # Emit MATSim rejection event and delegate to base implementation
    def _create_rejection(self, prq: PlanRequest, simulation_time: int):
        try:
            self._matsim_emit({
                "type": "offer_rejected",
                "ts": int(simulation_time),
                "rid": int(prq.get_rid()) if hasattr(prq, 'get_rid') else None,
                "route_id": None,
            })
        except Exception:
            pass
        return super()._create_rejection(prq, simulation_time)

    def _ensure_route_plan_exists(self, vid: int, sim_time: int, route_id: int):
        plans_for_vehicle = self.veh_route_plans.setdefault(vid, {})
        if route_id in plans_for_vehicle:
            return
        veh_obj = self.sim_vehicles[vid]
        base_plan = self.planner.build_base_plan(veh_obj, sim_time, route_id)
        plans_for_vehicle[route_id] = base_plan

    def _active_route_id(self, sim_time: int) -> Optional[int]:
        # active route where sim_time is within [start_dep, last_dep)
        active = None
        for route_id, deps in self.rs_data.fixed_departures_by_route.items():
            start_dep = deps[0].departure_time
            end_dep = deps[-1].departure_time  # last fixed stop departure of this route
            # include the last fixed stop departure instant to avoid None at exact boundary
            if start_dep <= sim_time <= end_dep:
                active = route_id
                break
        return active

    def _compute_access_time(self, rq: Any, pickup_stop_id: int) -> int:
        # Walking time from origin node to pickup_stop node; assume 1.34 m/s standard walking speed
        start_node = rq.get_origin_pos()[0]
        pu_node = self.rs_data.stops_by_id[pickup_stop_id].node_index
        o_pos = return_node_position(start_node)
        d_pos = return_node_position(pu_node)
        _, tt, dist = self.routing_engine.return_travel_costs_1to1(o_pos, d_pos)
        # Use distance-based walking at 1.34 m/s if routing returns street travel; ensures independence of congestion
        walk_time = int(round(dist / 1.34))
        return rq.rq_time + walk_time

    def _choose_stops_for_request(self, rq: Any) -> Tuple[Optional[int], Optional[int], float, float]:
        # choose closest pickup and dropoff stops with pickup_id < dropoff_id, else next best
        sx, sy = self.routing_engine.return_node_coordinates(rq.o_pos[0])
        dx, dy = self.routing_engine.return_node_coordinates(rq.d_pos[0])

        # find best under id-ordering constraint with 2500m radius
        best_pu = None
        best_do = None
        best_pu_dist = float('inf')
        best_do_dist = float('inf')

        # Start with closest pickup, then closest dropoff with higher order
        # Naive scan sufficient for small stop set
        for pu in self.rs_data.stops_by_id.values():
            pu_dx = pu.pos_x - sx
            pu_dy = pu.pos_y - sy
            pu_dist = (pu_dx * pu_dx + pu_dy * pu_dy) ** 0.5
            if pu_dist > 2500:
                continue
            for do in self.rs_data.stops_by_id.values():
                if int(do.stop_id) <= int(pu.stop_id):
                    continue
                do_dx = do.pos_x - dx
                do_dy = do.pos_y - dy
                do_dist = (do_dx * do_dx + do_dy * do_dy) ** 0.5
                if do_dist > 2500:
                    continue
                # prioritize closer pu then closer do
                if pu_dist < best_pu_dist or (pu_dist == best_pu_dist and do_dist < best_do_dist):
                    best_pu = pu.stop_id
                    best_do = do.stop_id
                    best_pu_dist = pu_dist
                    best_do_dist = do_dist
        return best_pu, best_do, best_pu_dist, best_do_dist

    def user_request(self, rq: Any, sim_time: int):
        LOG.debug(f"[RideSync] incoming rq={rq.get_rid_struct()} t={sim_time} o={rq.o_pos[0]} d={rq.d_pos[0]}")
        is_matsim = self.scenario_parameters.get("rq_type") == "SlaveRequest"
        LOG.info(f"[RideSync] Processing request {rq.get_rid_struct()} - MATSim mode: {is_matsim}")
        self.sim_time = sim_time

        # 1) determine candidate stops
        pu_stop, do_stop, pu_dist, do_dist = self._choose_stops_for_request(rq)
        if pu_stop is None or do_stop is None:
            # reject
            prq = PlanRequest(rq, self.routing_engine, boarding_time=self.const_bt)
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} no feasible pu/do within 2500m and id constraint")
            # MUST add to rq_dict before creating rejection so offer can be retrieved later
            self.rq_dict[rq.rid] = prq
            self._create_rejection(prq, sim_time)
            return

        # 2) select route by prev fixed stop and access_time (more permissive than only active)
        access_time = self._compute_access_time(rq, pu_stop)
        prev_fixed_sid = self.rs_data.get_prev_fixed_stop_id(pu_stop)
        sel = None
        if prev_fixed_sid is not None:
            sel = self.rs_data.select_route_by_prev_fixed_and_access(prev_fixed_sid, access_time)
        if sel is None:
            prq = PlanRequest(rq, self.routing_engine, boarding_time=self.const_bt)
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} no feasible route for prev_fixed={prev_fixed_sid} access={access_time}")
            # MUST add to rq_dict before creating rejection so offer can be retrieved later
            self.rq_dict[rq.rid] = prq
            self._create_rejection(prq, sim_time)
            return
        route_id, sel_bus_id, prev_dep = sel

        # 3) ensure a persistent plan for this route_id (do not assign to vehicle unless route is active and plan not assigned yet)
        # Select a valid vehicle deterministically by highest vid (robust if vids start at 1)
        try:
            veh_obj = max(self.sim_vehicles, key=lambda v: v.vid)
        except Exception:
            veh_obj = self.sim_vehicles[0]
        vid = veh_obj.vid
        self._ensure_route_plan_exists(vid, sim_time, route_id)
        base_plan = self.veh_route_plans[vid][route_id]

        # 4) try to insert pickup/dropoff (optional stops inserted, fixed stops attached)
        # For non-active routes, anchor evaluation at the route's first fixed stop (arrival = dep-30)
        anchor = (self._active_route_id(sim_time) != route_id)
        matsim_coupling = self.scenario_parameters.get("sim_env") == "MobiTopp" or self.scenario_parameters.get("rq_type") == "SlaveRequest"
        new_plan = self.planner.insert_optional_pair(veh_obj, sim_time, base_plan, route_id, pu_stop, do_stop, rq.get_rid_struct(), getattr(rq, 'nr_pax', 1), anchor_to_route_start=anchor, matsim_coupling=matsim_coupling)
        # If infeasible, expand search: try up to 3 pickup and 3 dropoff candidates, then planner fallback
        if new_plan is None:
            pu_id_orig = int(pu_stop)
            do_id_orig = int(do_stop)
            # Coordinates for distance checks
            sx, sy = self.routing_engine.return_node_coordinates(rq.o_pos[0])
            dx, dy = self.routing_engine.return_node_coordinates(rq.d_pos[0])

            # Build pickup candidates (<=3) with stop_id < original dropoff_id and within 2500m
            pu_candidates = []
            for sid, st in self.rs_data.stops_by_id.items():
                if int(sid) >= do_id_orig:
                    continue
                pu_dx = st.pos_x - sx
                pu_dy = st.pos_y - sy
                pu_dist_c = (pu_dx * pu_dx + pu_dy * pu_dy) ** 0.5
                if pu_dist_c <= 2500:
                    pu_candidates.append((pu_dist_c, sid))
            pu_candidates.sort(key=lambda x: (x[0], x[1]))
            pu_candidates = pu_candidates[:3]

            # Build dropoff candidates (<=3) with stop_id > original pickup_id and within 2500m
            do_candidates = []
            for dsid, dst in self.rs_data.stops_by_id.items():
                if int(dsid) <= pu_id_orig:
                    continue
                do_dx = dst.pos_x - dx
                do_dy = dst.pos_y - dy
                do_dist_c = (do_dx * do_dx + do_dy * do_dy) ** 0.5
                if do_dist_c <= 2500:
                    do_candidates.append((do_dist_c, dsid))
            do_candidates.sort(key=lambda x: (x[0], x[1]))
            do_candidates = do_candidates[:3]

            found = False
            for _, pu_sid in pu_candidates:
                for _, do_sid in do_candidates:
                    if int(do_sid) <= int(pu_sid):
                        continue
                    tmp = self.planner.insert_optional_pair(veh_obj, sim_time, base_plan, route_id, pu_sid, do_sid, rq.get_rid_struct(), getattr(rq, 'nr_pax', 1), allow_fallback=False, anchor_to_route_start=anchor, matsim_coupling=matsim_coupling)
                    if tmp is not None:
                        LOG.debug(f"[RideSync] chosen candidate pu={pu_sid} do={do_sid} for rid={rq.get_rid_struct()}")
                        new_plan = tmp
                        pu_stop = pu_sid
                        do_stop = do_sid
                        found = True
                        break
                if found:
                    break

            # If still none, invoke planner fallback once using the original pair
            if new_plan is None and not found:
                new_plan = self.planner.insert_optional_pair(veh_obj, sim_time, base_plan, route_id, pu_stop, do_stop, rq.get_rid_struct(), getattr(rq, 'nr_pax', 1), allow_fallback=True, anchor_to_route_start=anchor, matsim_coupling=matsim_coupling)
        # If we have a feasible plan (from any path), align pu/do to actual plan content
        if new_plan is not None:
            try:
                rid_key = rq.get_rid_struct()
                pu_node = None
                do_node = None
                for ps in new_plan.list_plan_stops:
                    bd = getattr(ps, 'boarding_dict', {}) or {}
                    if rid_key in bd.get(1, []):
                        pu_node = ps.get_pos()[0]
                    if rid_key in bd.get(-1, []):
                        do_node = ps.get_pos()[0]
                if pu_node is not None:
                    mapped_pu = self.rs_data.node_to_stop_id.get(pu_node)
                    if mapped_pu is not None:
                        pu_stop = mapped_pu
                if do_node is not None:
                    mapped_do = self.rs_data.node_to_stop_id.get(do_node)
                    if mapped_do is not None:
                        do_stop = mapped_do
            except Exception:
                pass
        # compute walking times for QoS transparency
        o_pos = return_node_position(rq.o_pos[0])
        d_pos = return_node_position(rq.d_pos[0])
        pu_node = self.rs_data.stops_by_id[pu_stop].node_index
        do_node = self.rs_data.stops_by_id[do_stop].node_index
        pu_pos = return_node_position(pu_node)
        do_pos = return_node_position(do_node)
        _, _, o_pu_dist = self.routing_engine.return_travel_costs_1to1(o_pos, pu_pos)
        _, _, do_d_dist = self.routing_engine.return_travel_costs_1to1(do_pos, d_pos)
        walk_time_start = int(round(o_pu_dist / 1.34))
        walk_time_end = int(round(do_d_dist / 1.34))

        prq = PlanRequest(
            rq,
            self.routing_engine,
            min_wait_time=0,
            max_wait_time=self.rs_max_wait_cutoff,
            max_detour_time_factor=self.max_dtf,
            max_constant_detour_time=self.max_cdt,
            add_constant_detour_time=self.add_cdt,
            min_detour_time_window=self.min_dtw,
            boarding_time=self.const_bt,
            pickup_pos=pu_pos,
            dropoff_pos=do_pos,
            walking_time_start=walk_time_start,
            walking_time_end=walk_time_end,
        )
        # Enforce walking time cutoff before storing request
        if walk_time_start > self.rs_max_walk_cutoff or walk_time_end > self.rs_max_walk_cutoff:
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} walk_start={walk_time_start} walk_end={walk_time_end} > cutoff={self.rs_max_walk_cutoff}")
            self._create_rejection(prq, sim_time)
            return
        # Store using the original request rid (which matches what broker will use in get_current_offer)
        self.rq_dict[rq.rid] = prq
        if new_plan is None:
            LOG.debug(f"[RideSync] infeasible insertion for rq={prq.get_rid_struct()} pu={pu_stop} do={do_stop} -> reject")
            self._create_rejection(prq, sim_time)
            return

        # Compute offer times from plan
        times = new_plan.pax_info.get(prq.get_rid_struct())
        pu_time, do_time = (None, None)
        if isinstance(times, (list, tuple)):
            if len(times) >= 2:
                pu_time, do_time = times[0], times[1]
            elif len(times) == 1:
                pu_time = times[0]
        # If pax_info is not auto-populated (no boarding_dict entries), approximate using planned arrival at pu/do
        if pu_time is None:
            # find indices by node id match
            pu_node = self.rs_data.stops_by_id[pu_stop].node_index
            do_node = self.rs_data.stops_by_id[do_stop].node_index
            pu_time = next((ps.get_planned_arrival_and_departure_time()[0] for ps in new_plan.list_plan_stops if ps.get_pos()[0] == pu_node), None)
            do_time = next((ps.get_planned_arrival_and_departure_time()[1] for ps in new_plan.list_plan_stops if ps.get_pos()[0] == do_node), None)

        if pu_time is None or do_time is None:
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} pu_time/do_time None after insertion")
            self._create_rejection(prq, sim_time)
            return

        # Enforce MATSim time window from request (earliest/latest pickup)
        try:
            rq_ept = getattr(rq, 'ept', None)
            rq_lpt = getattr(rq, 'lpt', None)
        except Exception:
            rq_ept = None
            rq_lpt = None
        if rq_ept is not None and pu_time < rq_ept:
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} pu_time {int(pu_time)} < EPT {int(rq_ept)}")
            self._create_rejection(prq, sim_time)
            return
        if rq_lpt is not None and pu_time > rq_lpt:
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} pu_time {int(pu_time)} > LPT {int(rq_lpt)}")
            self._create_rejection(prq, sim_time)
            return

        # Enforce easy cutoff: reject only if offered waiting time exceeds 3600s
        if pu_time - rq.rq_time > self.rs_max_wait_cutoff:
            LOG.debug(f"[RideSync] reject rq={rq.get_rid_struct()} wait={pu_time - rq.rq_time} > cutoff={self.rs_max_wait_cutoff}")
            self._create_rejection(prq, sim_time)
            return

        add = {
            'ridesync_bus_id': sel_bus_id,
            'ridesync_route_id': route_id,
            'ridesync_pickup_stop_id': pu_stop,
            'ridesync_dropoff_stop_id': do_stop,
            'ridesync_access_time': access_time,
            'ridesync_pickup_time': int(pu_time),
            'ridesync_dropoff_time': int(do_time),
            'ridesync_walking_time_start': int(walk_time_start),
            'ridesync_walking_time_end': int(walk_time_end),
        }
        offer_wait = pu_time - prq.rq_time
        offer_drive = do_time - pu_time
        LOG.debug(f"[RideSync] offer rq={prq.get_rid_struct()} route={route_id} bus={add['ridesync_bus_id']} pu={pu_stop}@{int(pu_time)} do={do_stop}@{int(do_time)} wait={offer_wait} drive={offer_drive} walk_start={int(walk_time_start)} walk_end={int(walk_time_end)} access={access_time}")
        LOG.info(f"[RideSync] Creating offer for request {prq.get_rid_struct()} - pickup time: {pu_time}, dropoff time: {do_time}")
        offer = TravellerOffer(prq.get_rid_struct(), self.op_id, offer_wait, offer_drive, 0, add)
        prq.set_service_offered(offer)
        # Store temp assignment awaiting confirmation, and persist this plan candidate to the route-specific store
        self.tmp_assignment[rq.rid] = (vid, route_id, new_plan)
        # mark offer as pending to detect declines later
        self._pending_offers[rq.rid] = sim_time
        # cache minimal info to recover assignment on confirm even if tmp_assignment was pruned
        try:
            self._offer_cache[rq.rid] = {
                "vid": vid,
                "route_id": route_id,
                "pu_stop": int(pu_stop),
                "do_stop": int(do_stop),
                "sim_time": int(sim_time),
            }
        except Exception:
            pass

    # --- required abstract methods delegating to base implementations or minimal logic ---
    def receive_status_update(self, vid: int, simulation_time: int, list_finished_VRL, force_update: bool = True):
        super().receive_status_update(vid, simulation_time, list_finished_VRL, force_update=force_update)
        veh_obj = self.sim_vehicles[vid]
        try:
            self.pos_veh_dict[veh_obj.pos].append(veh_obj)
        except KeyError:
            self.pos_veh_dict[veh_obj.pos] = [veh_obj]
        # When a route becomes active, assign any stored plans for that route
        # This is needed for both MATSim and non-MATSim simulations
        # Only assign once per (vid, route_id) to avoid re-assigning every time step
        is_matsim_coupling = self.scenario_parameters.get("sim_env") == "MobiTopp" or self.scenario_parameters.get("rq_type") == "SlaveRequest"
        
        active_rid = self._active_route_id(simulation_time)
        if active_rid is not None and (vid, active_rid) not in self._assigned_routes:
            route_plan = self.veh_route_plans.get(vid, {}).get(active_rid)
            if route_plan is not None:
                # prune stale rids from stored plan to keep sync with rq_dict
                stale = [rid for rid in list(route_plan.pax_info.keys()) if rid not in self.rq_dict]
                for rid in stale:
                    LOG.debug(f"[RideSync] pruning stale rid {rid} from stored route plan before assignment")
                    try:
                        del route_plan.pax_info[rid]
                    except KeyError:
                        pass
                for ps in route_plan.list_plan_stops:
                    try:
                        bd = ps.boarding_dict
                    except Exception:
                        continue
                    for key in (1, -1):
                        if key in bd:
                            bd[key] = [rid for rid in bd[key] if rid in self.rq_dict]
                # drop empty non-fixed stops
                filtered = []
                for ps in route_plan.list_plan_stops:
                    try:
                        is_fixed = ps.is_fixed_stop()
                    except Exception:
                        is_fixed = False
                    bd = getattr(ps, 'boarding_dict', {}) or {}
                    if not is_fixed and len(bd.get(1, [])) == 0 and len(bd.get(-1, [])) == 0 and getattr(ps, 'change_nr_pax', 0) == 0:
                        continue
                    filtered.append(ps)
                route_plan.list_plan_stops = filtered
                # For MATSim coupling: filter to only actionable stops (with boarding/alighting)
                # This ensures FleetPy's plan matches what MATSimSocket sends to MATSim
                if is_matsim_coupling:
                    try:
                        stops_to_keep = []
                        for ps in route_plan.list_plan_stops:
                            bd = getattr(ps, 'boarding_dict', {}) or {}
                            if len(bd.get(1, [])) > 0 or len(bd.get(-1, [])) > 0:
                                stops_to_keep.append(ps)
                        route_plan.list_plan_stops = stops_to_keep
                        LOG.debug(f"[RideSync] Filtered to {len(stops_to_keep)} actionable stops for MATSim coupling")
                    except Exception:
                        pass
                    # DON'T recompute timings for MATSim - they're already correct from schedule-based planning
                    # Recomputing would destroy the original offered times and cause prebooking failures
                else:
                    # For non-MATSim: recompute timings from current vehicle position
                    try:
                        route_plan.update_tt_and_check_plan(veh_obj, simulation_time, self.routing_engine, keep_feasible=True)
                    except Exception:
                        pass
                # avoid overwriting a locked first VRL
                try:
                    current_first_locked = bool(veh_obj.assigned_route and veh_obj.assigned_route[0].locked)
                except Exception:
                    current_first_locked = False
                if not current_first_locked:
                    try:
                        # Always assign the full route plan when the route becomes active, even for MATSim
                        # The prebooking was just to inform MATSim about accepted requests
                        self.assign_vehicle_plan(veh_obj, route_plan, simulation_time, force_assign=is_matsim_coupling)
                        # Mark this (vid, route_id) as assigned so we don't re-assign every time step
                        self._assigned_routes.add((vid, active_rid))
                        LOG.debug(f"[RideSync] Assigned route {active_rid} to vehicle {vid} at {simulation_time}")
                    except AssertionError:
                        LOG.debug(f"[RideSync] skip assign at {simulation_time} due to locked VRL; will retry later")
        # Record bus usage from finished VRLs
        for vrl in list_finished_VRL:
            node = getattr(vrl, 'destination_pos', None)
            if not node:
                continue
            node_id = node[0]
            stop_id = self.rs_data.node_to_stop_id.get(node_id)
            if stop_id is None:
                continue
            # determine route-id for record
            rid_for_record = active_rid
            bus_id = self.rs_data.route_bus_id(rid_for_record) if rid_for_record is not None else self.rs_data.route_bus_id(list(self.rs_data.fixed_departures_by_route.keys())[0])
            # arrival when finishing driving leg
            if vrl.status in G_DRIVING_STATUS:
                prev = self._bus_usage_current.get(vid)
                if prev is not None:
                    # finalize previous optional stop only here; fixed stops will be written on PLANNED_STOP finish
                    if not prev.get('is_fixed'):
                        # resolve planned arrival and departure from the route plan at write-time
                        arr_planned = None
                        dep_planned = None
                        try:
                            route_hint = prev.get('route_id', rid_for_record)
                            rp = self.veh_route_plans.get(vid, {}).get(route_hint)
                            if rp is not None:
                                for ps in rp.list_plan_stops:
                                    if ps.get_pos()[0] == prev['node_id']:
                                        arr_planned, dep_planned = ps.get_planned_arrival_and_departure_time()
                                        break
                            if arr_planned is None or dep_planned is None:
                                # try any stored plan for this vehicle
                                for rid_key, plan in self.veh_route_plans.get(vid, {}).items():
                                    for ps in plan.list_plan_stops:
                                        if ps.get_pos()[0] == prev['node_id']:
                                            arr_planned, dep_planned = ps.get_planned_arrival_and_departure_time()
                                            break
                                    if arr_planned is not None or dep_planned is not None:
                                        break
                        except Exception:
                            pass
                        arr_time = int(arr_planned) if arr_planned is not None else prev.get('arrival', simulation_time)
                        dep_time = int(dep_planned) if dep_planned is not None else prev.get('dep', simulation_time)
                        self._write_bus_usage_row(bus_id, prev.get('route_id', rid_for_record), prev['node_id'], arr_time, dep_time, prev.get('picks', []), prev.get('drops', []))
                is_fixed = bool(self.rs_data.stops_by_id.get(stop_id).fixed)
                # if a fixed stop is reached exactly at its scheduled departure, attribute it to the current active route
                if is_fixed and rid_for_record is None:
                    rid_for_record = active_rid
                # determine planned arrival for this node from the current route plan
                arr_planned = None
                try:
                    rp = self.veh_route_plans.get(vid, {}).get(rid_for_record)
                    if rp is not None:
                        for ps in rp.list_plan_stops:
                            if ps.get_pos()[0] == node_id:
                                arr_planned, _ = ps.get_planned_arrival_and_departure_time()
                                break
                except Exception:
                    pass
                # initialize current stop state with resolved route-id and planned arrival (fallback to event time)
                self._bus_usage_current[vid] = {"node_id": node_id, "arrival": int(arr_planned) if arr_planned is not None else simulation_time, "picks": [], "drops": [], "route_id": rid_for_record, "is_fixed": is_fixed}
                # MATSim arrive event
                try:
                    bus_id_emit = self.rs_data.route_bus_id(rid_for_record) if rid_for_record is not None else self.rs_data.route_bus_id(list(self.rs_data.fixed_departures_by_route.keys())[0])
                except Exception:
                    bus_id_emit = ""
                self._matsim_emit({
                    "type": "arrive",
                    "ts": int(simulation_time),
                    "bus_id": bus_id_emit,
                    "route_id": int(rid_for_record) if rid_for_record is not None else None,
                    "stop_id": int(stop_id),
                    "planned_arrival": int(arr_planned) if arr_planned is not None else None,
                })
            # collect pick-ups and drop-offs
            elif vrl.status == VRL_STATES.BOARDING:
                cur = self._bus_usage_current.get(vid)
                if cur and cur.get("node_id") == node_id:
                    bd = getattr(vrl, 'rq_dict', {})
                    # avoid double recording of same rid pickup/dropoff across alternative attempts
                    seen_pu = self._bus_usage_seen_pickups.setdefault(vid, set())
                    seen_do = self._bus_usage_seen_dropoffs.setdefault(vid, set())
                    new_picks = []
                    for rq in bd.get(1, []):
                        rid = rq.get_rid()
                        if rid not in seen_pu:
                            new_picks.append(rid)
                            seen_pu.add(rid)
                    new_drops = []
                    for rq in bd.get(-1, []):
                        rid = rq.get_rid()
                        if rid not in seen_do:
                            new_drops.append(rid)
                            seen_do.add(rid)
                    cur['picks'].extend(new_picks)
                    cur['drops'].extend(new_drops)
                    # set planned departure time from route plan to enforce 30s dwell in logging
                    dep_planned = None
                    try:
                        rp = self.veh_route_plans.get(vid, {}).get(cur.get('route_id', rid_for_record))
                        if rp is not None:
                            for ps in rp.list_plan_stops:
                                if ps.get_pos()[0] == node_id:
                                    _, dep_planned = ps.get_planned_arrival_and_departure_time()
                                    break
                    except Exception:
                        pass
                    cur['dep'] = int(dep_planned) if dep_planned is not None else simulation_time
                    # MATSim board event
                    try:
                        bus_id_emit = self.rs_data.route_bus_id(cur.get('route_id', rid_for_record)) if cur.get('route_id', rid_for_record) is not None else self.rs_data.route_bus_id(list(self.rs_data.fixed_departures_by_route.keys())[0])
                    except Exception:
                        bus_id_emit = ""
                    self._matsim_emit({
                        "type": "board",
                        "ts": int(simulation_time),
                        "bus_id": bus_id_emit,
                        "route_id": int(cur.get('route_id', rid_for_record)) if cur.get('route_id', rid_for_record) is not None else None,
                        "stop_id": int(stop_id),
                        "pick_rids": cur.get('picks', []),
                        "drop_rids": cur.get('drops', []),
                        "planned_departure": int(dep_planned) if dep_planned is not None else None,
                    })
            # departure for fixed planned stops
            elif vrl.status == VRL_STATES.PLANNED_STOP:
                cur = self._bus_usage_current.get(vid)
                # helper to resolve route plan and planned times
                def _resolve_plan_and_times(route_hint):
                    plan = None
                    arr_p = None
                    dep_p = None
                    if route_hint is not None:
                        plan = self.veh_route_plans.get(vid, {}).get(route_hint)
                    if plan is None:
                        for _rid, _plan in self.veh_route_plans.get(vid, {}).items():
                            if any(ps.get_pos()[0] == node_id for ps in _plan.list_plan_stops):
                                plan = _plan
                                break
                    if plan is not None:
                        for ps in plan.list_plan_stops:
                            if ps.get_pos()[0] == node_id:
                                arr_p, dep_p = ps.get_planned_arrival_and_departure_time()
                                break
                    return plan, arr_p, dep_p

                # If we have the current stop cached and matching, use it; otherwise handle fixed-stop directly
                if cur and cur.get("node_id") == node_id:
                    # determine route id for this row
                    route_for_row = cur.get('route_id', rid_for_record)
                    if route_for_row is None and self.rs_data.stops_by_id.get(stop_id).fixed:
                        route_for_row = active_rid
                    # planned times
                    _, arr_planned, dep_planned = None, None, None
                    try:
                        _, arr_planned, dep_planned = _resolve_plan_and_times(route_for_row)
                    except Exception:
                        pass
                    # enforce scheduled departure for fixed stops
                    if self.rs_data.stops_by_id.get(stop_id).fixed and route_for_row is not None:
                        sched_dep = self.rs_data.get_departure_time(route_for_row, stop_id)
                    else:
                        sched_dep = None
                    dep_t = int(sched_dep) if sched_dep is not None else (int(dep_planned) if dep_planned is not None else simulation_time)
                    # arrivals: for fixed stops use planned arrival; for first stop force dep-30; else clamp to <= dep-30
                    arr_t = cur.get('arrival', simulation_time)
                    if self.rs_data.stops_by_id.get(stop_id).fixed:
                        if arr_planned is not None:
                            arr_t = int(arr_planned)
                        try:
                            if route_for_row is not None:
                                first_fixed_sid = self.rs_data.fixed_departures_by_route[route_for_row][0].stop_id
                                if stop_id == first_fixed_sid and dep_t is not None:
                                    arr_t = max(0, int(dep_t) - 30)
                                else:
                                    # clamp arrival before scheduled dep-30 if necessary
                                    arr_t = min(arr_t, int(dep_t) - 30)
                        except Exception:
                            pass
                    self._write_bus_usage_row(bus_id, route_for_row, node_id, arr_t, dep_t, cur.get('picks', []), cur.get('drops', []))
                    # MATSim depart event
                    self._matsim_emit({
                        "type": "depart",
                        "ts": int(simulation_time),
                        "bus_id": bus_id,
                        "route_id": int(route_for_row) if route_for_row is not None else None,
                        "stop_id": int(stop_id),
                        "planned_departure": int(dep_t) if dep_t is not None else None,
                    })
                    self._bus_usage_current.pop(vid, None)
                else:
                    # No cached state (e.g., first fixed stop had no driving leg). If fixed stop, write directly from plan/schedule.
                    if self.rs_data.stops_by_id.get(stop_id).fixed:
                        route_for_row = rid_for_record if rid_for_record is not None else active_rid
                        sched_dep = None
                        if route_for_row is not None:
                            sched_dep = self.rs_data.get_departure_time(route_for_row, stop_id)
                        _, arr_planned, _ = _resolve_plan_and_times(route_for_row)
                        dep_t = int(sched_dep) if sched_dep is not None else simulation_time
                        # arrival handling as above
                        if route_for_row is not None:
                            try:
                                first_fixed_sid = self.rs_data.fixed_departures_by_route[route_for_row][0].stop_id
                            except Exception:
                                first_fixed_sid = None
                        else:
                            first_fixed_sid = None
                        if stop_id == first_fixed_sid and dep_t is not None:
                            arr_t = max(0, int(dep_t) - 30)
                        else:
                            arr_t = int(arr_planned) if arr_planned is not None else max(0, int(dep_t) - 30)
                        self._write_bus_usage_row(bus_id, route_for_row, node_id, arr_t, dep_t, [], [])
                        # MATSim depart event
                        self._matsim_emit({
                            "type": "depart",
                            "ts": int(simulation_time),
                            "bus_id": bus_id,
                            "route_id": int(route_for_row) if route_for_row is not None else None,
                            "stop_id": int(stop_id),
                            "planned_departure": int(dep_t) if dep_t is not None else None,
                        })
                    # if this is the last fixed stop of the active route, write final plan snapshot once
                    try:
                        if active_rid is not None:
                            last_fixed_sid = self.rs_data.fixed_departures_by_route[active_rid][-1].stop_id
                            if self.rs_data.node_to_stop_id.get(node_id) == last_fixed_sid:
                                if (vid, active_rid) not in self._vp_written_routes:
                                    route_plan = self.veh_route_plans.get(vid, {}).get(active_rid)
                                    if route_plan is not None:
                                        self._write_route_plan_snapshot(bus_id, vid, active_rid, route_plan)
                                        # MATSim plan complete notification
                                        self._matsim_emit({
                                            "type": "route_completed",
                                            "ts": int(simulation_time),
                                            "bus_id": bus_id,
                                            "route_id": int(active_rid),
                                        })
                                        self._vp_written_routes.add((vid, active_rid))
                    except Exception:
                        pass

    def _write_route_plan_snapshot(self, bus_id: str, vid: int, route_id: int, route_plan):
        try:
            iteration = self.scenario_parameters.get("matsim_iteration", 0)
            with open(self._vehplan_f, "a", newline='', encoding='utf-8') as fh:
                w = csv.writer(fh)
                seq = 0
                for ps in route_plan.list_plan_stops:
                    node_id = ps.get_pos()[0]
                    stop_id = self.rs_data.node_to_stop_id.get(node_id)
                    if stop_id is None:
                        continue
                    try:
                        fixed = bool(ps.is_fixed_stop())
                    except Exception:
                        fixed = False
                    arr, dep = ps.get_planned_arrival_and_departure_time()
                    bd = getattr(ps, 'boarding_dict', {}) or {}
                    picks = ";".join(map(str, bd.get(1, []))) if bd.get(1) else ""
                    drops = ";".join(map(str, bd.get(-1, []))) if bd.get(-1) else ""
                    w.writerow([iteration, bus_id, vid, route_id, seq, stop_id, node_id, int(fixed), int(arr) if arr is not None else "", int(dep) if dep is not None else "", picks, drops])
                    seq += 1
        except Exception:
            LOG.debug(f"[RideSync] failed to write route plan snapshot for vid={vid} route_id={route_id}")

    # removed unused route inference helper

    def _create_user_offer(self, prq: PlanRequest, simulation_time: int, assigned_vehicle_plan=None, offer_dict_without_plan: Dict = {}):
        if assigned_vehicle_plan is not None:
            pu_time, do_time = assigned_vehicle_plan.pax_info.get(prq.get_rid_struct())
            offer = TravellerOffer(prq.get_rid_struct(), self.op_id, pu_time - prq.rq_time, do_time - pu_time, 0)
            prq.set_service_offered(offer)
        else:
            offer = self._create_rejection(prq, simulation_time)
        return offer

    def change_prq_time_constraints(self, sim_time: int, rid: Any, new_lpt: int, new_ept: int = None):
        LOG.debug("change time constraints for rid {}".format(rid))
        prq = self.rq_dict[rid]
        prq.set_new_pickup_time_constraint(new_lpt, new_ept)
        ass_vid = self.rid_to_assigned_vid.get(rid)
        if ass_vid is not None:
            self.veh_plans[ass_vid].update_prq_hard_constraints(self.sim_vehicles[ass_vid], sim_time,
                                                                self.routing_engine, prq, new_lpt, new_ept=new_ept,
                                                                keep_feasible=True)

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, assigned_charging_task=None, add_arg=None):
        super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign, assigned_charging_task=assigned_charging_task, add_arg=add_arg)

    def lock_current_vehicle_plan(self, vid):
        super().lock_current_vehicle_plan(vid)

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        super()._lock_vid_rid_pickup(sim_time, vid, rid)

    def _call_time_trigger_request_batch(self, simulation_time: int):
        self.sim_time = simulation_time
        self.pos_veh_dict = {}
        # detect likely declined offers (no confirmation after threshold)
        try:
            threshold = 1800
            to_remove = []
            for rid, t0 in self._pending_offers.items():
                if simulation_time - t0 > threshold:
                    LOG.debug(f"[RideSync] offer likely declined rid={rid} age={simulation_time - t0}s")
                    # Emit MATSim offer_declined
                    self._matsim_emit({
                        "type": "offer_declined",
                        "ts": int(simulation_time),
                        "rid": int(rid) if isinstance(rid, int) else None,
                    })
                    to_remove.append(rid)
            for rid in to_remove:
                del self._pending_offers[rid]
                try:
                    if rid in self.tmp_assignment:
                        del self.tmp_assignment[rid]
                except Exception:
                    pass
                try:
                    if rid in self._offer_cache:
                        del self._offer_cache[rid]
                except Exception:
                    pass
        except Exception:
            pass

    def acknowledge_boarding(self, rid: Any, vid: int, simulation_time: int):
        LOG.debug(f"acknowledge boarding {rid} in {vid} at {simulation_time}")
        self.rq_dict[rid].set_pickup(vid, simulation_time)

    def acknowledge_alighting(self, rid: Any, vid: int, simulation_time: int):
        LOG.debug(f"acknowledge alighting {rid} from {vid} at {simulation_time}")
        del self.rq_dict[rid]
        del self.rid_to_assigned_vid[rid]

    def _prq_from_reservation_to_immediate(self, rid, sim_time):
        LOG.debug(f"activate {rid} for global optimisation at time {sim_time}!")
        self.rq_dict[rid].set_reservation_flag(False)

    def compute_VehiclePlan_utility(self, simulation_time, veh_obj, vehicle_plan):
        """Prune stale rids from plan before computing utility to avoid KeyError in objective."""
        # prune pax_info
        stale = [rid for rid in list(vehicle_plan.pax_info.keys()) if rid not in self.rq_dict]
        for rid in stale:
            try:
                del vehicle_plan.pax_info[rid]
            except KeyError:
                pass
        # prune plan stop boarding_dict
        for ps in vehicle_plan.list_plan_stops:
            try:
                bd = ps.boarding_dict
            except Exception:
                continue
            for key in (1, -1):
                if key in bd:
                    bd[key] = [rid for rid in bd[key] if rid in self.rq_dict]
        return self.vr_ctrl_f(simulation_time, veh_obj, vehicle_plan, self.rq_dict, self.routing_engine)

    def _write_bus_usage_row(self, bus_id: str, route_id: Optional[int], node_id: int, arr: float, dep: float, picks: list, drops: list):
        stop_id = self.rs_data.node_to_stop_id.get(node_id)
        if stop_id is None:
            return
        # deduplicate identical rows to avoid multiple writes for the same stop/event
        picks_s = ";".join(map(str, sorted(picks))) if picks else ""
        drops_s = ";".join(map(str, sorted(drops))) if drops else ""
        iteration = self.scenario_parameters.get("matsim_iteration", 0)
        row_key = (iteration, bus_id, route_id if route_id is not None else "", stop_id, int(arr), int(dep), picks_s, drops_s)
        if row_key in self._bus_usage_seen:
            return
        self._bus_usage_seen.add(row_key)
        with open(self._bus_usage_f, "a", newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(list(row_key))

    def user_confirms_booking(self, rid: Any, simulation_time: int):
        # The rid parameter should match the key used in rq_dict (rq.rid)
        prq = self.rq_dict.get(rid)
        
        if prq is None:
            LOG.warning(f"RideSync booking confirmed but request {rid} not found in rq_dict")
            return
            
        super().user_confirms_booking(rid, simulation_time)
        LOG.debug(f"RideSync booking confirmed {rid} at {simulation_time}")
        print(f"[DEBUG RideSync] Booking confirmed for request {rid} at time {simulation_time}") #new-change
        
        # Add minimum prebooking buffer for MATSim
        self._matsim_prebooking_buffer = 60  # seconds minimum buffer for prebooking
        
        if rid not in self.tmp_assignment:
            # attempt recovery: try cached plan metadata or locate rid in existing route plans
            LOG.warning(f"RideSync booking confirmed but {rid} not in tmp_assignment")
            assigned_plan = None
            vid = None
            route_id = None
            meta = None
            try:
                meta = self._offer_cache.get(rid)
            except Exception:
                meta = None
            if meta is not None:
                try:
                    vid = meta.get("vid")
                    route_id = meta.get("route_id")
                    pu_stop = meta.get("pu_stop")
                    do_stop = meta.get("do_stop")
                    if vid is not None and route_id is not None:
                        self._ensure_route_plan_exists(vid, simulation_time, route_id)
                        veh_obj = self.sim_vehicles[vid]
                        base_plan = self.veh_route_plans[vid][route_id]
                        tmp = self.planner.insert_optional_pair(veh_obj, simulation_time, base_plan, route_id, pu_stop, do_stop, rid, getattr(self.rq_dict[rid], 'nr_pax', 1), allow_fallback=True, anchor_to_route_start=True)
                        if tmp is not None:
                            assigned_plan = tmp
                except Exception:
                    assigned_plan = None
            if assigned_plan is None:
                try:
                    for v_id, plans in self.veh_route_plans.items():
                        for r_id, plan in plans.items():
                            try:
                                bd_has = any((rid in (getattr(ps, 'boarding_dict', {}) or {}).get(1, []) or rid in (getattr(ps, 'boarding_dict', {}) or {}).get(-1, [])) for ps in plan.list_plan_stops)
                            except Exception:
                                bd_has = False
                            if bd_has or (rid in getattr(plan, 'pax_info', {})):
                                vid = v_id
                                route_id = r_id
                                assigned_plan = plan
                                break
                        if assigned_plan is not None:
                            break
                except Exception:
                    assigned_plan = None
            if assigned_plan is None or vid is None or route_id is None:
                return
        else:
            vid, route_id, assigned_plan = self.tmp_assignment[rid]
        # persist to route-specific plan store
        self.veh_route_plans[vid][route_id] = assigned_plan
        
        # For RideSync: Only assign plans when the route becomes active
        # This preserves the schedule-based timing that RideSync uses
        # Immediate assignment for future routes would recompute timings from current time, breaking the schedule
        is_matsim_coupling = self.scenario_parameters.get("sim_env") == "MobiTopp" or self.scenario_parameters.get("rq_type") == "SlaveRequest"
        
        active_rid = self._active_route_id(simulation_time)
        print(f"[DEBUG RideSync] Active route at time {simulation_time}: {active_rid}, Request route: {route_id}, MATSim coupling: {is_matsim_coupling}") #new-change
        
        # Only assign when route is active (regardless of MATSim coupling)
        if active_rid == route_id:
            try:
                veh_obj = self.sim_vehicles[vid]
                print(f"[DEBUG RideSync] Assigning plan to vehicle {vid} for route {route_id}")
                # For MATSim coupling: keep only stops with boarding/alighting (actionable stops)
                # Fixed stops with no boarding are sent by MATSimSocket but not kept in FleetPy's plan
                # This prevents desynchronization between FleetPy (which tracks all stops) and MATSim (which only gets actionable stops)
                try:
                    plan_to_assign = assigned_plan.copy()
                    if is_matsim_coupling:
                        # Filter to only actionable stops (with boarding/alighting)
                        stops_to_keep = []
                        for ps in plan_to_assign.list_plan_stops:
                            bd = getattr(ps, 'boarding_dict', {}) or {}
                            if len(bd.get(1, [])) > 0 or len(bd.get(-1, [])) > 0:
                                stops_to_keep.append(ps)
                        plan_to_assign.list_plan_stops = stops_to_keep
                        # DON'T recompute timings - they're already correct from schedule-based planning
                    else:
                        # For non-MATSim: recompute timings from current vehicle position
                        plan_to_assign.update_tt_and_check_plan(veh_obj, simulation_time, self.routing_engine, keep_feasible=True)
                except Exception:
                    plan_to_assign = assigned_plan
                self.assign_vehicle_plan(veh_obj, plan_to_assign, simulation_time, force_assign=is_matsim_coupling)
                # Mark this (vid, route_id) as assigned so receive_status_update doesn't re-assign it
                self._assigned_routes.add((vid, route_id))
                LOG.debug(f"[RideSync] Immediately assigned plan for rid={rid} route={route_id} (MATSim={is_matsim_coupling} or route is active)")
            except Exception as e:
                LOG.warning(f"[RideSync] Could not immediately assign plan rid={rid}: {e}")
                print(f"[DEBUG RideSync] ERROR assigning plan: {e}")
        else:
            LOG.debug(f"[RideSync] Plan for rid={rid} route={route_id} stored; will be assigned when route becomes active (current active route: {active_rid})")
            print(f"[DEBUG RideSync] Plan stored for later assignment (route not active yet)")
            # For MATSim coupling, send prebooking immediately: assign actionable-only plan now (force),
            # but do not mark as permanently assigned so we can re-assign full plan when route becomes active.
            if is_matsim_coupling:
                try:
                    veh_obj = self.sim_vehicles[vid]
                    plan_to_assign = assigned_plan.copy()
                    # keep only actionable stops (boarding/alighting)
                    stops_to_keep = []
                    for ps in plan_to_assign.list_plan_stops:
                        bd = getattr(ps, 'boarding_dict', {}) or {}
                        if len(bd.get(1, [])) > 0 or len(bd.get(-1, [])) > 0:
                            stops_to_keep.append(ps)
                    plan_to_assign.list_plan_stops = stops_to_keep
                    
                    # Set proper timing constraints for prebooking to prevent premature execution
                    if stops_to_keep:
                        # Get the first actionable stop's planned arrival time
                        first_stop = stops_to_keep[0]
                        planned_arr, planned_dep = first_stop.get_planned_arrival_and_departure_time()
                        if planned_arr is not None:
                            # Set earliest_start_time to prevent MATSim from starting too early
                            # Use planned arrival minus a small buffer for travel time
                            first_stop.earliest_start_time = max(simulation_time, planned_arr - 60)
                            LOG.debug(f"[RideSync] Set earliest_start_time={first_stop.earliest_start_time} for prebooking first stop")
                    
                    # don't recompute timings
                    self.assign_vehicle_plan(veh_obj, plan_to_assign, simulation_time, force_assign=True)
                    LOG.debug(f"[RideSync] Sent immediate MATSim prebooking assignment for rid={rid}, route={route_id}")
                except Exception as e:
                    LOG.warning(f"[RideSync] Failed to send prebooking assignment: {e}")
        
        try:
            if rid in self.tmp_assignment:
                del self.tmp_assignment[rid]
        except Exception:
            pass
        # also snapshot the current route plan for visibility
        try:
            bus_id = self.rs_data.route_bus_id(route_id)
            self._write_route_plan_snapshot(bus_id, vid, route_id, assigned_plan)
            # MATSim plan update
            self._matsim_emit({"type": "booking_confirmed", "ts": int(simulation_time), "rid": int(prq.get_rid()) if hasattr(prq, 'get_rid') else None, "route_id": int(route_id), "bus_id": bus_id})
            self._matsim_plan_update(bus_id, route_id, assigned_plan)
        except Exception:
            pass
        # clear pending flag and caches
        try:
            if rid in self._pending_offers:
                del self._pending_offers[rid]
        except Exception:
            pass
        try:
            if rid in self._offer_cache:
                del self._offer_cache[rid]
        except Exception:
            pass
        # remember rid -> route for later bus_usage attribution
        try:
            self.rid_to_route_id[rid] = route_id
            try:
                simple_rid = prq.get_rid()
                self.rid_to_route_id[simple_rid] = route_id
            except Exception:
                pass
        except Exception:
            pass

    def user_cancels_request(self, rid: Any, simulation_time: int):
        LOG.debug(f"RideSync booking cancelled {rid} at {simulation_time}")
        if rid in self.tmp_assignment:
            del self.tmp_assignment[rid]
        # Don't delete from rq_dict here - let it be removed properly when alighting
        # if rid in self.rq_dict:
        #     del self.rq_dict[rid]
        try:
            if rid in self._pending_offers:
                del self._pending_offers[rid]
        except Exception:
            pass
        try:
            if rid in self.rid_to_route_id:
                del self.rid_to_route_id[rid]
        except Exception:
            pass


