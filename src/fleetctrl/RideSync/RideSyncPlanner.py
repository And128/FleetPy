import logging
from typing import Dict, List, Tuple, Optional, Any

from src.fleetctrl.planning.VehiclePlan import VehiclePlan, PlanStop, BoardingPlanStop
from src.routing.NetworkBase import NetworkBase, return_node_position

from .RideSyncData import RideSyncData

LOG = logging.getLogger(__name__)

class RideSyncPlanner:
    """Build and update a per-route vehicle plan respecting:
    - fixed stops: enforced departure times from bus_schedule (order anchored)
    - optional stops: 30s dwell, flexible arrival/departure and flexible ordering to minimize travel time
    - precedence: for every rid, pickup occurs before dropoff; fixed stop order is immutable
    """

    def __init__(self, data: RideSyncData, routing_engine: NetworkBase):
        self.data = data
        self.routing_engine = routing_engine

    def build_base_plan(self, veh_obj, sim_time: int, route_id: int) -> VehiclePlan:
        """Create a VehiclePlan containing only fixed stops with enforced departure times.
        Dwell at fixed stops is flexible arrival with departure exactly at schedule.
        """
        dep_list = self.data.fixed_departures_by_route[route_id]
        LOG.debug(f"[RideSyncPlanner] build_base_plan route_id={route_id} fixed_stops={[(d.stop_id,d.departure_time) for d in dep_list]}")
        plan_stops: List[PlanStop] = []
        for dep in dep_list:
            pos = return_node_position(self.data.stops_by_id[dep.stop_id].node_index)
            # For fixed stops: duration = None, earliest_end_time = scheduled departure
            ps = BoardingPlanStop(
                pos,
                boarding_dict={},
                duration=None,
                earliest_end_time=dep.departure_time,  # cannot depart earlier
                latest_start_time=max(0, dep.departure_time - 30),  # ensure >=30s dwell at fixed stops
                fixed_stop=True,
            )
            plan_stops.append(ps)
        vp = VehiclePlan(veh_obj, sim_time, self.routing_engine, plan_stops)
        return vp

    def insert_optional_pair(self, veh_obj, sim_time: int, vp: VehiclePlan, route_id: int,
                             pickup_stop_id: int, dropoff_stop_id: int,
                             rid_struct: Any, pax_change: int = 1,
                             tried_pairs: Optional[set] = None,
                             allow_fallback: bool = True,
                             anchor_to_route_start: bool = False,
                             matsim_coupling: bool = False) -> Optional[VehiclePlan]:
        """Insert pickup and dropoff across multiple segments.
        - If a chosen stop is fixed, attach boarding/alighting to the existing fixed PlanStop (no new stop inserted)
        - If a chosen stop is optional, insert a 30s BoardingPlanStop (position will be optimized)
        - Respect enforced departures at fixed stops; optional stop order is optimized for travel time within fixed segments
        Returns a new plan if feasible, else None."""
        LOG.debug(f"[RideSyncPlanner] insert_optional_pair rid={rid_struct} route_id={route_id} pu={pickup_stop_id} do={dropoff_stop_id} allow_fallback={allow_fallback}")
        if tried_pairs is None:
            tried_pairs = set()
        key_pair = (pickup_stop_id, dropoff_stop_id)
        if key_pair in tried_pairs:
            return None
        tried_pairs.add(key_pair)
        new_vp = vp.copy()
        # Clean stale alightings with no prior boarding (can happen due to earlier pruning elsewhere)
        self._prune_plan_for_insertion(new_vp)
        # Enforce booking constraint: pickup stop_id < dropoff stop_id
        pu_order = self.data.stop_order_by_id[pickup_stop_id]
        do_order = self.data.stop_order_by_id[dropoff_stop_id]
        if not (int(pickup_stop_id) < int(dropoff_stop_id)):
            return None

        def _find_planstop_for_fixed(stop_id: int) -> Optional[PlanStop]:
            node_idx = self.data.stops_by_id[stop_id].node_index
            for ps in new_vp.list_plan_stops:
                if ps.get_pos()[0] == node_idx:
                    return ps
            return None

        # Handle pickup (fixed vs optional)
        if self.data.stops_by_id[pickup_stop_id].fixed:
            ps = _find_planstop_for_fixed(pickup_stop_id)
            if ps is None:
                return None
            bd = getattr(ps, 'boarding_dict', {}) or {}
            lst = bd.setdefault(1, [])
            if rid_struct not in lst:
                lst.append(rid_struct)
            ps.boarding_dict = bd
            try:
                ps.change_nr_pax = getattr(ps, 'change_nr_pax', 0) + pax_change
            except Exception:
                pass
            LOG.debug(f"[RideSyncPlanner] attached pickup to fixed stop_id={pickup_stop_id}")
        else:
            # optional pickup: if an optional PlanStop at this node already exists, attach; else insert a new 30s stop (insert before last fixed anchor)
            pu_node_idx = self.data.stops_by_id[pickup_stop_id].node_index
            attach_ps = None
            for ps in new_vp.list_plan_stops:
                if ps.get_pos()[0] == pu_node_idx and not getattr(ps, 'fixed_stop', False):
                    attach_ps = ps
                    break
            if attach_ps is not None:
                bd = getattr(attach_ps, 'boarding_dict', {}) or {}
                lst = bd.setdefault(1, [])
                if rid_struct not in lst:
                    lst.append(rid_struct)
                attach_ps.boarding_dict = bd
                try:
                    attach_ps.change_nr_pax = getattr(attach_ps, 'change_nr_pax', 0) + pax_change
                except Exception:
                    pass
                LOG.debug(f"[RideSyncPlanner] attached pickup to existing optional stop_id={pickup_stop_id}")
            else:
                pu_pos = return_node_position(pu_node_idx)
                # For MATSim coupling, set a minimum earliest_start_time to allow prebooking
                pu_earliest_start_time = None
                if matsim_coupling:
                    # Ensure at least 60 seconds from current sim_time for prebooking
                    pu_earliest_start_time = sim_time + 60
                pu_ps = BoardingPlanStop(pu_pos, boarding_dict={1: [rid_struct]}, duration=30, earliest_start_time=pu_earliest_start_time, fixed_stop=False, change_nr_pax=pax_change)
                # insert before last fixed stop to ensure no optional after route end
                last_fixed_idx = None
                for idx in range(len(new_vp.list_plan_stops) - 1, -1, -1):
                    try:
                        if new_vp.list_plan_stops[idx].is_fixed_stop():
                            last_fixed_idx = idx
                            break
                    except Exception:
                        continue
                ins_idx = last_fixed_idx if last_fixed_idx is not None else len(new_vp.list_plan_stops)
                new_vp.list_plan_stops.insert(ins_idx, pu_ps)
                LOG.debug(f"[RideSyncPlanner] inserted optional pickup before last fixed at idx={ins_idx} stop_id={pickup_stop_id}")

        # Insert/attach dropoff
        if self.data.stops_by_id[dropoff_stop_id].fixed:
            ps = _find_planstop_for_fixed(dropoff_stop_id)
            if ps is None:
                return None
            bd = getattr(ps, 'boarding_dict', {}) or {}
            lst = bd.setdefault(-1, [])
            if rid_struct not in lst:
                lst.append(rid_struct)
            ps.boarding_dict = bd
            try:
                ps.change_nr_pax = getattr(ps, 'change_nr_pax', 0) - pax_change
            except Exception:
                pass
            LOG.debug(f"[RideSyncPlanner] attached dropoff to fixed stop_id={dropoff_stop_id}")
        else:
            # optional dropoff: attach to existing optional stop at this node if present; else insert a new 30s stop
            do_node_idx = self.data.stops_by_id[dropoff_stop_id].node_index
            attach_ps = None
            for ps in new_vp.list_plan_stops:
                if ps.get_pos()[0] == do_node_idx and not getattr(ps, 'fixed_stop', False):
                    attach_ps = ps
                    break
            if attach_ps is not None:
                bd = getattr(attach_ps, 'boarding_dict', {}) or {}
                lst = bd.setdefault(-1, [])
                if rid_struct not in lst:
                    lst.append(rid_struct)
                attach_ps.boarding_dict = bd
                try:
                    attach_ps.change_nr_pax = getattr(attach_ps, 'change_nr_pax', 0) - pax_change
                except Exception:
                    pass
                LOG.debug(f"[RideSyncPlanner] attached dropoff to existing optional stop_id={dropoff_stop_id}")
            else:
                do_pos = return_node_position(do_node_idx)
                do_ps = BoardingPlanStop(do_pos, boarding_dict={-1: [rid_struct]}, duration=30, fixed_stop=False, change_nr_pax=-pax_change)
                # insert before last fixed stop to ensure no optional after route end
                last_fixed_idx = None
                for idx in range(len(new_vp.list_plan_stops) - 1, -1, -1):
                    try:
                        if new_vp.list_plan_stops[idx].is_fixed_stop():
                            last_fixed_idx = idx
                            break
                    except Exception:
                        continue
                ins_idx = last_fixed_idx if last_fixed_idx is not None else len(new_vp.list_plan_stops)
                new_vp.list_plan_stops.insert(ins_idx, do_ps)
                LOG.debug(f"[RideSyncPlanner] inserted optional dropoff before last fixed at idx={ins_idx} stop_id={dropoff_stop_id}")

        # Ensure rid is attached exactly once for pickup and once for dropoff at the intended nodes
        try:
            pu_node_idx_final = self.data.stops_by_id[pickup_stop_id].node_index
            do_node_idx_final = self.data.stops_by_id[dropoff_stop_id].node_index
            for ps in new_vp.list_plan_stops:
                bd = getattr(ps, 'boarding_dict', {}) or {}
                if bd.get(1):
                    if ps.get_pos()[0] != pu_node_idx_final and rid_struct in bd[1]:
                        bd[1] = [r for r in bd[1] if r != rid_struct]
                if bd.get(-1):
                    if ps.get_pos()[0] != do_node_idx_final and rid_struct in bd[-1]:
                        bd[-1] = [r for r in bd[-1] if r != rid_struct]
                ps.boarding_dict = bd
        except Exception:
            pass

        # Reorder optional stops within each fixed-stop segment to reduce travel time while preserving precedence and fixed stop order
        try:
            self._reorder_optional_segments(new_vp)
        except Exception:
            pass
        # Global precedence validation mirroring VehiclePlan processing order: in each PlanStop, boarding first, then alighting
        if self._has_global_precedence_violation(new_vp):
            LOG.debug("[RideSyncPlanner] precedence violation detected after reordering; rejecting plan to avoid drop-before-pick")
            return None

        # Recompute timings (optionally anchored at route's first fixed stop)
        init_state = None
        if anchor_to_route_start:
            try:
                first_dep = self.data.fixed_departures_by_route[route_id][0]
                first_node = self.data.stops_by_id[first_dep.stop_id].node_index
                init_state = {
                    "stop_index": -1,
                    "c_pos": return_node_position(first_node),
                    "c_soc": getattr(veh_obj, 'soc', 1.0),
                    "c_time": max(0, int(first_dep.departure_time) - 30),
                    "c_pax": {},
                    "pax_info": {},
                    "c_nr_pax": 0,
                    "c_nr_parcels": 0,
                }
            except Exception:
                init_state = None
        feasible = new_vp.update_tt_and_check_plan(veh_obj, sim_time, self.routing_engine, init_plan_state=init_state)
        LOG.debug(f"[RideSyncPlanner] plan feasibility after insertion: {feasible}")
        if not feasible:
            if not allow_fallback:
                return None
            # Fallback attempts: choose nearest already-planned stops from the base plan (exclude just-inserted optional stops)
            base_planned_ids = [self.data.node_to_stop_id.get(ps.get_pos()[0], None) for ps in vp.list_plan_stops]
            planned_ids = [sid for sid in base_planned_ids if sid is not None]
            LOG.debug(f"[RideSyncPlanner] infeasible plan; trying fallback with planned_ids={planned_ids}")

            def nearest_planned_pickup(original_sid: int, max_order: int) -> int:
                orig_ord = self.data.stop_order_by_id[original_sid]
                candidates = []
                for sid in planned_ids:
                    if sid == original_sid:
                        continue
                    so = self.data.stop_order_by_id[sid]
                    if so < max_order:  # maintain pu_order < do_order
                        candidates.append((abs(so - orig_ord), so, sid))
                candidates.sort(key=lambda t: (t[0], t[1]))
                return candidates[0][2] if candidates else original_sid

            def nearest_planned_dropoff(original_sid: int, min_order: int) -> int:
                orig_ord = self.data.stop_order_by_id[original_sid]
                candidates = []
                for sid in planned_ids:
                    if sid == original_sid:
                        continue
                    so = self.data.stop_order_by_id[sid]
                    if so > min_order:  # maintain pu_order < do_order
                        candidates.append((abs(so - orig_ord), so, sid))
                candidates.sort(key=lambda t: (t[0], t[1]))
                return candidates[0][2] if candidates else original_sid

            # Try replace pickup only (closest planned stop before current dropoff order)
            rep_pu = nearest_planned_pickup(pickup_stop_id, do_order)
            if rep_pu != pickup_stop_id and (rep_pu, dropoff_stop_id) not in tried_pairs:
                try_vp = vp.copy()
                return self.insert_optional_pair(veh_obj, sim_time, try_vp, route_id, rep_pu, dropoff_stop_id, rid_struct, pax_change, tried_pairs, allow_fallback=True, anchor_to_route_start=anchor_to_route_start, matsim_coupling=matsim_coupling)

            # Try replace dropoff only (closest planned stop after current pickup order)
            rep_do = nearest_planned_dropoff(dropoff_stop_id, pu_order)
            if rep_do != dropoff_stop_id and (pickup_stop_id, rep_do) not in tried_pairs:
                try_vp = vp.copy()
                return self.insert_optional_pair(veh_obj, sim_time, try_vp, route_id, pickup_stop_id, rep_do, rid_struct, pax_change, tried_pairs, allow_fallback=True, anchor_to_route_start=anchor_to_route_start, matsim_coupling=matsim_coupling)

            # Try replace both
            if (rep_pu != pickup_stop_id or rep_do != dropoff_stop_id) and (rep_pu, rep_do) not in tried_pairs:
                try_vp = vp.copy()
                return self.insert_optional_pair(veh_obj, sim_time, try_vp, route_id, rep_pu, rep_do, rid_struct, pax_change, tried_pairs, allow_fallback=True, anchor_to_route_start=anchor_to_route_start, matsim_coupling=matsim_coupling)

            return None
        # Fail-safe capacity pass: ensure occupancy never exceeds max_pax when applying boarding_dict deltas
        try:
            start_pax = 0 if anchor_to_route_start else veh_obj.get_nr_pax_without_currently_boarding()
        except Exception:
            start_pax = 0
        cur_pax = start_pax
        max_pax = getattr(veh_obj, 'max_pax', 20)
        for ps in new_vp.list_plan_stops:
            bd = getattr(ps, 'boarding_dict', {}) or {}
            cur_pax += len(bd.get(1, [])) - len(bd.get(-1, []))
            if cur_pax > max_pax:
                LOG.debug(f"[RideSyncPlanner] capacity violation: pax={cur_pax} > max={max_pax} at node={ps.get_pos()[0]}")
                return None
        return new_vp

    def _prune_plan_for_insertion(self, veh_plan: VehiclePlan):
        """Remove alighting entries for rids that have no boarding time in pax_info,
        and drop empty non-fixed stops to avoid KeyErrors during update_tt.
        """
        # 1) remove alighting-only rids (not present in pax_info) from boarding_dict at each stop
        for ps in veh_plan.list_plan_stops:
            try:
                bd = ps.boarding_dict
            except Exception:
                continue
            # remove alightings without prior boarding info
            if -1 in bd:
                cleaned = [rid for rid in bd[-1] if veh_plan.pax_info.get(rid) is not None]
                if len(cleaned) != len(bd[-1]):
                    LOG.debug(f"[RideSyncPlanner] pruned alighting-only rids at stop {ps.get_pos()}")
                bd[-1] = cleaned
            # sanity: if a boarding list exists but pax_change was set erroneously, neutralize pax count to avoid inconsistencies
            if len(bd.get(1, [])) == 0 and len(bd.get(-1, [])) == 0:
                try:
                    # neutralize pax deltas if no boarding/alighting here
                    ps.change_nr_pax = 0
                except Exception:
                    pass
        # 2) drop empty non-fixed stops
        filtered = []
        for ps in veh_plan.list_plan_stops:
            try:
                is_fixed = ps.is_fixed_stop()
            except Exception:
                is_fixed = False
            try:
                bd = ps.boarding_dict
            except Exception:
                bd = {}
            if not is_fixed and len(bd.get(1, [])) == 0 and len(bd.get(-1, [])) == 0 and getattr(ps, 'change_nr_pax', 0) == 0:
                # drop
                continue
            filtered.append(ps)
        veh_plan.list_plan_stops = filtered

    def _has_global_precedence_violation(self, veh_plan: VehiclePlan) -> bool:
        """Return True if any rid appears in a dropoff before its pickup when processing stops sequentially,
        simulating update_tt_and_check_plan's per-stop ordering (boardings first, then alightings)."""
        seen_pick = set()
        for ps in veh_plan.list_plan_stops:
            bd = getattr(ps, 'boarding_dict', {}) or {}
            # first, boardings
            for rid in bd.get(1, []) or []:
                seen_pick.add(rid)
            # then, alightings
            for rid in bd.get(-1, []) or []:
                if rid not in seen_pick:
                    return True
        return False

    def _reorder_optional_segments(self, veh_plan: VehiclePlan) -> None:
        """Within each pair of consecutive fixed stops, reorder optional stops to greedily minimize travel time
        subject to precedence constraints (pickup before dropoff for every rid)."""
        # Locate indices of fixed stops
        fixed_indices = []
        for idx, ps in enumerate(veh_plan.list_plan_stops):
            try:
                if ps.is_fixed_stop():
                    fixed_indices.append(idx)
            except Exception:
                continue
        if not fixed_indices:
            return
        # Process each segment between fixed stops
        for seg_start_i, seg_end_i in zip(fixed_indices[:-1], fixed_indices[1:]):
            segment = veh_plan.list_plan_stops[seg_start_i + 1:seg_end_i]
            if not segment:
                continue
            # Build precedence info (record which rids have pickups in this segment)
            rid_pick_seen: Dict[Any, bool] = {}
            rid_has_pick: Dict[Any, bool] = {}
            for ps in segment:
                bd = getattr(ps, 'boarding_dict', {}) or {}
                for rid in bd.get(1, []) or []:
                    rid_has_pick[rid] = True
            # Greedy nearest-neighbor with precedence filtering
            current_pos = veh_plan.list_plan_stops[seg_start_i].get_pos()
            remaining = list(segment)
            ordered: List[PlanStop] = []
            while remaining:
                candidates = []
                for ps in remaining:
                    bd = getattr(ps, 'boarding_dict', {}) or {}
                    # Do not choose a stop that contains a dropoff for any rid whose pickup hasn't been visited yet
                    illegal = False
                    for rid in bd.get(-1, []) or []:
                        if rid_has_pick.get(rid, False) and not rid_pick_seen.get(rid, False):
                            illegal = True
                            break
                    if illegal:
                        continue
                    dpos = ps.get_pos()
                    try:
                        _, tt, _ = self.routing_engine.return_travel_costs_1to1(current_pos, dpos)
                    except Exception:
                        tt = 0
                    candidates.append((tt, ps))
                if not candidates:
                    # fallback: pick a pickup-only stop if any
                    picked_any = False
                    for ps in list(remaining):
                        bd = getattr(ps, 'boarding_dict', {}) or {}
                        if bd.get(1):
                            ordered.append(ps)
                            remaining.remove(ps)
                            for rid in bd.get(1, []) or []:
                                rid_pick_seen[rid] = True
                            current_pos = ps.get_pos()
                            picked_any = True
                            break
                    if not picked_any:
                        break
                    continue
                candidates.sort(key=lambda t: t[0])
                chosen = candidates[0][1]
                ordered.append(chosen)
                remaining.remove(chosen)
                bd = getattr(chosen, 'boarding_dict', {}) or {}
                for rid in bd.get(1, []) or []:
                    rid_pick_seen[rid] = True
                current_pos = chosen.get_pos()
            # Rewrite segment
            veh_plan.list_plan_stops[seg_start_i + 1:seg_end_i] = ordered
