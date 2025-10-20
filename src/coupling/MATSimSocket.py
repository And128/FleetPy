import os
import zmq
import json
import traceback
import datetime
import pandas as pd
from typing import TYPE_CHECKING, Dict, List, Tuple, Any
import logging

to_del = []
for p in os.sys.path:
    if "FleetPy" in p:
        to_del.append(p)
for p in to_del:
    os.sys.path.remove(p)
os.sys.path.append(r"C:\Users\ge37ser\Documents\Coding\FleetPy")

from src.misc.globals import *
from src.coupling.misc import *
from src.coupling.MATSimSimulationClass import MATSimSimulationClass
from src.FleetSimulationBase import build_operator_attribute_dicts

if TYPE_CHECKING:
    from src.fleetctrl.planning.VehiclePlan import VehiclePlan
    
LOG = logging.getLogger(__name__)

STAT_INT = 60
ENCODING = "utf-8"
LOG_COMMUNICATION = True
LARGE_INT = 100000

class MATSimSocket:
    """
    A class to handle communication with a MATSim server using sockets.
    """
    def __init__(self, host: str, port: int, scenario_parameters, log_communication: bool = LOG_COMMUNICATION):
        self.server_ip = host
        self.server_port = port
        self.log_communication = log_communication
        self.matsim_iteration = 0
        scenario_parameters["matsim_iteration"] = self.matsim_iteration
        self.scenario_parameters = scenario_parameters
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{self.server_ip}:{self.server_port}")
        
        # build list of operator dictionaries  # TODO: this could be eliminated with a new YAML-based config system
        self.list_op_dicts = build_operator_attribute_dicts(scenario_parameters, scenario_parameters[G_NR_OPERATORS],
                                                                              prefix="op_")
        self.list_ch_op_dicts  = build_operator_attribute_dicts(scenario_parameters, scenario_parameters.get(G_NR_CH_OPERATORS, 0),
                                                                                 prefix="ch_op_")
        
        self.dir_names = get_directory_dict(scenario_parameters, self.list_op_dicts)
        self.scenario_parameters: dict = scenario_parameters
        
        self.matsim_edge_to_fp_edge, self.fp_edge_to_matsim_edge = self._create_fleetpy_network(scenario_parameters["matsim_network_path"])
        # check unique mapping
        self._non_unique_matsim_links = []
        for matsim_link, fp_edge in self.matsim_edge_to_fp_edge.items():
            rev_matsim_link = self.fp_edge_to_matsim_edge[fp_edge[0]][fp_edge[1]]
            if rev_matsim_link != matsim_link:
                LOG.warning(f"Mapping between MATSim and FleetPy edges is not unique! {matsim_link} -> {fp_edge} -> {rev_matsim_link}")
                self._non_unique_matsim_links.append(matsim_link)
        
        self.matsim_to_fleetpy_vid = {}
        self.fleetpy_to_matsim_vid = {}
        
        self.matsim_to_fleetpy_rid = {}
        self.fleetpy_to_matsim_rid = {}
        self._fp_rid_counter = 0
        
        self._last_veh_state = {}   # vid -> [state, [list_current_boarding], [list_current_alighting]]
        self._force_veh_pos_update_interval = scenario_parameters.get("force_veh_pos_update_interval", 1)
        
        self.fs_obj = MATSimSimulationClass(scenario_parameters)
        self.dir_names = self.fs_obj.dir_names
        
        self._network_update_interval = scenario_parameters.get(G_MATSIM_STAT_INT, None)
        if self._network_update_interval is not None:
            self._network_update_interval = int(self._network_update_interval)
        self._last_network_update_time = None
        self._stored_matsim_response = None # need to store updated state response if network update is requested
        
        # create communication log
        self._output_dir = self.fs_obj.dir_names[G_DIR_OUTPUT]
        self.log_f = os.path.join(self._output_dir, "00_socket_com.txt")
        self.last_stat_report_time = datetime.datetime.now()
        with open(self.log_f, "w") as fh_touch:
            fh_touch.write(f"{self.last_stat_report_time}: Opening socket communication ...\n")
            
        self._simulation_terminated = False
        # cache last non-empty assignment per vehicle to avoid clearing schedules prematurely
        self._last_assignment_by_vid = {} #new-change
        # Track current and completed pickups/dropoffs per vehicle (FleetPy rid ints)
        self._fp_current_pickups_by_vid = {} #new-change (line 97-102)
        self._fp_pickedup_by_vid = {}
        self._fp_current_dropoffs_by_vid = {}
        self._fp_droppedoff_by_vid = {}
        # Freeze earliestStartTime for pickups per FleetPy rid to avoid drifting beyond request LPT in later updates
        self._pickup_earliest_by_fprid = {}
        # Stable numeric stop ids per (veh, type, rid set, link)
        self._stable_stop_id = {}
        self._stop_id_counter = 1
        # Map MATSim request ids to their exact origin/destination MATSim link ids
        self._rid_to_matsim_origin = {}
        self._rid_to_matsim_destination = {}
        # Track pickups already sent to MATSim to avoid duplicate scheduling
        self._pickup_sent_to_matsim = set()
        # Track vehicles with active prebookings (vid -> set of MATSim rids with full prebooking)
        self._vehicle_prebooking_active = {}  # fp_vid -> set of matsim rids
        # (reverted) remove added tracking structures to restore previous behavior
                
    def log_com(self, msg):
        with open(self.log_f, "a") as fhout:
            fhout.write(msg)
            
    def format_object_and_send_msg(self, obj):
        json_content = json.dumps(obj)
        msg = json_content + "\n"
        if self.log_communication:
            prt_str = f"sending: {msg} to {self.socket}\n" + "-" * 20 + "\n"
            self.log_com(prt_str)
        byte_msg = msg.encode(ENCODING)
        self.socket.send(byte_msg)

    def keep_socket_alive(self):
        if self.log_communication:
            prt_str = f"run client mode\n" + "-" * 20 + "\n"
            self.log_com(prt_str)
            
        #
        print("starting socket communication")
        init_obj = {"@message": "initialization"}
        self.format_object_and_send_msg(init_obj)
            
        full_msg = None
        current_msg = ""

        retry = True
        stay_online = True
        while stay_online:
            if self.log_communication:
                prt_str = f"{datetime.datetime.now()}: connection from :{self.socket}\n" + "-" * 20 + "\n"
                self.log_com(prt_str)
            #
            if retry:
                retry = False
                continue
            # TODO # think about error status != 0 in init
            await_response = True
            while await_response:
                # listen to server connection
                byte_stream_msg = self.socket.recv()
                time_now = datetime.datetime.now()
                if time_now - self.last_stat_report_time > datetime.timedelta(seconds=STAT_INT):
                    self.last_stat_report_time = time_now
                    if self.log_communication:
                        prt_str = f"time:{time_now}\ncurrent_msg:{current_msg}\nbyte_stream_msg:{byte_stream_msg}\n" \
                                  + "-" * 20 + "\n"
                        self.log_com(prt_str)
                if not byte_stream_msg:
                    continue
                full_msg = byte_stream_msg.decode(ENCODING)
                if self.log_communication:
                    prt_str = f"{datetime.datetime.now()}: received :{full_msg}\n" + "-" * 20 + "\n"
                    self.log_com(prt_str)
                response_obj = json.loads(full_msg)
                #print("RECEIVED:", response_obj)
                self._treat_matsim_response(response_obj)
                
                if self._simulation_terminated:
                    stay_online = False
                    await_response = False
                    
        self.socket.close()
        self.context.term()
        
        print(" -> Socket closed")
        LOG.info("Socket closed")        
                    
    def _treat_matsim_response(self, response_obj):
        """
        Process the response from MATSim.
        """
        #print("get meassage: ", response_obj["@message"])
        if response_obj["@message"] == "iteration":
            self._new_iteration(response_obj)
        elif response_obj["@message"] == "state":
            new_sim_time = response_obj["time"]
            if self._network_update_interval is not None:
                if (self._last_network_update_time is None) or (new_sim_time - self._last_network_update_time >= self._network_update_interval):
                    LOG.info(f"querry travel time updates at {new_sim_time}")
                    self._last_network_update_time = new_sim_time
                    tt_update_request = {"@message": "travel_time_query", "links": []} # empty list means all links (maybe TODO in the future)
                    self._stored_matsim_response = response_obj
                    self.format_object_and_send_msg(tt_update_request)
                    return
            self._new_state_update(response_obj)
        elif response_obj["@message"] == "travel_time_response":
            self._new_edge_traveltimes(response_obj, self._last_network_update_time)
            if self._stored_matsim_response is not None:
                self._new_state_update(self._stored_matsim_response)
                self._stored_matsim_response = None
        elif response_obj["@message"] == "finalization":
            self._end_simulation(response_obj)
        # elif response_obj["@message"] == "error":
        #     self._handle_error(response_obj)
        else:
            raise KeyError(f"Unknown message type {response_obj['@message']}!")

    def _start_simulation(self, response_obj):
        list_vehicle_attributes = response_obj["vehicle_attributes"]
        
        self._initialize_vehicles(list_vehicle_attributes)
            
        response = {"type": "start_simulation", "status": 0}
        self.format_object_and_send_msg(response)
        
    def _end_simulation(self, response_obj):
        """
        Handle the end of the simulation.
        """
        print(" -> Simulation ended")
        LOG.info("Simulation ended")
        self.fs_obj.terminate()
        self._simulation_terminated = True
        
    def _initialize_vehicles(self, list_vehicle_attributes):
        self.matsim_to_fleetpy_vid = {}
        self.fleetpy_to_matsim_vid = {}
        
        for vehicle_attributes in list_vehicle_attributes:
            matsim_vehicle_id = vehicle_attributes["id"]
            vehicle_start_pos = self.from_matsim_to_fleetpy_position(int(vehicle_attributes["startLink"]))
            vehicle_capacity = int(vehicle_attributes["capacity"])

            vehicle_id = self.fs_obj.add_vehicle(0, vehicle_capacity, vehicle_start_pos[0])
            
            self.matsim_to_fleetpy_vid[matsim_vehicle_id] = vehicle_id
            self.fleetpy_to_matsim_vid[vehicle_id] = matsim_vehicle_id
            
            self._last_veh_state[vehicle_id] = [VRL_STATES.IDLE, [], []]
        
    def _new_iteration(self, response_obj):
        """
        Handle new iteration request from MATSim.
        """
        # end FP simulation
        iteration = int(response_obj["iteration"])
        if iteration > 0:
            self.fs_obj.terminate()
            
            self.fs_obj = None

        
            scenario_parameters = self.scenario_parameters.copy()
            scenario_parameters["matsim_iteration"] = iteration
            
            self.list_op_dicts = build_operator_attribute_dicts(scenario_parameters, scenario_parameters[G_NR_OPERATORS],
                                                                                prefix="op_")
            self.list_ch_op_dicts  = build_operator_attribute_dicts(scenario_parameters, scenario_parameters.get(G_NR_CH_OPERATORS, 0),
                                                                                    prefix="ch_op_")
            
            #self.dir_names = get_directory_dict(scenario_parameters, self.list_op_dicts)
            self.scenario_parameters: dict = scenario_parameters
            
            self.fs_obj = MATSimSimulationClass(self.scenario_parameters)
            self.dir_names = self.fs_obj.dir_names
            
            # create communication log
            self._output_dir = self.fs_obj.dir_names[G_DIR_OUTPUT]
            self.log_f = os.path.join(self._output_dir, "00_socket_com.txt")
            self.last_stat_report_time = datetime.datetime.now()
            with open(self.log_f, "w") as fh_touch:
                fh_touch.write(f"{self.last_stat_report_time}: Opening socket communication ...\n")
                
            level = logging.DEBUG
            logger = logging.getLogger()
            logger.setLevel(level)
            for handler in logger.handlers:
                handler.setLevel(level)
                
            self._simulation_terminated = False
            self._last_network_update_time = None
            self._stored_matsim_response = None # need to store updated state response if network update is requested
            
            self.matsim_to_fleetpy_vid = {}
            self.fleetpy_to_matsim_vid = {}
            
            self.matsim_to_fleetpy_rid = {}
            self.fleetpy_to_matsim_rid = {}
            self._fp_rid_counter = 0
            
            self._last_veh_state = {}   # vid -> [state, [list_current_boarding], [list_current_alighting]]
            # Reset sent-pickup tracking at new iteration
            self._pickup_sent_to_matsim = set()
        
        list_vehicle_attributes = response_obj["vehicles"]
        
        self._initialize_vehicles(list_vehicle_attributes)
        
        self.fs_obj.step(self.scenario_parameters[G_SIM_START_TIME])

        new_assignments = self.fs_obj.get_current_assignments(self.scenario_parameters[G_SIM_START_TIME]) # dict (op_id, vid) -> VehPlan
        
        assignment_message = self._create_assignment_message(new_assignments)
        self.format_object_and_send_msg(assignment_message)
        
    def _new_state_update(self, response_obj):
        """
        Handle new time step request from MATSim.
        """
        new_sim_time = response_obj["time"]
        self.fs_time = new_sim_time  # Track current simulation time for prebooking calculations #new-change
        if new_sim_time % 300 == 0:
            print(" -> new sim time: ", new_sim_time)
            
        force_update = (new_sim_time % self._force_veh_pos_update_interval == 0)
        # LOG.info(f"Socked new state: {new_sim_time}")
        # LOG.info(f"matsim vid to vid: {self.matsim_to_fleetpy_vid}")
        # LOG.info(f"matsim rid to rid: {self.matsim_to_fleetpy_rid}")
        
        picked_up_requests = response_obj["pickedUp"] # dict { "req1": "veh1" }
        dropped_off_requests = response_obj["droppedOff"] # { "req5": "veh10", "req7": "veh12" }
        veh_pick_up_requests = {}
        veh_drop_off_requests = {}
        for rq_id, veh_id in picked_up_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_pick_up_requests[veh_id].append(rq_id)
            except KeyError:
                veh_pick_up_requests[veh_id] = [rq_id]
            # remember as already picked-up to stop listing it in assignments
            try: #new-change (line 324-328)
                self._fp_pickedup_by_vid.setdefault(veh_id, set()).add(rq_id)
                self._fp_current_pickups_by_vid.setdefault(veh_id, set()).discard(rq_id)
            except Exception:
                pass
        for rq_id, veh_id in dropped_off_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_drop_off_requests[veh_id].append(rq_id)
            except KeyError:
                veh_drop_off_requests[veh_id] = [rq_id]
            try: #new-change (line 336-340)
                self._fp_droppedoff_by_vid.setdefault(veh_id, set()).add(rq_id)
                self._fp_current_dropoffs_by_vid.setdefault(veh_id, set()).discard(rq_id)
            except Exception:
                pass
            # Clear prebooking status when passenger is dropped off
            try:
                matsim_rid = self._from_fleetpy_to_matsim_rid(rq_id)
                if veh_id in self._vehicle_prebooking_active and matsim_rid in self._vehicle_prebooking_active[veh_id]:
                    self._vehicle_prebooking_active[veh_id].discard(matsim_rid)
                    LOG.debug(f"[MATSimSocket] Cleared prebooking for vehicle {veh_id}, rid {matsim_rid} after dropoff")
            except Exception:
                pass
                
        picking_up_requests = response_obj["pickingUp"] # dict { "req1": "veh1" }
        dropping_off_requests = response_obj["droppingOff"] # { "req5": "veh10", "req7": "veh12" }
        veh_current_pick_up_requests = {}
        veh_current_drop_off_requests = {}
        for rq_id, veh_id in picking_up_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_current_pick_up_requests[veh_id].append(rq_id)
            except KeyError:
                veh_current_pick_up_requests[veh_id] = [rq_id]
            try:  # new-change (line 353-356)
                self._fp_current_pickups_by_vid.setdefault(veh_id, set()).add(rq_id)
            except Exception:
                pass
        for rq_id, veh_id in dropping_off_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_current_drop_off_requests[veh_id].append(rq_id)
            except KeyError:
                veh_current_drop_off_requests[veh_id] = [rq_id]
            try:  # new-change (line 364-367)
                self._fp_current_dropoffs_by_vid.setdefault(veh_id, set()).add(rq_id)
            except Exception:
                pass
                
        
        list_vehicle_states = response_obj["vehicles"] # list of dicts
        
        for veh_state in list_vehicle_states:
            vid = self.matsim_to_fleetpy_vid[veh_state["id"]]
            
            picked_up = veh_pick_up_requests.get(vid, [])
            dropped_off = veh_drop_off_requests.get(vid, [])
            
            current_pick_up = veh_current_pick_up_requests.get(vid, [])
            current_drop_off = veh_current_drop_off_requests.get(vid, [])
            
            state = self._from_matsim_to_fleetpy_veh_state(veh_state["state"])
            finished_leg_ids = [int(x) for x in veh_state["finished"]]
            
            last_state, last_current_pick_up, last_current_drop_off = self._last_veh_state[vid]
            if force_update or state != last_state or set(sorted(current_pick_up)) != set(sorted(last_current_pick_up)) or set(sorted(current_drop_off)) != set(sorted(last_current_drop_off)):
                LOG.debug(f"veh {vid} state changed from {last_state} to {state} | current pick up from {last_current_pick_up} to {current_pick_up} | current drop off from {last_current_drop_off} to {current_drop_off}")
                self._last_veh_state[vid] = [state, current_pick_up, current_drop_off]
            
                matsim_link = veh_state["currentLink"]
                matsim_link_exit_time = veh_state["currentExitTime"]
                
                if type(matsim_link_exit_time) == str and matsim_link_exit_time == "Infinity":
                    LOG.warning("MATSim diverge link exit time 'Infinity' mapped to LARGE_INT")
                    matsim_link_exit_time = LARGE_INT
                
                veh_pos = self.from_matsim_to_fleetpy_position(matsim_link, remaining_time=matsim_link_exit_time - new_sim_time)
                
                matsim_diverge_link = veh_state["divergeLink"]
                matsim_diverge_link_exit_time = veh_state["divergeTime"]
                if type(matsim_diverge_link_exit_time) == str and matsim_diverge_link_exit_time == "Infinity":
                    LOG.warning("MATSim diverge link exit time 'Infinity' mapped to LARGE_INT")
                    matsim_diverge_link_exit_time = LARGE_INT
                earliest_diverge_pos = self.from_matsim_to_fleetpy_position(matsim_diverge_link)
                earliest_diverge_time = matsim_diverge_link_exit_time
                
                self.fs_obj.update_veh_state(new_sim_time, vid, 0, veh_pos, picked_up, dropped_off, state, earliest_diverge_pos, earliest_diverge_time, finished_leg_ids,
                                            current_pick_up, current_drop_off)
                # (reverted) do not record currentLink here
        
        list_requests = response_obj["submitted"] # list of dicts
        #print(" -> number of new requests: ", len(list_requests))
        for rq_entry in list_requests:
            org_pos = self.from_matsim_to_fleetpy_position(int(rq_entry["originLink"]))
            org_str = f"{org_pos[0]};{org_pos[1]};{org_pos[2]}"
            dest_pos = self.from_matsim_to_fleetpy_position(int(rq_entry["destinationLink"]))
            dest_str = f"{dest_pos[0]};{dest_pos[1]};{dest_pos[2]}"
            if int(rq_entry["originLink"]) in self._non_unique_matsim_links or int(rq_entry["destinationLink"]) in self._non_unique_matsim_links:
                LOG.warning(f"Request {rq_entry['id']} has origin or destination on non-uniquely mapped link! -> set for automatic decline")
                dest_str = org_str  # just to have a valid destination, will be declined anyway
            fp_rid = self._from_matsim_to_fleetpy_rid(rq_entry["id"])
            # Remember exact MATSim origin/destination links per MATSim rid for precise prebooking
            try:
                self._rid_to_matsim_origin[rq_entry["id"]] = int(rq_entry["originLink"])  # e.g., "drt_4" -> 48145
                self._rid_to_matsim_destination[rq_entry["id"]] = int(rq_entry["destinationLink"])  # e.g., "drt_4" -> 5577
            except Exception:
                pass
            rq_info_dict = {G_RQ_ID: fp_rid,
                            G_RQ_ORIGIN: org_str, 
                            G_RQ_DESTINATION: dest_str, 
                            G_RQ_TIME: new_sim_time,
                            G_RQ_EPT: int(rq_entry["earliestPickupTime"]), # TODO optional
                            G_RQ_LPT: int(rq_entry["latestPickupTime"]), # TODO optional
                            G_RQ_LDT: new_sim_time + self.scenario_parameters[G_AR_MAX_DEC_T], 
                            G_RQ_PAX: int(rq_entry["size"])} # TODO to add?
            rq_series = pd.Series(rq_info_dict)
            rq_series.name = rq_info_dict[G_RQ_ID]
            self.fs_obj.add_request(rq_series)
            # (reverted) do not store request time windows here
            
        self.fs_obj.step(new_sim_time)
        
        new_assignments = self.fs_obj.get_current_assignments(new_sim_time) # dict (op_id, vid) -> VehPlan
        
        assignment_message = self._create_assignment_message(new_assignments)
        self.format_object_and_send_msg(assignment_message)
    
    def _create_assignment_message(self, new_assignments: Dict[Any, List[dict]]):
        """
        Create a message with the new assignments for MATSim.
        """
        # Increase waitFor to allow more time for prebooking (configurable)
        assignment_message = {"@message": "assignment", "stops": {}, "waitFor": float(self.scenario_parameters.get("matsim_wait_for", 120.0))} #new-change

        for (op_id, veh_id), stop_list in new_assignments.items():
            matsim_vehicle_id = self.fleetpy_to_matsim_vid[veh_id]
            
            # Skip assignment updates if vehicle has active prebooking - don't override MATSim's prebooking plan
            if veh_id in self._vehicle_prebooking_active and len(self._vehicle_prebooking_active[veh_id]) > 0:
                LOG.debug(f"[MATSimSocket] *** SKIPPING assignment for vehicle {matsim_vehicle_id} (fp_vid {veh_id}) - active prebooking for {self._vehicle_prebooking_active[veh_id]} ***")
                continue
            elif veh_id in self._vehicle_prebooking_active:
                LOG.debug(f"[MATSimSocket] Vehicle {veh_id} has empty prebooking set: {self._vehicle_prebooking_active[veh_id]}")
            else:
                LOG.debug(f"[MATSimSocket] Vehicle {veh_id} not in prebooking dict (keys: {list(self._vehicle_prebooking_active.keys())})")
            
            # Extract pax_info from the vehicle plan to get original pickup times
            veh_obj = self.fs_obj.sim_vehicles.get((op_id, veh_id)) #new-change (line 442-424)
            veh_plan = self.fs_obj.operators[op_id].veh_plans.get(veh_id)
            pax_info = getattr(veh_plan, 'pax_info', {}) if veh_plan else {}
            
            # Debug logging for RideSync
            if len(stop_list) > 0:
                LOG.debug(f"[MATSimSocket] Processing {len(stop_list)} stops for vehicle {matsim_vehicle_id}")
                for i, s in enumerate(stop_list):
                    LOG.debug(f"  Stop {i}: pos={s.get('pos')}, boarding={s.get('boarding_rids')}, alighting={s.get('alighting_rids')}")
            
            list_stops = []
            for idx, stop in enumerate(stop_list): #new-change (line 418-441)
                pos = stop["pos"]
                matsim_edge = None
                # If position is on a node, try mapping using next/previous node to select a concrete edge
                if pos[1] is None:
                    try:
                        # Forward scan to find the next stop with a different node to infer direction
                        if matsim_edge is None: #new-change (line 434-454)
                            j = idx + 1
                            while j < len(stop_list):
                                next_pos = stop_list[j]["pos"]
                                if next_pos and next_pos[0] != pos[0]:
                                    next_node = next_pos[0]
                                    matsim_edge = self.fp_edge_to_matsim_edge.get(pos[0], {}).get(next_node)
                                    if matsim_edge is not None:
                                        break
                                j += 1
                        # Backward scan to find the previous stop with a different node
                        if matsim_edge is None:
                            j = idx - 1
                            while j >= 0:
                                prev_pos = stop_list[j]["pos"]
                                if prev_pos and prev_pos[0] != pos[0]:
                                    prev_node = prev_pos[0]
                                    matsim_edge = self.fp_edge_to_matsim_edge.get(prev_node, {}).get(pos[0])
                                    if matsim_edge is not None:
                                        break
                                j -= 1
                    except Exception:
                        matsim_edge = None
                if matsim_edge is None:
                    try:
                        matsim_edge = self.from_fleetpy_to_matsim_position(pos)
                    except Exception:
                        matsim_edge = None
                if matsim_edge is None:
                    # Log warning for debugging RideSync issues
                    LOG.warning(f"Could not map FleetPy position {pos} to MATSim link for stop {idx} - skipping this stop!")
                    LOG.warning(f"  Stop details: boarding={fp_boarding_rids}, alighting={stop.get('alighting_rids')}")
                    # Skip invalid stops rather than sending an unknown link to MATSim
                    continue

                fp_boarding_rids = list(stop["boarding_rids"]) if stop.get("boarding_rids") is not None else []
                list_pick_up = [self._from_fleetpy_to_matsim_rid(rid) for rid in fp_boarding_rids]
                # Suppress pickups already sent to MATSim to avoid duplicate scheduling
                try:
                    if list_pick_up:
                        list_pick_up = [rid for rid in list_pick_up if rid not in self._pickup_sent_to_matsim]
                except Exception:
                    pass
                list_drop_off = [self._from_fleetpy_to_matsim_rid(rid) for rid in stop["alighting_rids"]]

                # Only emit stops that actually perform pickup/dropoff
                if len(list_pick_up) == 0 and len(list_drop_off) == 0: #new-change (line 447-458)
                    LOG.debug(f"  Stop {idx} has no pickups or dropoffs after filtering, skipping")
                    continue

                LOG.debug(f"  Stop {idx} will be sent: link={matsim_edge}, pickup={list_pick_up}, dropoff={list_drop_off}")

                # Use the bus stop link derived from the stop position (aligns with Roman implementation)

                # Normalize fields for MATSim 
                stop_duration_val = stop["duration"] if stop["duration"] is not None else 0
                try:
                    stop_duration_val = int(stop_duration_val)
                except (ValueError, TypeError):
                    stop_duration_val = 0
                if stop_duration_val <= 0:
                    stop_duration_val = 1
                earliest_start_time = stop["earliest_start_time"]
                stop_id_val = stop["id"] #new-change (line 460-470)
                stop_id_str = str(stop_id_val) if stop_id_val is not None else None

                entry = {
                    "link": matsim_edge,
                    "pickup": list_pick_up,
                    "dropoff": list_drop_off,
                    "stopDuration": stop_duration_val,
                }
                pickup_time_set = False
                # Use a stable synthetic id that does not change across re-optimizations
                # This prevents MATSim from losing the prebooking if internal plan ids shift
                # Use a stable numeric id per (veh, type, rid set, link) to keep ids constant across resends
                try:
                    fp_vid = self.matsim_to_fleetpy_vid.get(matsim_vehicle_id)
                except Exception:
                    fp_vid = None
                stop_type = 1 if len(list_pick_up) > 0 else (-1 if len(list_drop_off) > 0 else 0)
                rid_key = tuple(sorted(list_pick_up if stop_type == 1 else list_drop_off))
                if fp_vid is not None and stop_type != 0:
                    key = (fp_vid, stop_type, rid_key, matsim_edge)
                    sid = self._stable_stop_id.get(key)
                    if sid is None:
                        sid = self._stop_id_counter
                        self._stable_stop_id[key] = sid
                        self._stop_id_counter += 1
                    entry["id"] = sid
                elif stop_id_val is not None:
                    try:
                        entry["id"] = int(stop_id_val)
                    except Exception:
                        pass
                # Do NOT include earliestStartTime for pickup stops (align with working minimal behavior)
                if len(fp_boarding_rids) == 0 and earliest_start_time is not None and earliest_start_time > 0:
                    # For non-pickup stops, keep provided earliest_start_time if present
                    try:
                        entry["earliestStartTime"] = int(earliest_start_time)
                    except (ValueError, TypeError):
                        pass
                # Skip emitting if this rid is already picked up/dropped off on this vehicle
                try: #new-change (line 602-622)
                    fp_vid = self.matsim_to_fleetpy_vid.get(matsim_vehicle_id)
                except Exception:
                    fp_vid = None
                if fp_vid is not None:
                    try:
                        if len(list_pick_up) > 0:
                            # Drop rids as soon as MATSim starts pickingUp OR after pickedUp to avoid duplicate assignments
                            list_pick_up = [rid for rid in list_pick_up if (
                                self._from_matsim_to_fleetpy_rid(rid) not in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                and self._from_matsim_to_fleetpy_rid(rid) not in self._fp_pickedup_by_vid.get(fp_vid, set())
                            )]
                            entry["pickup"] = list_pick_up
                    except Exception:
                        pass
                    try:
                        if len(list_drop_off) > 0:
                            # Check if we're using RideSync fleet control (where prebooking sends both pickup and dropoff together)
                            is_ridesync = False
                            
                            # Direct flag for forcing RideSync behavior
                            if self.scenario_parameters.get("force_ridesync_mode", False):
                                is_ridesync = True
                                LOG.debug(f"  RideSync forced via force_ridesync_mode flag")
                            
                            try:
                                # Check multiple possible parameter names for fleet control algorithm
                                if not is_ridesync:
                                    for param_name in ["op_module", "op_0_module", "op_fleetctrl_alg", "op_0_fleetctrl_alg", 
                                                      "fleetctrl_alg", "op_fleetctrl"]:
                                        fleetctrl_alg = self.scenario_parameters.get(param_name, "")
                                        if fleetctrl_alg:
                                            LOG.debug(f"  Checking RideSync: {param_name} = '{fleetctrl_alg}'")
                                            if "RideSync" in fleetctrl_alg:
                                                is_ridesync = True
                                                LOG.debug(f"  RideSync detected via scenario parameter {param_name}")
                                                break
                                
                                if not is_ridesync:
                                    # Check operator attributes
                                    for i, op_dict in enumerate(self.list_op_dicts):
                                        # Check various keys that might contain the fleet control module name
                                        for key in ["module", "fleetctrl", "fleetctrl_alg", "type", "op_type"]:
                                            val = op_dict.get(key, "")
                                            if "RideSync" in str(val):
                                                is_ridesync = True
                                                LOG.debug(f"  RideSync detected via operator dict key '{key}' = '{val}'")
                                                break
                                        if is_ridesync:
                                            break
                                
                                if not is_ridesync:
                                    # Also check operator class name as fallback
                                    if hasattr(self.fs_obj, 'operators'):
                                        operators = self.fs_obj.operators
                                        if isinstance(operators, dict):
                                            for op_id, op in operators.items():
                                                if "RideSync" in op.__class__.__name__:
                                                    is_ridesync = True
                                                    LOG.debug(f"  RideSync detected via operator class: {op.__class__.__name__}")
                                                    break
                                        elif isinstance(operators, list):
                                            for op in operators:
                                                if "RideSync" in op.__class__.__name__:
                                                    is_ridesync = True
                                                    LOG.debug(f"  RideSync detected via operator class: {op.__class__.__name__}")
                                                    break
                            except Exception as e:
                                LOG.debug(f"  Exception during RideSync detection: {e}")
                                pass
                            
                            # Check if this is a prebooking (pickup far in the future)
                            is_prebooking = False
                            try:
                                # If earliest_start_time is far in the future, treat as prebooking
                                earliest_start = stop.get("earliest_start_time")
                                current_time = getattr(self.fs_obj, 'sim_time', 0)
                                LOG.debug(f"  Prebooking check: earliest_start={earliest_start}, current_time={current_time}")
                                if earliest_start and earliest_start > 0 and earliest_start - current_time > 1800:  # More than 30 minutes in future
                                    is_prebooking = True
                                    LOG.debug(f"  Prebooking detected: pickup at {earliest_start}, current time {current_time}")
                            except Exception as e:
                                LOG.debug(f"  Exception during prebooking detection: {e}")
                            
                            if not is_ridesync and not is_prebooking:
                                # Gate dropoffs: only send after pickup has started or completed on this vehicle
                                original_len = len(list_drop_off)
                                list_drop_off = [rid for rid in list_drop_off if (
                                    self._from_matsim_to_fleetpy_rid(rid) in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                    or self._from_matsim_to_fleetpy_rid(rid) in self._fp_pickedup_by_vid.get(fp_vid, set())
                                )]
                                if original_len != len(list_drop_off):
                                    LOG.debug(f"  Gated dropoffs from {original_len} to {len(list_drop_off)} (not RideSync/prebooking)")
                            else:
                                LOG.debug(f"  RideSync/prebooking detected - not gating {len(list_drop_off)} dropoffs")
                            # For RideSync, send all dropoffs without gating
                            entry["dropoff"] = list_drop_off
                    except Exception:
                        pass
                # If this is a pickup and we did not set earliestStartTime from pax_info, ensure it is absent
                try:
                    if len(list_pick_up) > 0 and "earliestStartTime" in entry:
                        del entry["earliestStartTime"]
                except Exception:
                    pass
                # (reverted) do not override link to currentLink here
                # After filtering, add only if still actionable
                if len(entry["pickup"]) == 0 and len(entry["dropoff"]) == 0:
                    pass
                else:
                    # For RideSync: If this stop has BOTH pickup and dropoff for the same request,
                    # split into two separate stops (MATSim expects separate stops)
                    is_ridesync = False
                    try:
                        # Check multiple possible parameter names for fleet control algorithm
                        for param_name in ["op_module", "op_0_module", "op_fleetctrl_alg", "op_0_fleetctrl_alg",
                                          "fleetctrl_alg", "op_fleetctrl"]:
                            fleetctrl_alg = self.scenario_parameters.get(param_name, "")
                            if fleetctrl_alg and "RideSync" in fleetctrl_alg:
                                is_ridesync = True
                                break
                        
                        if not is_ridesync:
                            # Check operator attributes
                            for op_dict in self.list_op_dicts:
                                # Check various keys that might contain the fleet control module name
                                for key in ["module", "fleetctrl", "fleetctrl_alg", "type"]:
                                    if "RideSync" in str(op_dict.get(key, "")):
                                        is_ridesync = True
                                        break
                                if is_ridesync:
                                    break
                        
                        if not is_ridesync:
                            # Also check operator class name as fallback
                            if hasattr(self.fs_obj, 'operators'):
                                operators = self.fs_obj.operators
                                if isinstance(operators, dict):
                                    for op in operators.values():
                                        if "RideSync" in op.__class__.__name__:
                                            is_ridesync = True
                                            break
                                elif isinstance(operators, list):
                                    for op in operators:
                                        if "RideSync" in op.__class__.__name__:
                                            is_ridesync = True
                                            break
                    except Exception:
                        pass
                    
                    if is_ridesync and len(entry["pickup"]) > 0 and len(entry["dropoff"]) > 0:
                        # Check if it's the same request
                        same_request = False
                        for p_rid in entry["pickup"]:
                            if p_rid in entry["dropoff"]:
                                same_request = True
                                break
                        
                        if same_request:
                            # Split into two stops: one for pickup, one for dropoff
                            pickup_entry = entry.copy()
                            pickup_entry["dropoff"] = []
                            pickup_entry["id"] = entry.get("id", self._stop_id_counter)
                            self._stop_id_counter += 1
                            
                            dropoff_entry = entry.copy()
                            dropoff_entry["pickup"] = []
                            dropoff_entry["id"] = entry.get("id", self._stop_id_counter) + 1000  # Different ID
                            self._stop_id_counter += 1
                            
                            # Mark pickups as sent
                            try:
                                for rid in pickup_entry.get("pickup", []):
                                    self._pickup_sent_to_matsim.add(rid)
                            except Exception:
                                pass
                            
                            list_stops.append(pickup_entry)
                            list_stops.append(dropoff_entry)
                            LOG.debug(f"  Split RideSync stop into separate pickup and dropoff stops")
                        else:
                            # Different requests, keep as is
                            try:
                                for rid in entry.get("pickup", []):
                                    self._pickup_sent_to_matsim.add(rid)
                            except Exception:
                                pass
                            list_stops.append(entry)
                    else:
                        # Normal case: add the stop as is
                        # Mark pickups as sent so we never send them again
                        try:
                            for rid in entry.get("pickup", []):
                                self._pickup_sent_to_matsim.add(rid)
                        except Exception:
                            pass
                        list_stops.append(entry) #new-change
            # If we computed no actionable stops, reuse last non-empty assignment to keep MATSim prebooking intact
            if list_stops: #new-change (line 480-487)
                LOG.debug(f"[MATSimSocket] Sending {len(list_stops)} stops for vehicle {matsim_vehicle_id}")
                for i, stop in enumerate(list_stops):
                    LOG.debug(f"    Final stop {i}: link={stop.get('link')}, pickup={stop.get('pickup', [])}, dropoff={stop.get('dropoff', [])}")
                
                # Track prebookings: if we're sending both pickup and dropoff for same request, mark as prebooking
                try:
                    pickups_in_msg = set()
                    dropoffs_in_msg = set()
                    for stop in list_stops:
                        pickups_in_msg.update(stop.get('pickup', []))
                        dropoffs_in_msg.update(stop.get('dropoff', []))
                    # If any rid has both pickup and dropoff in this message, it's a prebooking
                    prebooking_rids = pickups_in_msg & dropoffs_in_msg
                    if prebooking_rids and veh_id is not None:
                        if veh_id not in self._vehicle_prebooking_active:
                            self._vehicle_prebooking_active[veh_id] = set()
                        self._vehicle_prebooking_active[veh_id].update(prebooking_rids)
                        LOG.debug(f"[MATSimSocket] Marked prebooking active for vehicle {veh_id} (MATSim: {matsim_vehicle_id}), rids: {prebooking_rids}")
                except Exception as e:
                    LOG.debug(f"[MATSimSocket] Exception marking prebooking: {e}")
                    pass
                
                assignment_message["stops"][matsim_vehicle_id] = list_stops
                # cache
                self._last_assignment_by_vid[matsim_vehicle_id] = list_stops
            else:
                cached = self._last_assignment_by_vid.get(matsim_vehicle_id)
                if cached:
                    # Filter cached to drop pickups if already picking up / picked up
                    try: #new-change (line 623-645)
                        fp_vid = self.matsim_to_fleetpy_vid.get(vid_cached)
                    except Exception:
                        fp_vid = None
                    if fp_vid is not None:
                        filtered_cached = []
                        for entry in cached:
                            try:
                                pickup_list = list(entry.get("pickup", []))
                                drop_list = list(entry.get("dropoff", []))
                                if len(pickup_list) > 0:
                                    # Never re-send pickups already sent
                                    pickup_list = [rid for rid in pickup_list if rid not in self._pickup_sent_to_matsim]
                                    # Remove as soon as pickingUp OR pickedUp
                                    pickup_list = [rid for rid in pickup_list if (
                                        self._from_matsim_to_fleetpy_rid(rid) not in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                        and self._from_matsim_to_fleetpy_rid(rid) not in self._fp_pickedup_by_vid.get(fp_vid, set())
                                    )]
                                    entry = dict(entry)
                                    entry["pickup"] = pickup_list
                                if len(drop_list) > 0:
                                    # Gate dropoffs: only send after pickup has started or completed
                                    drop_list = [rid for rid in drop_list if (
                                        self._from_matsim_to_fleetpy_rid(rid) in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                        or self._from_matsim_to_fleetpy_rid(rid) in self._fp_pickedup_by_vid.get(fp_vid, set())
                                    )]
                                    entry = dict(entry)
                                    entry["dropoff"] = drop_list
                                if len(pickup_list) == 0 and len(drop_list) == 0:
                                    continue
                            except Exception:
                                pass
                            filtered_cached.append(entry)
                        if filtered_cached:
                            assignment_message["stops"][matsim_vehicle_id] = filtered_cached
                    else:
                        assignment_message["stops"][matsim_vehicle_id] = cached

        # If some vehicles had cached assignments but no new entries were produced (or vehicle missing in new_assignments),
        # keep sending cached stops to preserve MATSim prebookings until they are consumed.
        try: #new-change (line 559-564)
            for vid_cached, cached_stops in self._last_assignment_by_vid.items():
                if vid_cached not in assignment_message["stops"] and cached_stops:
                    # Filter cached similarly to avoid resending pickups during pickingUp/pickedUp
                    try: #new-change (line 654-676)
                        fp_vid = self.matsim_to_fleetpy_vid.get(vid_cached)
                    except Exception:
                        fp_vid = None
                    if fp_vid is not None:
                        filtered_cached = []
                        for entry in cached_stops:
                            try:
                                pickup_list = list(entry.get("pickup", []))
                                drop_list = list(entry.get("dropoff", []))
                                if len(pickup_list) > 0:
                                    # Never re-send pickups already sent
                                    pickup_list = [rid for rid in pickup_list if rid not in self._pickup_sent_to_matsim]
                                    # Remove as soon as pickingUp OR pickedUp
                                    pickup_list = [rid for rid in pickup_list if (
                                        self._from_matsim_to_fleetpy_rid(rid) not in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                        and self._from_matsim_to_fleetpy_rid(rid) not in self._fp_pickedup_by_vid.get(fp_vid, set())
                                    )]
                                    entry = dict(entry)
                                    entry["pickup"] = pickup_list
                                if len(drop_list) > 0:
                                    # Gate dropoffs: only send after pickup has started or completed
                                    drop_list = [rid for rid in drop_list if (
                                        self._from_matsim_to_fleetpy_rid(rid) in self._fp_current_pickups_by_vid.get(fp_vid, set())
                                        or self._from_matsim_to_fleetpy_rid(rid) in self._fp_pickedup_by_vid.get(fp_vid, set())
                                    )]
                                    entry = dict(entry)
                                    entry["dropoff"] = drop_list
                                if len(pickup_list) == 0 and len(drop_list) == 0:
                                    continue
                            except Exception:
                                pass
                            filtered_cached.append(entry)
                        if filtered_cached:
                            assignment_message["stops"][vid_cached] = filtered_cached
                    else:
                        assignment_message["stops"][vid_cached] = cached_stops
        except Exception:
            pass
            
        return assignment_message    

    def _create_fleetpy_network(self, matsim_network_path):
        """
        Create FleetPy network based on MATSim network.
        """
        # Example conversion logic (to be replaced with actual logic)
        fleetpy_data_path = self.dir_names[G_DIR_DATA]
        network_name = self.scenario_parameters[G_NETWORK_NAME]
        matsim_edge_to_fp_edge, fp_edge_to_matsim_edge = create_fleetpy_network_from_matsim(matsim_network_path, fleetpy_data_path, network_name)     
        return matsim_edge_to_fp_edge, fp_edge_to_matsim_edge
    
    def _from_matsim_to_fleetpy_veh_state(self, state_str):
        if state_str == "drive":
            return VRL_STATES.ROUTE
        elif state_str == "stop":
            return VRL_STATES.BOARDING
        elif state_str == "stay":
            return VRL_STATES.IDLE
        elif state_str == "inactive":
            LOG.warning("MATSim vehicle state 'inactive' mapped to FleetPy state 'IDLE'")
            return VRL_STATES.IDLE
        else:
            raise KeyError(f"Unknown matsim vehicle state {state_str}!")
        
    def _from_matsim_to_fleetpy_rid(self, matsim_rid):
        fleetpy_rid = self.matsim_to_fleetpy_rid.get(matsim_rid)
        if fleetpy_rid is None:
            fleetpy_rid = self._fp_rid_counter
            self.matsim_to_fleetpy_rid[matsim_rid] = fleetpy_rid
            self.fleetpy_to_matsim_rid[fleetpy_rid] = matsim_rid
            self._fp_rid_counter += 1
        return fleetpy_rid
    
    def _from_fleetpy_to_matsim_rid(self, fleetpy_rid):
        return self.fleetpy_to_matsim_rid[fleetpy_rid]
            
    def from_matsim_to_fleetpy_position(self, matsim_link, remaining_time=None):
        """
        Convert MATSim position to FleetPy position.
        """
        if remaining_time is None:
            fp_edge = self.matsim_edge_to_fp_edge[int(matsim_link)]
            return (fp_edge[0], fp_edge[1], 1.0)  # at the end of the edge
        elif type(remaining_time) == str and remaining_time == "Infinity":
            LOG.warning("MATSim position with remaining_time 'Infinity' mapped to position at the start of the edge")
            fp_edge = self.matsim_edge_to_fp_edge[int(matsim_link)]
            return (fp_edge[0], fp_edge[1], 0.0)  # at the start of the edge
        else:
            #print("WARNING MATSimSocket: remaining_time is not None, but not implemented yet")
            fp_edge = self.matsim_edge_to_fp_edge[int(matsim_link)]
            start_node, end_node = fp_edge
            tt, _ = self.fs_obj.routing_engine.get_section_infos(start_node, end_node)
            frac = 1 - remaining_time / tt
            #print(f"matsim to fleetpy pos: {matsim_link} {fp_edge} {tt} {remaining_time} -> {frac} -> {max(min(frac, 1), 0)}")
            return (start_node, end_node, max(min(frac, 1), 0))
    
    def from_fleetpy_to_matsim_position(self, fleetpy_position):
        """
        Convert FleetPy position to MATSim position.
        """
        # TODO think about this
        if fleetpy_position[-1] is None:
            # For node-only positions, try to find the most suitable edge
            # First check if this node has only one outgoing edge (common for bus stops)
            node_id = fleetpy_position[0]
            if node_id not in self.fp_edge_to_matsim_edge:
                LOG.warning(f"Node {node_id} not found in fp_edge_to_matsim_edge mapping")
                return None
                
            outgoing_edges = self.fp_edge_to_matsim_edge[node_id]
            if len(outgoing_edges) == 1:
                # Only one outgoing edge, use it
                any_target = list(outgoing_edges.keys())[0]
                matsim_edge = outgoing_edges[any_target]
            else:
                # Multiple outgoing edges - for RideSync, log which one we're choosing
                LOG.warning(f"fleetpy position is on node {node_id} with {len(outgoing_edges)} outgoing edges, picking first one")
                any_target = list(outgoing_edges.keys())[0]
                matsim_edge = outgoing_edges[any_target]
                LOG.debug(f"  Available edges from node {node_id}: {list(outgoing_edges.keys())}")
                LOG.debug(f"  Chose edge to node {any_target} -> MATSim link {matsim_edge}")
            return matsim_edge
        else:
            # Position is on an edge
            if fleetpy_position[0] not in self.fp_edge_to_matsim_edge:
                LOG.warning(f"Edge start node {fleetpy_position[0]} not found in mapping")
                return None
            if fleetpy_position[1] not in self.fp_edge_to_matsim_edge[fleetpy_position[0]]:
                LOG.warning(f"Edge {fleetpy_position[0]}->{fleetpy_position[1]} not found in mapping")
                return None
            matsim_edge = self.fp_edge_to_matsim_edge[fleetpy_position[0]][fleetpy_position[1]]
            return matsim_edge
    
    def from_matsim_to_fleetpy_route(self, matsim_route):
        """
        Convert MATSim route to FleetPy route.
        """
        # Example conversion logic (to be replaced with actual logic)
        raise NotImplementedError("from_matsim_to_fleetpy_route is not implemented yet")
    
    def from_fleetpy_to_matsim_route(self, fleetpy_route):
        """
        Convert FleetPy route to MATSim route.
        """
        # Example conversion logic (to be replaced with actual logic)
        matsim_route = []
        for i in range(len(fleetpy_route) - 1):
            matsim_edge = self.fp_edge_to_matsim_edge[fleetpy_route[i]][fleetpy_route[i + 1]]
            matsim_route.append(matsim_edge)
        return matsim_route
    
    def _new_edge_traveltimes(self, response_obj, sim_time):
        """
        Update edge travel times based on MATSim response.
        """
        list_link_times = response_obj["travelTimes"]
        edge_tt_df_list = []
        for matsim_link, travel_time in list_link_times.items():
            fp_edge = self.matsim_edge_to_fp_edge[int(matsim_link)]
            edge_tt_df_list.append({
                "from_node" : fp_edge[0],
                "to_node" : fp_edge[1],
                "edge_tt" : travel_time
            })
        tt_f_p = os.path.join(self._output_dir, f"matsim_edge_traveltimes_{int(sim_time)}.csv")
        pd.DataFrame(edge_tt_df_list).to_csv(tt_f_p, index=False)
        self.fs_obj.routing_engine.load_tt_file(sim_time, ext_path=tt_f_p)   
    
    
if __name__ == "__main__":
    # Example usage of MATSimSocket class
    scenario_parameters = {}
    host = "localhost"
    port = 1234
    
    from src.misc.config import ConstantConfig, ScenarioConfig
    
    #matsim_network_path = r"C:\Users\ge37ser\Documents\Projekte\MINGA\AP5\IRTSystemX\KopplungMATSimFleetPy\matsim-fleetpy\scenario\network.xml.gz"
    
    matsim_network_path = r"C:\Users\ge37ser\Documents\Projekte\MINGA\AP5\IRTSystemX\KopplungMATSimFleetPy\MATSim Populations\muenchen_1pct\test_cut\cut_network.xml.gz"
    
    const_cfg = ConstantConfig(r"C:\Users\ge37ser\Documents\Coding\FleetPy\studies\test_matsim_coupling\scenarios\constant_config_pool.csv")
    print(const_cfg)
    scenarios_cfg = ScenarioConfig(r"C:\Users\ge37ser\Documents\Coding\FleetPy\studies\test_matsim_coupling\scenarios\example_pool.csv")
    print(scenarios_cfg)
    
    whole_config = const_cfg + scenarios_cfg[0]
    whole_config["matsim_network_path"] = matsim_network_path
    whole_config["study_name"] = "test_matsim_coupling"
    whole_config["log_level"] = "info"
    whole_config["n_cpu_per_sim"] = 1
    whole_config["force_veh_pos_update_interval"] = 15
    
    profile = False
    
    if not profile:
        matsim_socket = MATSimSocket(host, port, whole_config, log_communication=LOG_COMMUNICATION)
        matsim_socket.keep_socket_alive()
    
    else:
        import sys
        import cProfile, pstats, io
        
        profiler = cProfile.Profile()
        profiler.enable()
        
        try:
            matsim_socket = MATSimSocket(host, port, whole_config, log_communication=LOG_COMMUNICATION)
            matsim_socket.keep_socket_alive()
            
        finally:
            profiler.disable()
            s = io.StringIO()
            #sortby = SortKey.CUMULATIVE
            ps = pstats.Stats(profiler, stream=s).sort_stats('cumtime') #tottime cumtime
            ps.print_stats(150)
            print(s.getvalue())