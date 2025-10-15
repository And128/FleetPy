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
        # Force SlaveRequest for MATSim coupling
        self.scenario_parameters["rq_type"] = "SlaveRequest"
        
        self.matsim_edge_to_fp_edge, self.fp_edge_to_matsim_edge = self._create_fleetpy_network(scenario_parameters["matsim_network_path"])
        # check unique mapping
        self._non_unique_matsim_links = []
        for matsim_link, fp_edge in self.matsim_edge_to_fp_edge.items():
            rev_matsim_link = self.fp_edge_to_matsim_edge[fp_edge[0]][fp_edge[1]]
            if rev_matsim_link != matsim_link:
                LOG.warning(f"Mapping between MATSim and FleetPy edges is not unique! {matsim_link} -> {fp_edge} -> {rev_matsim_link}")
                self._non_unique_matsim_links.append(matsim_link)
        # feature flags (default off to avoid behavior changes)
        self._validate_stops_on_start = bool(self.scenario_parameters.get("matsim_validate_stops_on_start", False))
        self._prefer_incoming_node_edge = bool(self.scenario_parameters.get("matsim_prefer_incoming_for_node_positions", False))
        self._skip_unknown_assignment_links = bool(self.scenario_parameters.get("matsim_skip_unknown_links", False))
        self._force_start_on_current_link = bool(self.scenario_parameters.get("matsim_force_start_on_current_link", False))
        # optional: validate RideSync all_stops.csv against built network
        if self._validate_stops_on_start:
            try:
                self._validate_ridesync_stops_against_network()
            except Exception as e:
                LOG.debug(f"RideSync stops validation skipped/failed: {e}")
        
        self.matsim_to_fleetpy_vid = {}
        self.fleetpy_to_matsim_vid = {}
        
        self.matsim_to_fleetpy_rid = {}
        self.fleetpy_to_matsim_rid = {}
        self._fp_rid_counter = 0
        # track last seen MATSim link per FleetPy vehicle id for diagnostics
        self._last_matsim_link_by_fp_vid = {}
        
        # Ensure SlaveRequest is used
        scenario_parameters["rq_type"] = "SlaveRequest"
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
        print("get meassage: ", response_obj["@message"])
        if response_obj["@message"] == "iteration":
            self._new_iteration(response_obj)
        elif response_obj["@message"] == "state":
            new_sim_time = response_obj["time"]
            # Cast time to float if it arrives as a string to avoid type issues
            if isinstance(new_sim_time, str): #new-change
                try:
                    new_sim_time = float(new_sim_time)
                except Exception:
                    LOG.warning(f"Unexpected time format in response: {new_sim_time}")
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
        
    def _new_iteration(self, response_obj):
        """
        Handle new iteration request from MATSim.
        """
        # end FP simulation
        if self.matsim_iteration > 0:
            self.fs_obj.terminate()
        
            self.matsim_iteration = response_obj["iteration"]
            self.scenario_parameters["matsim_iteration"] = self.matsim_iteration
            # Ensure SlaveRequest is used
            self.scenario_parameters["rq_type"] = "SlaveRequest"
            self.fs_obj = MATSimSimulationClass(self.scenario_parameters)
            self.fs_obj.dir_names = self.dir_names
        
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
        # Ensure time is numeric if it arrives as a string
        if isinstance(new_sim_time, str): #new-change
            try:
                new_sim_time = float(new_sim_time)
            except Exception:
                LOG.warning(f"Unexpected time format: {new_sim_time} -> defaulting to 0.0")
                new_sim_time = 0.0
        print(" -> new sim time: ", new_sim_time)
        LOG.info(f"Socked new state: {new_sim_time}")
        LOG.info(f"matsim vid to vid: {self.matsim_to_fleetpy_vid}")
        LOG.info(f"matsim rid to rid: {self.matsim_to_fleetpy_rid}")
        
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
        for rq_id, veh_id in dropped_off_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_drop_off_requests[veh_id].append(rq_id)
            except KeyError:
                veh_drop_off_requests[veh_id] = [rq_id]
                
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
        for rq_id, veh_id in dropping_off_requests.items():
            rq_id = self._from_matsim_to_fleetpy_rid(rq_id)
            veh_id = self.matsim_to_fleetpy_vid[veh_id]
            try:
                veh_current_drop_off_requests[veh_id].append(rq_id)
            except KeyError:
                veh_current_drop_off_requests[veh_id] = [rq_id]
                
        
        list_vehicle_states = response_obj["vehicles"] # list of dicts
        
        for veh_state in list_vehicle_states: #updated-change
            vid = self.matsim_to_fleetpy_vid[veh_state["id"]]
            matsim_link = veh_state["currentLink"]
            matsim_link_exit_time = veh_state["currentExitTime"]
            # remember last MATSim link for this FleetPy vid
            try:
                self._last_matsim_link_by_fp_vid[vid] = matsim_link
            except Exception:
                pass
            # Compute remaining time robustly; currentExitTime may be a string (including "Infinity")
            remaining_time = None
            if matsim_link_exit_time is None:
                remaining_time = "Infinity"
            elif isinstance(matsim_link_exit_time, str):
                if matsim_link_exit_time == "Infinity":
                    remaining_time = "Infinity"
                else:
                    try:
                        matsim_link_exit_time = float(matsim_link_exit_time)
                    except Exception:
                        LOG.warning(f"Unexpected currentExitTime format: {matsim_link_exit_time} -> treating as Infinity")
                        remaining_time = "Infinity"
            # If not set to Infinity above, compute numeric remaining time
            if remaining_time != "Infinity":
                try:
                    exit_time_numeric = float(matsim_link_exit_time)
                    remaining_time = exit_time_numeric - float(new_sim_time)
                except Exception:
                    LOG.warning(f"Failed to compute remaining_time from currentExitTime={matsim_link_exit_time} and time={new_sim_time}; treating as Infinity")
                    remaining_time = "Infinity"

            veh_pos = self.from_matsim_to_fleetpy_position(matsim_link, remaining_time=remaining_time)
            
            matsim_diverge_link = veh_state["divergeLink"]
            matsim_diverge_link_exit_time = veh_state["divergeTime"]
            if type(matsim_diverge_link_exit_time) == str and matsim_diverge_link_exit_time == "Infinity":
                LOG.warning("MATSim diverge link exit time 'Infinity' mapped to LARGE_INT")
                matsim_diverge_link_exit_time = LARGE_INT
            earliest_diverge_pos = self.from_matsim_to_fleetpy_position(matsim_diverge_link)
            earliest_diverge_time = matsim_diverge_link_exit_time
            
            state = self._from_matsim_to_fleetpy_veh_state(veh_state["state"])
            finished_leg_ids = [int(x) for x in veh_state["finished"]]
            
            picked_up = veh_pick_up_requests.get(vid, [])
            dropped_off = veh_drop_off_requests.get(vid, [])
            
            current_pick_up = veh_current_pick_up_requests.get(vid, [])
            current_drop_off = veh_current_drop_off_requests.get(vid, [])
            
            self.fs_obj.update_veh_state(new_sim_time, vid, 0, veh_pos, picked_up, dropped_off, state, earliest_diverge_pos, earliest_diverge_time, finished_leg_ids,
                                         current_pick_up, current_drop_off)
        
        list_requests = response_obj["submitted"] # list of dicts
        print(" -> number of new requests: ", len(list_requests))
        for rq_entry in list_requests:
            org_pos = self.from_matsim_to_fleetpy_position(int(rq_entry["originLink"]))
            org_str = f"{org_pos[0]};{org_pos[1]};{org_pos[2]}"
            dest_pos = self.from_matsim_to_fleetpy_position(int(rq_entry["destinationLink"]))
            dest_str = f"{dest_pos[0]};{dest_pos[1]};{dest_pos[2]}"
            if int(rq_entry["originLink"]) in self._non_unique_matsim_links or int(rq_entry["destinationLink"]) in self._non_unique_matsim_links:
                LOG.warning(f"Request {rq_entry['id']} has origin or destination on non-uniquely mapped link! -> set for automatic decline")
                dest_str = org_str  # just to have a valid destination, will be declined anyway
            rq_info_dict = {G_RQ_ID: self._from_matsim_to_fleetpy_rid(rq_entry["id"]),
                            G_RQ_ORIGIN: org_str, 
                            G_RQ_DESTINATION: dest_str, 
                            G_RQ_TIME: new_sim_time,
                            G_RQ_EPT: int(rq_entry["earliestPickupTime"]), # TODO optional
                            G_RQ_LPT: int(rq_entry["latestPickupTime"]), # TODO optional
                            G_RQ_LDT: int(rq_entry["latestArrivalTime"]), # TODO where does it come from?
                            G_RQ_PAX: int(rq_entry["size"])} # TODO to add?
            rq_series = pd.Series(rq_info_dict)
            rq_series.name = rq_info_dict[G_RQ_ID]
            self.fs_obj.add_request(rq_series)
            
        self.fs_obj.step(new_sim_time)
        
        new_assignments = self.fs_obj.get_current_assignments(new_sim_time) # dict (op_id, vid) -> VehPlan
        
        assignment_message = self._create_assignment_message(new_assignments)
        self.format_object_and_send_msg(assignment_message)

    def _create_assignment_message(self, new_assignments: Dict[Any, List[dict]]):
        """
        Create a message with the new assignments for MATSim.
        """
        assignment_message = {"@message": "assignment", "stops": {}}

        for (op_id, veh_id), stop_list in new_assignments.items():
            try:
                matsim_vehicle_id = self.fleetpy_to_matsim_vid[veh_id]
            except KeyError:
                LOG.warning(f"Unknown FleetPy vehicle id {veh_id} in assignments; known FleetPy->MATSim vids: {list(self.fleetpy_to_matsim_vid.keys())}")
                # Fallback: if exactly one vehicle is known, attribute to it; otherwise skip this vehicle's stops
                if len(self.fleetpy_to_matsim_vid) == 1:
                    matsim_vehicle_id = next(iter(self.fleetpy_to_matsim_vid.values()))
                else:
                    continue
            list_stops = []
            link_ids_for_log = []
            cur_link = self._last_matsim_link_by_fp_vid.get(veh_id)

            for i, stop in enumerate(stop_list):
                try:
                    matsim_edge = self.from_fleetpy_to_matsim_position(stop["pos"])
                except Exception as e:
                    LOG.error(f"Failed to convert stop position {stop['pos']}")
                    continue
                
                # validate link against known network ids; skip if not present
                if self._skip_unknown_assignment_links:
                    try:
                        _ = self.matsim_edge_to_fp_edge[int(matsim_edge)]
                    except Exception:
                        LOG.warning(f"Skipping assignment stop with unknown link {matsim_edge} for fp vid {veh_id}")
                        continue
                
                # Additional validation: ensure the link ID is not None and is valid
                if matsim_edge is None:
                    LOG.error(f"Got None as MATSim edge for stop {stop['pos']}, skipping")
                    continue
                
                try:
                    matsim_edge_str = str(matsim_edge)
                    # Verify this is a valid integer link ID
                    int(matsim_edge_str)
                except (ValueError, TypeError) as e:
                    LOG.error(f"Invalid MATSim link ID {matsim_edge} for stop {stop['pos']}: {e}")
                    continue
                
                # **NEW: Compute route to next stop**
                route = None
                if i < len(stop_list) - 1:
                    next_matsim_edge = self.from_fleetpy_to_matsim_position(stop_list[i+1]["pos"])
                    route = self.compute_matsim_route_between_stops(int(matsim_edge), int(next_matsim_edge))
                    
                    if route is None:
                        LOG.error(f"Cannot compute route from stop {i} (link {matsim_edge}) to stop {i+1} (link {next_matsim_edge})")
                        # Optionally: skip this assignment or continue without route
                
                list_pick_up = [self._from_fleetpy_to_matsim_rid(rid) for rid in stop["boarding_rids"]]
                list_drop_off = [self._from_fleetpy_to_matsim_rid(rid) for rid in stop["alighting_rids"]]
                stop_duration = stop["duration"] if stop["duration"] is not None else 0
                earliest_start_time = stop["earliest_start_time"]
                stop_id = stop["id"]
                
                stop_dict = {
                    "link": matsim_edge_str,
                    "pickup": list_pick_up,
                    "dropoff": list_drop_off,
                    "stopDuration": int(stop_duration),
                    "id": stop_id
                }
                
                # **NEW: Add route if available**
                if route is not None:
                    stop_dict["route"] = route
                
                if earliest_start_time is not None:
                    stop_dict["earliestStartTime"] = int(earliest_start_time)
                
                list_stops.append(stop_dict)
                try:
                    link_ids_for_log.append(matsim_edge_str)
                except Exception:
                    pass
            # ensure first stop starts at current link if available and valid
            if self._force_start_on_current_link:
                try:
                    if list_stops and cur_link is not None and int(cur_link) in self.matsim_edge_to_fp_edge:
                        list_stops[0]["link"] = str(int(cur_link))
                        if link_ids_for_log:
                            link_ids_for_log[0] = str(int(cur_link))
                except Exception:
                    pass
            
            # Validate stop connectivity
            valid_stops = []   
            for i, stop in enumerate(list_stops):
                valid_stops.append(stop)
                if i < len(list_stops) - 1:
                    current_link = int(stop["link"])
                    next_link = int(list_stops[i+1]["link"])
                    try:
                        current_fp_edge = self.matsim_edge_to_fp_edge[current_link]
                        next_fp_edge = self.matsim_edge_to_fp_edge[next_link]
                        
                        current_end_node = current_fp_edge[1]
                        next_start_node = next_fp_edge[0]
                        
                        if current_end_node != next_start_node:
                            if current_end_node not in self.fp_edge_to_matsim_edge or not self.fp_edge_to_matsim_edge[current_end_node]:
                                LOG.error(f"Stop {i} link {current_link} end node {current_end_node} has no outgoing edges!")
                                valid_stops = []
                                break
                    except KeyError as e:
                        LOG.error(f"Link validation failed: {e}")
                        valid_stops = []
                        break
            
            list_stops = valid_stops
            if not list_stops:
                LOG.warning(f"Skipping assignment for vehicle {matsim_vehicle_id}: routing validation failed")
                continue
            
            assignment_message["stops"][matsim_vehicle_id] = list_stops
            try:
                LOG.info(f"Assignment for MATSim vehicle {matsim_vehicle_id}: cur_link={cur_link} links={link_ids_for_log}")
            except Exception:
                pass
            
        return assignment_message    

    def _validate_ridesync_stops_against_network(self):
        rs_file = self.scenario_parameters.get("ridesync_all_stops_file")
        if not rs_file:
            return
        # Build absolute path
        if os.path.isabs(rs_file):
            f_p = rs_file
        else:
            f_p = os.path.join(self.dir_names[G_DIR_DATA], rs_file)
        if not os.path.isfile(f_p):
            LOG.debug(f"RideSync stops file not found at {f_p}")
            return
        try:
            df = pd.read_csv(f_p, delimiter=';')
        except Exception:
            df = pd.read_csv(f_p)
        if 'node_index' not in df.columns:
            LOG.debug(f"RideSync stops file at {f_p} has no node_index column")
            return
        stop_nodes = set(int(x) for x in df['node_index'].tolist() if pd.notna(x))
        outgoing_nodes = set(self.fp_edge_to_matsim_edge.keys())
        incoming_nodes = set()
        for from_node, to_map in self.fp_edge_to_matsim_edge.items():
            try:
                incoming_nodes.update(to_map.keys())
            except Exception:
                pass
        missing = [n for n in stop_nodes if (n not in outgoing_nodes and n not in incoming_nodes)]
        if missing:
            LOG.warning(f"RideSync stops reference {len(missing)} nodes not present in network mapping: sample={missing[:10]}")
        # nodes present but with no incident edges (isolated)
        isolated = []
        for n in stop_nodes:
            has_out = n in outgoing_nodes and bool(self.fp_edge_to_matsim_edge.get(n))
            has_in = any((n in to_map) for to_map in self.fp_edge_to_matsim_edge.values())
            if not has_out and not has_in:
                isolated.append(n)
        if isolated:
            LOG.warning(f"RideSync stops on isolated nodes (no incoming/outgoing edges): count={len(isolated)} sample={isolated[:10]}")
        LOG.info(f"RideSync stops validation: total={len(stop_nodes)} missing={len(missing)} isolated={len(isolated)}")

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
        # Prefer an incoming edge ending at this node for node-only positions (stops at nodes)
        if fleetpy_position[-1] is None:
            node = fleetpy_position[0]
            if self._prefer_incoming_node_edge:
                try:
                    # Find any incoming neighbor f -> node
                    incoming_edge = None
                    for from_node, to_map in self.fp_edge_to_matsim_edge.items():
                        try:
                            link_id = to_map.get(node)
                        except Exception:
                            link_id = None
                        if link_id is not None:
                            incoming_edge = link_id
                            break
                    if incoming_edge is not None:
                        return incoming_edge
                except Exception:
                    pass
            
            # Fallback to any outgoing edge
            try:
                if node in self.fp_edge_to_matsim_edge and len(self.fp_edge_to_matsim_edge[node]) > 0:
                    LOG.warning(f"fleetpy position is on node {node}, selecting arbitrary outgoing edge as fallback")
                    any_target = list(self.fp_edge_to_matsim_edge[node].keys())[0]
                    matsim_edge = self.fp_edge_to_matsim_edge[node][any_target]
                    return matsim_edge
                else:
                    # Node has no outgoing edges, try to find any incoming edge to this node
                    LOG.warning(f"Node {node} has no outgoing edges in mapping, trying incoming edges")
                    for from_node, to_map in self.fp_edge_to_matsim_edge.items():
                        if node in to_map:
                            matsim_edge = to_map[node]
                            LOG.warning(f"Using incoming edge {from_node}->{node} (link {matsim_edge}) for node position")
                            return matsim_edge
                    
                    # Last resort: find any edge that contains this node
                    LOG.error(f"Node {node} not found in any edge mapping! Looking for alternative...")
                    # Check if this node appears anywhere in the matsim_edge_to_fp_edge mapping
                    for matsim_link, fp_edge in self.matsim_edge_to_fp_edge.items():
                        if node in fp_edge:
                            LOG.warning(f"Found node {node} in MATSim link {matsim_link} (edge {fp_edge})")
                            return matsim_link
                    
                    # Cannot map this node at all
                    LOG.error(f"Cannot map node {node} to any MATSim link! This stop will be invalid.")
                    raise ValueError(f"Node {node} cannot be mapped to any MATSim link")
                    
            except Exception as e:
                LOG.error(f"Failed to convert FleetPy node position {fleetpy_position} to MATSim: {e}")
                raise
        else:
            try:
                matsim_edge = self.fp_edge_to_matsim_edge[fleetpy_position[0]][fleetpy_position[1]]
                return matsim_edge
            except KeyError as e:
                LOG.error(f"Edge {fleetpy_position[0]}->{fleetpy_position[1]} not found in mapping!")
                raise
    
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
    
    def compute_matsim_route_between_stops(self, from_link_id: int, to_link_id: int) -> List[str]:
        """
        Compute a route (list of MATSim link IDs) from one stop to another.
        """
        try:
            from_fp_edge = self.matsim_edge_to_fp_edge[from_link_id]
            to_fp_edge = self.matsim_edge_to_fp_edge[to_link_id]
            
            from_node = from_fp_edge[1]
            to_node = to_fp_edge[0]
            
            # **WICHTIG: Konvertiere zu Positionen!**
            from_pos = return_node_position(from_node)
            to_pos = return_node_position(to_node)
            
            # **Übergib POSITIONEN, nicht Node-IDs!**
            route_nodes, _, _ = self.fs_obj.routing_engine.return_best_route_1to1(from_pos, to_pos)
            
            matsim_route = [str(from_link_id)]
            
            for i in range(len(route_nodes) - 1):
                node_a = route_nodes[i]
                node_b = route_nodes[i + 1]
                try:
                    link_id = self.fp_edge_to_matsim_edge[node_a][node_b]
                    matsim_route.append(str(link_id))
                except KeyError:
                    return None
            
            return matsim_route
            
        except Exception as e:
            LOG.error(f"Route computation failed: {e}")
            return None


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
    
    matsim_network_path = r"C:\Users\ge37ser\Documents\Projekte\MINGA\AP5\IRTSystemX\KopplungMATSimFleetPy\matsim-fleetpy\scenario\network.xml.gz"
    
    const_cfg = ConstantConfig(r"C:\Users\ge37ser\Documents\Coding\FleetPy\studies\test_matsim_coupling\scenarios\constant_config_pool.csv")
    print(const_cfg)
    scenarios_cfg = ScenarioConfig(r"C:\Users\ge37ser\Documents\Coding\FleetPy\studies\test_matsim_coupling\scenarios\example_pool.csv")
    print(scenarios_cfg)
    
    whole_config = const_cfg + scenarios_cfg[0]
    whole_config["matsim_network_path"] = matsim_network_path
    whole_config["study_name"] = "test_matsim_coupling"
    whole_config["log_level"] = "debug"
    whole_config["n_cpu_per_sim"] = 1
    
    matsim_socket = MATSimSocket(host, port, whole_config, log_communication=True)
    matsim_socket.keep_socket_alive()