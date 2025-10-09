#!/usr/bin/env python3
"""
RideSync Coordinate Mapping Utility
===================================
Maps UTM coordinates from all_stops.csv to actual network nodes
and adds node_index column to the all_stops.csv file.

Usage:
    python coordinate_mapping.py
"""

import pandas as pd
import numpy as np
import os
from typing import Tuple, Optional
import math

class RideSyncCoordinateMapper:
    """Maps stop coordinates to network nodes and updates all_stops.csv file."""
    
    def __init__(self, data_dir: str = None):
        if data_dir is None:
            # Auto-detect path based on current working directory
            current_dir = os.getcwd()
            if "ridesync_stops" in current_dir:
                # Running from ridesync_stops directory
                self.data_dir = ".."
            else:
                # Running from project root or FleetPy directory
                self.data_dir = "FleetPy/data"
        else:
            self.data_dir = data_dir
            
        self.nodes_df = None
        self.all_stops_df = None
        self._load_network_nodes()
        self._load_all_stops()
    
    def _load_network_nodes(self):
        """Load network nodes from nodes.csv"""
        nodes_path = os.path.join(self.data_dir, "networks/example_network/base/nodes.csv")
        
        print(f"Looking for nodes.csv at: {os.path.abspath(nodes_path)}")
        if not os.path.exists(nodes_path):
            raise FileNotFoundError(f"Cannot find nodes.csv at {nodes_path}")
            
        self.nodes_df = pd.read_csv(nodes_path)
        print(f"Loaded {len(self.nodes_df)} network nodes")
        print(f"Network bounds: X({self.nodes_df['pos_x'].min():.0f} - {self.nodes_df['pos_x'].max():.0f}), Y({self.nodes_df['pos_y'].min():.0f} - {self.nodes_df['pos_y'].max():.0f})")
    
    def _load_all_stops(self):
        """Load all stops with UTM coordinates"""
        if "ridesync_stops" in os.getcwd():
            # Running from ridesync_stops directory
            stops_path = "all_stops.csv"
        else:
            stops_path = os.path.join(self.data_dir, "ridesync_stops/all_stops.csv")
        
        print(f"Looking for all_stops.csv at: {os.path.abspath(stops_path)}")
        if not os.path.exists(stops_path):
            raise FileNotFoundError(f"Cannot find all_stops.csv at {stops_path}")
            
        # Try different encodings and parsing options for special characters
        for encoding in ['utf-8', 'iso-8859-1', 'cp1252']:
            try:
                # Read with different options to handle parsing issues
                self.all_stops_df = pd.read_csv(
                    stops_path, 
                    sep=';', 
                    encoding=encoding,
                    quotechar='"',
                    skipinitialspace=True,
                    engine='python'  # More flexible parser
                )
                print(f"Successfully loaded with encoding: {encoding}")
                break
            except (UnicodeDecodeError, pd.errors.ParserError) as e:
                print(f"Failed with encoding {encoding}: {e}")
                continue
        else:
            raise RuntimeError("Could not read all_stops.csv with any encoding")
            
        print(f"Loaded {len(self.all_stops_df)} stops from all_stops.csv")
        print(f"Columns: {list(self.all_stops_df.columns)}")
        
        # Check if UTM columns exist, if not, add them
        if 'pos_x' not in self.all_stops_df.columns or 'pos_y' not in self.all_stops_df.columns:
            print("WARNING: UTM columns (pos_x, pos_y) not found. Converting from lat/lng...")
            self._add_utm_coordinates()
        else:
            print("UTM columns found")
            
        print(f"Stop bounds: X({self.all_stops_df['pos_x'].min():.0f} - {self.all_stops_df['pos_x'].max():.0f}), Y({self.all_stops_df['pos_y'].min():.0f} - {self.all_stops_df['pos_y'].max():.0f})")
    
    def _add_utm_coordinates(self):
        """Convert lat/lng to UTM coordinates and add UTM columns to the dataframe"""
        print("Converting lat/lng to UTM coordinates (Zone 32U)...")
        
        utm_x_list = []
        utm_y_list = []
        
        for _, row in self.all_stops_df.iterrows():
            lat = row['lat']
            lng = row['long']
            
            # Convert lat/lng to UTM Zone 32U coordinates
            utm_x, utm_y = self._lat_lng_to_utm(lat, lng, zone_number=32, zone_letter='U')
            utm_x_list.append(utm_x)
            utm_y_list.append(utm_y)
        
        # Add UTM columns to dataframe
        self.all_stops_df['pos_x'] = utm_x_list
        self.all_stops_df['pos_y'] = utm_y_list
        self.all_stops_df['utm_zone_number'] = 32
        self.all_stops_df['utm_zone_letter'] = 'U'
        
        print(f"Added UTM coordinates for {len(self.all_stops_df)} stops")
    
    def _lat_lng_to_utm(self, lat: float, lng: float, zone_number: int = 32, zone_letter: str = 'U') -> Tuple[float, float]:
        """
        Convert latitude/longitude to UTM coordinates for Zone 32U.
        Simplified conversion for Central Europe (Zone 32U).
        """
        # UTM Zone 32 central meridian
        central_meridian = 9.0  # degrees
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lng_rad = math.radians(lng)
        central_meridian_rad = math.radians(central_meridian)
        
        # WGS84 ellipsoid parameters
        a = 6378137.0  # Semi-major axis
        e2 = 0.00669437999014  # First eccentricity squared
        
        # UTM scale factor
        k0 = 0.9996
        
        # Calculate UTM coordinates (simplified)
        # This is a simplified version - for production use, consider using pyproj
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        T = math.tan(lat_rad)**2
        C = e2 * math.cos(lat_rad)**2 / (1 - e2)
        A = math.cos(lat_rad) * (lng_rad - central_meridian_rad)
        
        # UTM Easting
        x = k0 * N * (A + (1 - T + C) * A**3 / 6 + (5 - 18*T + T**2 + 72*C - 58*e2) * A**5 / 120)
        x += 500000  # False easting
        
        # UTM Northing  
        M = a * ((1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat_rad
                - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat_rad)
                + (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat_rad)
                - (35*e2**3/3072) * math.sin(6*lat_rad))
        
        y = k0 * (M + N * math.tan(lat_rad) * (A**2/2 + (5 - T + 9*C + 4*C**2) * A**4/24 
                                               + (61 - 58*T + T**2 + 600*C - 330*e2) * A**6/720))
        
        # No false northing needed for northern hemisphere
        
        return x, y
    
    def find_nearest_node(self, utm_x: float, utm_y: float) -> Tuple[int, float]:
        """
        Find the nearest network node to given UTM coordinates.
        
        Returns:
            Tuple of (node_index, distance)
        """
        # Calculate Euclidean distances to all nodes
        distances = np.sqrt(
            (self.nodes_df['pos_x'] - utm_x)**2 + 
            (self.nodes_df['pos_y'] - utm_y)**2
        )
        
        # Find the nearest node
        nearest_idx = distances.idxmin()
        nearest_node = self.nodes_df.loc[nearest_idx]
        nearest_distance = distances.iloc[nearest_idx]
        
        return int(nearest_node['node_index']), nearest_distance
    
    def add_node_mapping_to_all_stops(self) -> str:
        """Add node_index column to all_stops.csv by finding nearest network nodes"""
        print("=== Adding node_index mapping to all_stops.csv ===")
        
        # Add node_index column
        node_indices = []
        distances = []
        
        for _, stop in self.all_stops_df.iterrows():
            # Find nearest network node
            node_id, distance = self.find_nearest_node(stop['pos_x'], stop['pos_y'])
            node_indices.append(node_id)
            distances.append(distance)
            
            stop_type = "Fixed" if stop['fixed_stop'] else "Optional"
            print(f"{stop_type} stop '{stop['stop_name']}' -> Node {node_id} (distance: {distance:.1f}m)")
        
        # Add new columns
        self.all_stops_df['node_index'] = node_indices
        self.all_stops_df['distance_to_node'] = distances
        
        # Save updated file
        if "ridesync_stops" in os.getcwd():
            # Running from ridesync_stops directory
            output_path = "all_stops.csv"
        else:
            output_path = os.path.join(self.data_dir, "ridesync_stops/all_stops.csv")
        
        self.all_stops_df.to_csv(output_path, sep=';', index=False)
        
        print(f"Updated {output_path} with node_index mappings")
        print(f"   Average distance to nearest node: {np.mean(distances):.1f}m")
        print(f"   Max distance to nearest node: {np.max(distances):.1f}m")
        
        return output_path
    
    def validate_node_mapping(self) -> bool:
        """Validate that all mapped nodes exist in the network"""
        if 'node_index' not in self.all_stops_df.columns:
            print("ERROR: No node_index column found. Run add_node_mapping_to_all_stops() first.")
            return False
        
        all_stop_nodes = set(self.all_stops_df['node_index'])
        network_nodes = set(self.nodes_df['node_index'])
        
        valid = True
        invalid_nodes = all_stop_nodes - network_nodes
        
        if invalid_nodes:
            print(f"ERROR: Invalid nodes found: {invalid_nodes}")
            valid = False
        else:
            print("All stop nodes are valid network nodes")
        
        # Check for duplicate node usage
        node_counts = self.all_stops_df['node_index'].value_counts()
        duplicates = node_counts[node_counts > 1]
        
        if not duplicates.empty:
            print(f"WARNING: {len(duplicates)} nodes are used by multiple stops:")
            for node_id, count in duplicates.items():
                stops = self.all_stops_df[self.all_stops_df['node_index'] == node_id]['stop_name'].tolist()
                print(f"   Node {node_id}: {stops}")
        
        return valid
    
    def show_statistics(self):
        """Show statistics about the stop-to-node mapping"""
        if 'node_index' not in self.all_stops_df.columns:
            print("No node mapping available yet.")
            return
        
        print("\n=== Stop-to-Node Mapping Statistics ===")
        
        # Fixed vs Optional stops
        fixed_stops = self.all_stops_df[self.all_stops_df['fixed_stop'] == True]
        optional_stops = self.all_stops_df[self.all_stops_df['fixed_stop'] == False]
        
        print(f"Fixed stops: {len(fixed_stops)}")
        print(f"Optional stops: {len(optional_stops)}")
        print(f"Total stops: {len(self.all_stops_df)}")
        
        # Distance statistics
        if 'distance_to_node' in self.all_stops_df.columns:
            distances = self.all_stops_df['distance_to_node']
            print(f"\nDistance to nearest node (meters):")
            print(f"  Average: {distances.mean():.1f}m")
            print(f"  Median: {distances.median():.1f}m") 
            print(f"  Min: {distances.min():.1f}m")
            print(f"  Max: {distances.max():.1f}m")
        
        print("="*45)
    
    def update_all_stops_with_nodes(self):
        """Main function to update all_stops.csv with node mappings"""
        print("=== Updating all_stops.csv with Network Node Mapping ===")
        
        self.add_node_mapping_to_all_stops()
        self.validate_node_mapping()
        self.show_statistics()
        
        print("=== all_stops.csv update completed ===")


if __name__ == "__main__":
    mapper = RideSyncCoordinateMapper()
    mapper.update_all_stops_with_nodes()