#!/usr/bin/env python3
"""
RideSync Semi-Flexible Routing Test Runner
"""

import os
import sys

# Add FleetPy source to path
MAIN_DIR = os.path.dirname(__file__)
src_dir = os.path.join(MAIN_DIR, "src")
sys.path.append(src_dir)

def run_ridesync_standalone_test():
    """Run RideSync algorithm in standalone FleetPy simulation"""
    print("Starting RideSync Semi-Flexible Routing Test...")
    print("=" * 60)
    
    # Set up paths
    scs_path = os.path.join(MAIN_DIR, "studies", "ridesync_study", "scenarios")
    
    # Configuration files
    cc = os.path.join(scs_path, "ridesync_constant_config.csv")
    sc = os.path.join(scs_path, "ridesync_scenario.csv")
    
    # Check if configuration files exist
    if not os.path.exists(cc):
        print(f"ERROR: Configuration file not found: {cc}")
        print("Please create the RideSync study configuration first.")
        return False
        
    if not os.path.exists(sc):
        print(f"ERROR: Scenario file not found: {sc}")
        print("Please create the RideSync scenario configuration first.")
        return False
    
    try:
        # Import after path setup
        from run_examples import run_scenarios
        
        print(f"Using configuration: {cc}")
        print(f"Using scenarios: {sc}")
        print("\nStarting simulation...")
        
        run_scenarios(cc, sc, log_level="warning", n_cpu_per_sim=1, n_parallel_sim=1)
        
        print("\nSUCCESS: RideSync standalone test completed successfully!")
        print("Results saved in: FleetPy/studies/ridesync_study/results/")
        return True
        
    except ImportError as e:
        print(f"\nERROR: Import error: {e}")
        print("Make sure FleetPy dependencies are installed.")
        return False
        
    except Exception as e:
        import traceback
        print(f"\nERROR: Error running RideSync test: {e}")
        print("Full traceback:")
        print(traceback.format_exc())
        print("Please fix the issues before proceeding.")
        return False

def check_ridesync_dependencies():
    """Check if all RideSync components are properly set up"""
    print("Checking RideSync setup...")
    
    # Check folder structure
    ridesync_path = os.path.join(MAIN_DIR, "src", "fleetctrl", "RideSync")
    if not os.path.exists(ridesync_path):
        print("ERROR: RideSync module folder not found")
        return False
    
    # Check study folder
    study_path = os.path.join(MAIN_DIR, "studies", "ridesync_study")
    if not os.path.exists(study_path):
        print("ERROR: RideSync study folder not found")
        return False
    
    print("SUCCESS: Basic folder structure exists")
    return True

if __name__ == "__main__":
    print("RideSync Semi-Flexible Routing System")
    print("=" * 60)
    
    # Check dependencies first
    if not check_ridesync_dependencies():
        print("\nPlease run the setup script to create the required folders.")
        sys.exit(1)
    
    # Run the test
    success = run_ridesync_standalone_test()
    
    if success:
        print("\nAll systems ready!")
    else:
        print("\nSetup incomplete - please address the issues above.")
        sys.exit(1) 