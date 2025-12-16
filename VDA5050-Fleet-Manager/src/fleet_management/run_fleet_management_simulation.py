# 【文件名：simulation.py】

# Import the necessary libraries and modules.
import os
import sys
import json
import time
import logging
import subprocess
import argparse
from typing import Dict, Optional 

# Get the current working directory.
current_dir = os.getcwd()

# Fix path handling for Linux (Ubuntu).
if current_dir.endswith("fleet_management"):
    pass
elif current_dir.endswith("imrl_workspace"):
    os.chdir(os.path.join(current_dir, "fleet_management"))
elif current_dir.endswith("src"):
    os.chdir(os.pardir)
else:
    pass 

# Finally add src path
sys.path.insert(0, os.path.join(os.getcwd(), "src"))


# Import the necessary classes.
from src.fleet_management.graph import Graph
from src.fleet_management.agents import Agents
from src.fleet_management.task_management import TaskManagement
from src.fleet_management.task_assignment import TaskAssignment
from src.fleet_management.fleet_management import FleetManagement

def setup_logging(log_file_path: str) -> logging.Logger:
    # Ensure directory exists
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        filename=log_file_path,
        filemode='a'
    )
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logging.getLogger('').addHandler(console_handler)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    return logging.getLogger(__name__)

class ConfigManager:
    def __init__(self, config_paths: Dict[str, str]):
        self.config_data = self._load_json(config_paths['config'])
        self.lif_data = self._load_json(config_paths['lif'])
        self.agents_initialization_data = self._load_json(config_paths['agents_initialization'])
        self.transportation_tasks_data = self._load_json(config_paths['transportation_tasks'])

    @staticmethod
    def _load_json(path: str) -> dict:
        try:
            with open(path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.error(f"File not found: {path}")
            sys.exit(1)
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON format in file: {path}")
            sys.exit(1)

def start_threads(task_management: TaskManagement, task_assignment: Optional[TaskAssignment], fleet_management: FleetManagement, current_task_id: int):
    """
    Start the threads.
    """
    # Always start task management (to check for completion of the current task)
    try:
        task_management.check_completion_thread.start()
    except RuntimeError:
        pass # Thread might already be started

    # === CONTROL LOGIC ===
    if current_task_id >= 8 and task_assignment is not None:
        print(f"--> [INFO] Starting Task Assignment Thread for Task {current_task_id} (Full Auto Mode)")
        try:
            task_assignment.assignment_thread.start()
        except AttributeError:
            print("--> [WARN] assignment_thread not found in TaskAssignment class.")
        except RuntimeError:
            pass
    else:
        print(f"--> [INFO] Task Assignment Thread SKIPPED for Task {current_task_id} (Single Task Mode)")

def run_simulation(config_manager: ConfigManager, logging: logging.Logger, current_task_id: int = 9):
    agent_simulation_path = "src/mobile_robot_simulation/dist/agent_simulation"
    try:
        subprocess.Popen([agent_simulation_path])
        time.sleep(3)
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to execute {agent_simulation_path}: {e}")
    
    simulation_start_time = time.time()

    # 1. Initialize Graph & Agents
    graph = Graph(lif_data=config_manager.lif_data)
    agents = Agents(config_data=config_manager.config_data, graph=graph,
                           agents_initialization_data=config_manager.agents_initialization_data, logging=logging,
                           simulation_start_time=simulation_start_time)
    
    # 2. Initialize TaskManagement
    task_management = TaskManagement(graph=graph, agents=agents,
                                     transportation_tasks_data=config_manager.transportation_tasks_data,
                                     simulation_start_time=simulation_start_time)
    
    # 3. Initialize FleetManagement FIRST (It holds the Reservation Table)
    # This is crucial: FleetManagement must exist before TaskAssignment can use it.
    fleet_management = FleetManagement(config_data=config_manager.config_data, graph=graph, agents=agents,
                                       task_management=task_management, simulation_start_time=simulation_start_time,
                                       current_task_id=current_task_id)
    
    # 4. Initialize TaskAssignment (Injecting fleet_management dependency)
    task_assignment = None
    if current_task_id >= 8:
        logging.info("Initializing TaskAssignment System...")
        # We pass fleet_management here so TaskAssignment can use the shared A* and Reservation Table
        task_assignment = TaskAssignment(graph=graph, agents=agents, task_management=task_management,
                                         simulation_start_time=simulation_start_time,
                                         fleet_management=fleet_management)
    else:
        logging.info("Skipping TaskAssignment System (Manual/Single Task Mode)")

    
    # 5. Start Threads
    start_threads(task_management, task_assignment, fleet_management, current_task_id)

    # Keep main thread alive
    time.sleep(config_manager.config_data["simulation_run_time"])

def main():
    parser = argparse.ArgumentParser(description="Run Fleet Management Simulation")
    parser.add_argument("--task", type=int, default=9, help="Task ID to execute (e.g., 2, 3, 6, 9)")
    args = parser.parse_args()
    current_task_id = args.task

    logging = setup_logging("data/output_files/logging_file.log")
    logging.info(f"--- STARTING SIMULATION FOR TASK {current_task_id} ---")

    config_paths = {
        "config": "data/input_files/config_file.json",
        "lif": "data/input_files/lif_file.json",
        "agents_initialization": "data/input_files/agentsInitialization_file.json",
        "transportation_tasks": "data/input_files/transportationTasks_file.json",
    }

    config_manager = ConfigManager(config_paths)

    run_simulation(config_manager, logging, current_task_id=current_task_id)

if __name__ == "__main__":
    main()