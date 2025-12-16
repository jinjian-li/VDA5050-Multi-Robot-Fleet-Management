import subprocess
import os

# Setup the directory paths
data_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
exe_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "mobile_robot_simulation", "dist","run_simulation")


# parameters
# change the task number to run different tasks, available tasks are 1, 2, 3, 4, 5, 6, 7
args = ["--task", "3", 
        "--data_root", str(data_root)]

result = subprocess.run([exe_path] + args, capture_output=True, text=True)
