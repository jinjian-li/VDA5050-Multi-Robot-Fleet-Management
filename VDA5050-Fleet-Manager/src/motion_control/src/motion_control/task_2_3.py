'''
This script is an example code to be used for individual task 2 and 3. 
The basic structure is provided, but the students need to implement the missing parts.
'''

import json
import time
import argparse
import math
from datetime import datetime
import paho.mqtt.client as mqtt

class Robot:
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_node_id = None

        self.nodes = []         # Released nodes from order
        self.edges = []         # Released edges from order
        self.trajectory = []    # List of (x, y) waypoints
        self.current_index = 0  # Index of current waypoint

        self.status_update_needed = False  # Flag for event-triggered status update

    def update_pose(self, msg):
        # Parse pose update from payload
        try:
            pose_data = json.loads(msg.payload.decode("utf-8"))
            
            # Extract position
            position = pose_data.get("position", {})
            self.x = position.get("x", self.x)
            self.y = position.get("y", self.y)
            
            # Extract orientation (quaternion) and convert to theta (yaw angle)
            orientation = pose_data.get("orientation", {})
            x = orientation.get("x", 0.0)
            y = orientation.get("y", 0.0)
            z = orientation.get("z", 0.0)
            w = orientation.get("w", 1.0)
            
            # Convert quaternion to yaw angle (theta)
            # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)
            
            # add here: Debug output for first pose received
            if not hasattr(self, '_pose_initialized'):
                print(f"✅ DATA RECEIVED! Robot is at position: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
                self._pose_initialized = True
        except Exception as e:
            print(f"Error parsing pose message: {e}")

    def receive_order(self, msg):
        # Parse nodes and edges from the order message
        try:
            order = json.loads(msg.payload.decode("utf-8"))
            
            # Extract released nodes
            nodes = order.get("nodes", [])
            released_nodes = [n for n in nodes if n.get("released", False)]
            
            # Sort nodes by sequenceId (execution order)
            released_nodes.sort(key=lambda n: n.get("sequenceId", 999))
            
            # Extract released edges
            edges = order.get("edges", [])
            released_edges = [e for e in edges if e.get("released", False)]
            released_edges.sort(key=lambda e: e.get("sequenceId", 999))
            
            # Save nodes and edges
            self.nodes = released_nodes
            self.edges = released_edges
            
            # Generate trajectory based on node positions
            self.trajectory = []
            for node in released_nodes:
                node_pos = node.get("nodePosition", {})
                x = node_pos.get("x")
                y = node_pos.get("y")
                if x is not None and y is not None:
                    self.trajectory.append((x, y))
            
            # Reset index and last node
            self.current_index = 0
            self.last_node_id = None
            self.status_update_needed = True  # Order receipt triggers a status update
            
            # add here: Debug output for order reception
            print(f"Received order: {len(self.trajectory)} waypoints")
        except Exception as e:
            print(f"Error parsing order message: {e}")

    def build_status_message(self):
        # Construct and return VDA5050-compliant status message
        status = {
            "headerId": 1,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": self.name,
            "orderId": "1",
            "orderUpdateId": 0,
            "agvPosition": {
                "positionInitialized": True,
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "mapId": "Map_1"
            },
            "driving": self.current_index < len(self.trajectory),
            "nodeStates": [],
            "edgeStates": [],
            "actionStates": [],
            "batteryState": {
                "batteryCharge": 100,
                "charging": False
            },
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": False
            }
        }
        
        # Add lastNodeId if available
        if self.last_node_id is not None:
            status["lastNodeId"] = self.last_node_id
            # Find the sequenceId for the last node
            for node in self.nodes:
                if node.get("nodeId") == self.last_node_id:
                    status["lastNodeSequenceId"] = node.get("sequenceId", 0)
                    break
        
        # Add node states (simplified - just released nodes)
        for node in self.nodes:
            status["nodeStates"].append({
                "nodeId": node.get("nodeId"),
                "sequenceId": node.get("sequenceId"),
                "released": node.get("released", False),
                "nodePosition": node.get("nodePosition", {}),
                "actions": node.get("actions", [])
            })
        
        # Add edge states (simplified - just released edges)
        for edge in self.edges:
            status["edgeStates"].append({
                "edgeId": edge.get("edgeId"),
                "sequenceId": edge.get("sequenceId"),
                "released": edge.get("released", False),
                "startNodeId": edge.get("startNodeId"),
                "endNodeId": edge.get("endNodeId"),
                "actions": edge.get("actions", [])
            })
        
        return status


def follow_trajectory(robot: Robot):
    """
    Follow the trajectory by:
    - Computing control commands based on current pose and next waypoint
    - Publishing movement commands
    - Advancing to next waypoint when close enough

    Students should:
    - Implement trajectory following logic here (e.g., PD controller)
    - Calculate linear and angular velocities
    - Decide when to advance to the next waypoint
    - Set robot.status_update_needed = True if a waypoint/node is reached
    """
    if robot.current_index < len(robot.trajectory):
        target = robot.trajectory[robot.current_index]
        target_x, target_y = target

        # Compute control commands (linear_vel, angular_vel)
        # Calculate distance and angle to target
        dx = target_x - robot.x
        dy = target_y - robot.y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Desired heading angle towards target
        desired_theta = math.atan2(dy, dx)
        
        # Angle error (normalize to [-pi, pi])
        angle_error = desired_theta - robot.theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Control parameters
        kp_linear = 0.5   # Proportional gain for linear velocity
        kp_angular = 1.0  # Proportional gain for angular velocity
        max_linear_vel = 0.2   # Maximum linear velocity
        max_angular_vel = 1.0  # Maximum angular velocity
        distance_threshold = 0.15  # Distance threshold to consider waypoint reached
        
        # Compute velocities using proportional control
        linear_vel = min(kp_linear * distance, max_linear_vel)
        angular_vel = kp_angular * angle_error
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        # Reduce linear velocity if angle error is large (prioritize rotation)
        # But allow some forward movement even with large angle error to avoid getting stuck
        if abs(angle_error) > 0.5:  # ~29 degrees
            linear_vel *= 0.2
        elif abs(angle_error) > 0.3:  # ~17 degrees
            linear_vel *= 0.3

        # Check if the robot is close enough to the target
        # Only advance if we're close AND moving slowly (to avoid skipping waypoints)
        if distance < distance_threshold:
            # Only consider reached if moving slowly or very close
            if distance < 0.08 or linear_vel < 0.1:
                # Update last_node_id to the node we just reached (before incrementing index)
                if robot.current_index < len(robot.nodes):
                    robot.last_node_id = robot.nodes[robot.current_index].get("nodeId")
                    # add here: Debug output when reaching a node
                    print(f"Reached node {robot.last_node_id} ({robot.current_index + 1}/{len(robot.trajectory)})")
                robot.current_index += 1
                robot.status_update_needed = True

        cmd = {
            "linear": {"x": linear_vel, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_vel}
        }
        return cmd
    else:
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

def send_status_update(client, topic, robot: Robot):
    status = robot.build_status_message()
    client.publish(topic, json.dumps(status))

def main(robot_name):
    robot = Robot(robot_name)

    topic_cmd = f"uagv/v2/KIT/{robot_name}/cmd"
    topic_pose = f"uagv/v2/KIT/{robot_name}/pose"
    topic_order = f"uagv/v2/KIT/{robot_name}/order"
    topic_state = f"uagv/v2/KIT/{robot_name}/state"

    def on_connect(client, userdata, flags, rc): # Called when the client connects to the broker
        client.subscribe(topic_pose)
        client.subscribe(topic_order)

    def on_message(client, userdata, msg): # Called when a message is received
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg)

    client = mqtt.Client() # Create a new MQTT client instance
    client.on_connect = on_connect # Set the on_connect callback
    client.on_message = on_message # Set the on_message callback
    client.connect("localhost", 1883, 60)
    client.loop_start()

    last_status_time = time.time()

    while True:
        cmd = follow_trajectory(robot)
        client.publish(topic_cmd, json.dumps(cmd))

        # Send status update if an event occurred or periodically
        if robot.status_update_needed or time.time() - last_status_time >= 30:
            send_status_update(client, topic_state, robot)
            robot.status_update_needed = False
            last_status_time = time.time()

        time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="mouse001") #Change to your robot name or parse it from command line
    args = parser.parse_args()
    main(args.robot)