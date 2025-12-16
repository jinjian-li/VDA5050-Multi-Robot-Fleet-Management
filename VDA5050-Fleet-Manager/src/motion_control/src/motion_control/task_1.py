'''
This script is an example code to be used for individual task 1. 
'''

import json
import paho.mqtt.client as mqtt

class Robot:
    def __init__(self, name):
        self.name = name
        self.nodes = []
        self.edges = []

    
        
    def receive_order(self, msg):
        """
        Process the received VDA5050 order message and extract a route.
        """
        # 1. Parse JSON payload
        try:
            order = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            print("JSON decode error:", e)
            return

        print("\n===== RECEIVED ORDER =====")

        # 2. Extract released nodes
        nodes = order.get("nodes", [])
        released_nodes = [n for n in nodes if n.get("released", False)]

        # 3. Sort nodes by sequenceId (execution order)
        released_nodes.sort(key=lambda n: n.get("sequenceId", 999))

        # 4. Convert nodes to route (waypoints)
        route = []
        for n in released_nodes:
            pos = n.get("nodePosition", {})
            x = pos.get("x")
            y = pos.get("y")
            route.append((x, y))

        # Save internally
        self.nodes = released_nodes
        self.route = route

        # 5. Print route
        print("Route (waypoints):")
        for wp in route:
            print(f" - {wp}")

        print("===== END ORDER =====\n")


def on_connect(client, userdata, flags, rc):
    client.subscribe(userdata["topic_order"])

def on_message(client, userdata, msg):
    robot: Robot = userdata["robot"]

    if msg.topic == userdata["topic_order"]:
        robot.receive_order(msg)

def main():
    robot_name = "mouse001"
    topic_order = f"uagv/v2/KIT/{robot_name}/order"

    robot = Robot(robot_name)

    client = mqtt.Client(userdata={
        "robot": robot,
        "topic_order": topic_order
    })

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
