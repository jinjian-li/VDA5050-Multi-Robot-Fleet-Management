import paho.mqtt.client as mqtt
import json
import pygame
import argparse

# MQTT Configuration
MQTT_BROKER = "localhost"  # Address of the MQTT broker
MQTT_PORT = 1883           # Default MQTT port for unencrypted communication

# Limits for the robot's speed
MAX_LINEAR_SPEED = 0.2  # Maximum linear speed in meters per second
MAX_ANGULAR_SPEED = 1   # Maximum angular speed in radians per second

# Twist command structure used for motion control
cmd_twist = {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
}

def handle_keyboard_input():
    """
    Capture keyboard inputs and update the twist command accordingly.
    """
    keys = pygame.key.get_pressed()

    # Reset the twist command to prevent unwanted motion persistence
    cmd_twist["linear"]["x"] = 0.0
    cmd_twist["angular"]["z"] = 0.0

    # Move forward if UP arrow key is pressed, backward if DOWN arrow key is pressed
    if keys[pygame.K_UP]:
        cmd_twist["linear"]["x"] = MAX_LINEAR_SPEED
    elif keys[pygame.K_DOWN]:
        cmd_twist["linear"]["x"] = -MAX_LINEAR_SPEED

    # Rotate left or right using LEFT and RIGHT arrow keys respectively
    if keys[pygame.K_LEFT]:
        cmd_twist["angular"]["z"] = MAX_ANGULAR_SPEED
    elif keys[pygame.K_RIGHT]:
        cmd_twist["angular"]["z"] = -MAX_ANGULAR_SPEED

    return cmd_twist

def publish_dict_to_mqtt(robot_name):
    """
    Initialize the MQTT client, capture keyboard input, and publish robot commands.
    """
    # Define the MQTT topic based on the robot's name
    mqtt_topic_cmd = "uagv/v2/KIT/" + robot_name + "/cmd"

    # Create a new MQTT client instance
    client = mqtt.Client()
    try:
        # Attempt to connect to the specified MQTT broker
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print(f"Connected to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}")
    except Exception as e:
        # Print an error message if connection fails and exit the function
        print(f"Error connecting to MQTT Broker: {e}")
        return

    # Initialize the Pygame library
    pygame.init()

    # Create a basic Pygame window
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Robot Control")

    # Main loop for handling events and publishing commands
    running = True
    clock = pygame.time.Clock()  # Used for controlling the frame rate

    while running:
        # Check for Pygame events (like quitting the window)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Capture keyboard input and generate twist commands
        twist_command = handle_keyboard_input()

        # Convert the twist command dictionary to a JSON string and publish it via MQTT
        client.publish(mqtt_topic_cmd, json.dumps(twist_command))

        # Control the update rate of the loop (20 frames per second)
        clock.tick(20)

    # Disconnect from the MQTT broker and close Pygame upon exiting the loop
    client.disconnect()
    print("Disconnected from MQTT Broker")
    pygame.quit()

def main(robot):
    """
    Main entry point for the script, which starts the MQTT control loop for the specified robot.
    """
    publish_dict_to_mqtt(robot)

if __name__ == "__main__":
    # Setup command-line argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, help="robot name", default='mouse001')
    args = parser.parse_args()
    
    # Start the main function with the specified robot name
    main(args.robot)
