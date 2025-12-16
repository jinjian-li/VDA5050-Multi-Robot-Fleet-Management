# 🚀 VDA5050 Multi-Robot Fleet Management System

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://www.python.org/)
[![Protocol](https://img.shields.io/badge/Protocol-VDA5050-orange)](https://github.com/VDA5050/VDA5050)
[![Middleware](https://img.shields.io/badge/Middleware-MQTT-green)](https://mqtt.org/)

## 📖 Overview

This repository implements the core logic of a **Centralized Fleet Management System** for industrial Autonomous Mobile Robots (AMRs). 

The system is designed to comply with the **VDA 5050 Standard** (Interface for communication between AGVs and a master control), enabling interoperability between different robot manufacturers via **MQTT**.

> **Note:** This is a code portfolio derived from a Robotics Lab project. It demonstrates the **Task Scheduling**, **Traffic Control**, and **VDA Protocol Parsing** logic. 
> *Actual map data and proprietary libraries are excluded for IP protection.*

---

## 🏗️ System Architecture & Code Map

The project is organized into three modules located in the `src/` directory.

### 1. Fleet Server (The "Brain") 🧠
Centralized logic for managing multiple robots.
* **[📄 fleet_management.py](src/fleet_server/fleet_management.py)**
    * **Traffic Manager:** Implements a reservation-based locking mechanism to prevent deadlocks.
    * **Safety Logic:** Features **Safety Tail Time** (`SAFETY_TAIL_TIME = 2.0s`) to prevent rear-end collisions.
* **[📄 task_assignment.py](src/fleet_server/task_assignment.py)**
    * **Task Dispatcher:** Assigns orders to the most suitable idle agent.
    * **Safety Guard:** Runs a background thread (`safety_guard_loop`) to monitor physical proximity and trigger E-Stops.

### 2. Robot Agent (The "Edge") 🚜
Client-side logic running on the AGV.
* **[📄 task_1.py](src/robot_agent/task_1.py)** *(VDA Parser)*
    * **Protocol Parsing:** Decodes JSON payloads (`Order`, `Nodes`, `Edges`) into actionable waypoints.
* **[📄 task_2_3.py](src/robot_agent/task_2_3.py)** *(Local Controller)*
    * **Motion Control:** Executes trajectory following.
    * **State Reporting:** Publishes `Position` and `State` updates back to the server via MQTT.

### 3. Shared Data Structures ⚙️
Common definitions used by both server and agents.
* **[📄 graph.py](src/shared/graph.py)**: Topological map definitions.
* **[📄 agent.py](src/shared/agent.py)**: Robot state machine classes.

---

## 💻 Key Technical Highlights

### Safety Guard Mechanism
A dedicated background thread ensures operational safety beyond the planning layer.
*(Snippet from `task_assignment.py`)*:
```python
def safety_guard_loop(self):
    """Continuously monitors physical proximity to trigger E-Stop."""
    while True:
        self._check_physical_collisions()
        time.sleep(0.05)
