# 【文件名：Agents.py (Version: Hybrid for V82)】
import math
import json
import time
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber
from vda5050_interface.interfaces.order_interface import OrderInterface


class Agents:
    def __init__(self, config_data, graph, agents_initialization_data, logging, simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.logging = logging
        self.config_data = config_data
        self.graph = graph
        self.order_header_id = 1
        self.agents = self.get_agents(agents_initialization_data)

    def get_agents(self, agents_initialization_data) -> list:
        agents = []
        for agent_data in agents_initialization_data.get('agents', []):
            agent = Agent(
                agents=self,
                agentId=agent_data.get('agentId'),
                agent_state='IDLE',
                agent_order_topic=agent_data.get('orderTopic'),
                agent_state_topic=agent_data.get('stateTopic'),
                agent_position=agent_data.get('agentPosition', {}),
                agent_velocity=agent_data.get('agentVelocity', 1.0),
                agent_rotation_velocity=agent_data.get('agentRotationVelocity', 3.0),
                logging=self.logging
            )
            agents.append(agent)
        return agents


class Agent:
    def __init__(self, agents, agentId, agent_state_topic, agent_order_topic, agent_state, agent_position, agent_velocity, agent_rotation_velocity, logging) -> None:
        self.agents = agents
        self.agentId = agentId
        self.agent_state = agent_state
        self.agvPosition = agent_position.copy() if agent_position else {}
        self.safetyState = {}
        self.loaded = False
        self.state_topic = agent_state_topic
        self.order_topic = agent_order_topic
        self.logging = logging
        self.agent_velocity = agent_velocity
        self.agent_rotation_velocity = agent_rotation_velocity
        
        self.current_node_id = None
        self.current_task = None 
        self.last_node_id = None 
        self.last_position = {
            'x': agent_position.get('x', 0.0),
            'y': agent_position.get('y', 0.0)
        } if agent_position else {'x': 0.0, 'y': 0.0}
        
        self.last_position_update_time = time.time()

        self.mqtt_subscriber_state = MQTTSubscriber(config_data=self.agents.config_data, logging=self.logging, on_message=self.state_callback,
                                                    channel=self.state_topic, client_id=f'state_subscriber_agent_{self.agentId}')
        self.order_interface = OrderInterface(config_data=self.agents.config_data, logging=logging,
                                              order_topic=self.order_topic, agentId=self.agentId)
        self.moved_distance = 0.0
    
    def state_callback(self, client, userdata, msg) -> None:
        # self.logging.info(f"Client {self.mqtt_subscriber_state.client_id} received message from topic `{msg.topic}`.") # 可注释掉减少刷屏

        try:
            state_data = json.loads(msg.payload.decode("utf-8"))
            
            # --- 1. 位置更新 (Position Update) ---
            current_x = self.last_position['x']
            current_y = self.last_position['y']

            agv_position = state_data.get('agvPosition', {})
            if agv_position:
                current_x = agv_position.get('x', self.last_position['x'])
                current_y = agv_position.get('y', self.last_position['y'])
                
                dx = current_x - self.last_position['x']
                dy = current_y - self.last_position['y']
                distance = math.sqrt(dx * dx + dy * dy)
                self.moved_distance += distance
                
                self.agvPosition = agv_position
                self.last_position = {'x': current_x, 'y': current_y}
                self.last_position_update_time = time.time()
            
            last_node_id = state_data.get('lastNodeId')
            if last_node_id:
                self.last_node_id = last_node_id
                self.current_node_id = last_node_id

            # --- 2. 状态与负载同步 (配合 V82 的关键部分) ---
            # 即使车不发 FINISHED，我们也要根据位置强制修正 loaded 状态
            # 这样 V82 才能正确判断是该去 Pick 还是去 Drop
            if self.current_task:
                start_station = self.agents.graph.stations.get(self.current_task.get('startStationId'))
                goal_station = self.agents.graph.stations.get(self.current_task.get('goalStationId'))
                
                pick_nodes = start_station.get('interactionNodeIds', []) if start_station else []
                drop_nodes = goal_station.get('interactionNodeIds', []) if goal_station else []

                # 如果到了 Pick 点，强制视为已装载
                if self.last_node_id in pick_nodes:
                    self.loaded = True
                
                # 如果到了 Drop 点，强制视为已卸载
                if self.last_node_id in drop_nodes:
                    self.loaded = False

            # 依然保留标准协议的读取，以防万一车修好了
            action_states = state_data.get('actionStates', [])
            for action in action_states:
                if action.get('actionStatus') == 'FINISHED':
                    if action.get('actionType') == 'pick': self.loaded = True
                    elif action.get('actionType') == 'drop': self.loaded = False

            # --- 3. 运行状态更新 ---
            # 注意：这里我们不急着把 EXECUTING 改回 IDLE，除非车真的停了
            # 真正的“任务结束”逻辑留给 V82 的 3秒计时器去处理
            if state_data.get('driving', False):
                if self.agent_state == 'IDLE':
                    self.agent_state = 'EXECUTING'
            else:
                # 只有当标准协议明确说 FINISHED 时才在这里转 IDLE
                # 否则保持 EXECUTING，直到 V82 强制介入
                all_finished = all(action.get('actionStatus') == 'FINISHED' for action in action_states)
                if all_finished and not state_data.get('driving', False):
                    self.agent_state = 'IDLE'
            
            self.safetyState = state_data.get('safetyState', {})
            
            # 这里的 Task Completion 逻辑保留给标准情况
            # 如果是死循环 Bug 情况，这段代码不会触发，而是由 V82 触发
            if (self.loaded == False and 
                not state_data.get('driving', False) and 
                self.current_task and 
                self.last_node_id):
                
                goal_station_id = self.current_task.get('goalStationId')
                if goal_station_id:
                    goal_station = self.agents.graph.stations.get(goal_station_id)
                    if goal_station and self.last_node_id in goal_station.get('interactionNodeIds', []):
                        # 检查是否有显式的 FINISHED 信号
                        drop_action_finished = any(a.get('actionType') == 'drop' and a.get('actionStatus') == 'FINISHED' for a in action_states)
                        if drop_action_finished:
                            self.current_task['task_completed'] = True
                            self.agent_state = 'IDLE'
                            self.current_task = None
                            self.logging.info(f"Agent {self.agentId} completed task (Standard Protocol)")
            
            if self.moved_distance >= 1.0 and int(self.moved_distance) % 1 == 0:
                self.logging.info(f"Agent {self.agentId} has moved {round(self.moved_distance, 4)} meters.")
                
        except json.JSONDecodeError as e:
            self.logging.error(f"Error parsing state message: {e}")
        except Exception as e:
            self.logging.error(f"Error processing state message: {e}")