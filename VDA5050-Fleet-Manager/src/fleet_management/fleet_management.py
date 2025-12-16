# 【文件名：src/fleet_management/fleet_management.py (V74)】
import math
import heapq
import time
import threading

class FleetManagement:
    def __init__(self, config_data, graph, agents, task_management, simulation_start_time, current_task_id: int = 9) -> None:
        self.simulation_start_time = simulation_start_time
        self.config_data = config_data
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.current_task_id = current_task_id 

        # --- 配置 ---
        self.agent_diameter = self.config_data.get('agent_rotation_diameter', 1.0)
        # 安全距离
        self.min_safe_distance = self.agent_diameter + 0.3
        
        # [TUNING] 统一的延迟释放时间 (秒)
        # 无论是否转向，离开节点后，该节点依然被锁定 2 秒，防止后车追尾
        self.SAFETY_TAIL_TIME = 2.0 

        self.reservations = {} 
        self.edge_lookup = {}
        self._precompute_edge_lookup()
        self.spatial_conflicts = self._precompute_spatial_conflicts()
        self.planning_lock = threading.Lock()
        
        self.fleet_manager(current_task_id=self.current_task_id)

    def _precompute_edge_lookup(self):
        for eid, einfo in self.graph.edges.items():
            u = einfo['startNodeId']
            v = einfo['endNodeId']
            self.edge_lookup[(u, v)] = {'edgeId': eid, 'startNodeId': u, 'endNodeId': v, 'actions': []}
            self.edge_lookup[(v, u)] = {'edgeId': f"{eid}_reverse", 'startNodeId': v, 'endNodeId': u, 'actions': []}

    def _precompute_spatial_conflicts(self):
        conflicts = {}
        nodes = list(self.graph.nodes.values())
        for i in range(len(nodes)):
            id_i = nodes[i]['nodeId']
            pos_i = nodes[i].get('position', {'x': 0, 'y': 0})
            conflicts[id_i] = []
            for j in range(len(nodes)):
                if i == j: continue
                id_j = nodes[j]['nodeId']
                pos_j = nodes[j].get('position', {'x': 0, 'y': 0})
                dx = pos_i['x'] - pos_j['x']
                dy = pos_i['y'] - pos_j['y']
                dist = math.sqrt(dx*dx + dy*dy)
                # 稍微扩大一点冲突检测范围
                if dist < (self.min_safe_distance + 0.1):
                    conflicts[id_i].append(id_j)
        return conflicts

    def get_time_now(self):
        return time.time() - self.simulation_start_time

    def is_node_free_space_time(self, node_id, start_time, end_time, my_agent_id=None, dwelling_occupancy=None):
        # 0. Dwelling Lock
        if dwelling_occupancy:
            owner = dwelling_occupancy.get(node_id)
            if owner and str(owner) != str(my_agent_id):
                return False, f"DWELL_LOCK by {owner}"

        # 1. Reservation Check
        res_ok, res_reason = self._check_reservation_table(node_id, start_time, end_time, my_agent_id)
        if not res_ok:
            return False, res_reason

        # 2. Physical Check
        phy_ok, phy_reason = self._check_physical_presence(node_id, my_agent_id)
        if not phy_ok:
            return False, phy_reason

        # 3. Spatial Neighbors Check
        if node_id in self.spatial_conflicts:
            for conflict_node_id in self.spatial_conflicts[node_id]:
                if dwelling_occupancy:
                    owner = dwelling_occupancy.get(conflict_node_id)
                    if owner and str(owner) != str(my_agent_id):
                        return False, f"NEIGHBOR_DWELL {conflict_node_id} by {owner}"

                res_ok_n, res_reason_n = self._check_reservation_table(conflict_node_id, start_time, end_time, my_agent_id)
                if not res_ok_n:
                    return False, f"NEIGHBOR_RES {conflict_node_id}: {res_reason_n}"
                
                phy_ok_n, phy_reason_n = self._check_physical_presence(conflict_node_id, my_agent_id)
                if not phy_ok_n:
                    return False, f"NEIGHBOR_PHY {conflict_node_id}: {phy_reason_n}"
                    
        return True, "OK"

    def _check_reservation_table(self, node_id, start_time, end_time, my_agent_id):
        if node_id not in self.reservations:
            return True, "OK"
        my_str = str(my_agent_id)
        for r_start, r_end, r_agent in self.reservations[node_id]:
            if str(r_agent) == my_str: continue
            if not (end_time <= r_start or start_time >= r_end):
                return False, f"RESERVED by {r_agent}"
        return True, "OK"

    def _check_physical_presence(self, target_node_id, my_agent_id):
        target_node = self.graph.nodes.get(target_node_id)
        if not target_node: return True, "OK"
        target_pos = target_node['position']
        my_str = str(my_agent_id)

        for other_agent in self.agents.agents:
            if str(other_agent.agentId) == my_str: continue
            
            # 1. 节点占用
            other_node = getattr(other_agent, 'last_node_id', getattr(other_agent, 'current_node_id', None))
            if other_node == target_node_id: 
                return False, f"OCCUPIED by {other_agent.agentId}"
            
            # 2. 距离占用
            other_pos = other_agent.agvPosition
            if not other_pos: continue
            
            dx = target_pos['x'] - other_pos.get('x', 0)
            dy = target_pos['y'] - other_pos.get('y', 0)
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < self.min_safe_distance:
                return False, f"COLLISION RISK {other_agent.agentId} (dist={dist:.2f})"
                
        return True, "OK"

    def reserve_path(self, path_nodes_with_time, agent_id):
        for node_id, entry_time, exit_time in path_nodes_with_time:
            if node_id not in self.reservations:
                self.reservations[node_id] = []
            self.reservations[node_id].append((entry_time, exit_time, agent_id))

    def cancel_reservations(self, agent_id):
        with self.planning_lock:
            my_str = str(agent_id)
            keys_to_clean = []
            for node_id, reservations in self.reservations.items():
                new_reservations = [r for r in reservations if str(r[2]) != my_str]
                self.reservations[node_id] = new_reservations
                if not new_reservations:
                    keys_to_clean.append(node_id)
            for k in keys_to_clean:
                del self.reservations[k]

    def space_time_astar(self, start_node_id: str, goal_node_id: str, agent_velocity: float, current_time: float, agent_id: str, avoid_node: str = None, dwelling_occupancy: dict = None) -> list:
        with self.planning_lock:
            return self._astar_impl(start_node_id, goal_node_id, agent_velocity, current_time, agent_id, avoid_node, dwelling_occupancy)

    def _astar_impl(self, start_node_id, goal_node_id, agent_velocity, current_time, agent_id, avoid_node=None, dwelling_occupancy=None):
        if start_node_id not in self.graph.nodes or goal_node_id not in self.graph.nodes:
            return []

        def heuristic(node1_id, node2_id):
            return self._get_dist_manhattan(node1_id, node2_id) / agent_velocity

        def get_dist(node1_id, node2_id):
            pos1 = self.graph.nodes[node1_id]['position']
            pos2 = self.graph.nodes[node2_id]['position']
            return math.sqrt((pos1['x']-pos2['x'])**2 + (pos1['y']-pos2['y'])**2)
        
        def to_key_time(t): return round(t, 1)

        open_set = [(heuristic(start_node_id, goal_node_id), start_node_id, current_time)]
        came_from = {}
        start_key = (start_node_id, to_key_time(current_time))
        g_score = {start_key: 0}
        visited = set()
        
        # A* 搜索时的节点占用缓冲，防止规划出太紧凑的路径
        SEARCH_BUFFER = 1.0 
        
        nodes_explored = 0
        MAX_EXPLORE = 4000 

        while open_set:
            current_f, current_node, current_t_exact = heapq.heappop(open_set)
            current_t_key = to_key_time(current_t_exact)
            
            nodes_explored += 1
            if nodes_explored > MAX_EXPLORE: break

            if current_node == goal_node_id:
                return self._reconstruct_path(came_from, current_node, current_t_key, start_node_id, current_time, agent_id, agent_velocity)

            state_key = (current_node, current_t_key)
            if state_key in visited: continue
            visited.add(state_key)
            
            current_neighbors = []
            for edge_id, edge_info in self.graph.edges.items():
                if edge_info.get('startNodeId') == current_node: current_neighbors.append(edge_info.get('endNodeId'))
                elif edge_info.get('endNodeId') == current_node: current_neighbors.append(edge_info.get('startNodeId'))

            # Action 1: Move
            for neighbor in current_neighbors:
                if avoid_node and neighbor == avoid_node: continue

                dist = get_dist(current_node, neighbor)
                travel_time = dist / agent_velocity
                arrival_time_exact = current_t_exact + travel_time
                
                is_free, _ = self.is_node_free_space_time(
                    neighbor, current_t_exact, arrival_time_exact + SEARCH_BUFFER, agent_id, dwelling_occupancy
                )
                
                if is_free:
                    neighbor_key = (neighbor, to_key_time(arrival_time_exact))
                    tentative_g = (current_t_exact - current_time) + travel_time
                    if neighbor_key not in g_score or tentative_g < g_score.get(neighbor_key, float('inf')):
                        came_from[neighbor_key] = (current_node, current_t_key)
                        g_score[neighbor_key] = tentative_g
                        f_score = tentative_g + heuristic(neighbor, goal_node_id)
                        heapq.heappush(open_set, (f_score, neighbor, arrival_time_exact))

            # Action 2: Wait
            wait_duration = 1.0
            next_t_exact = current_t_exact + wait_duration
            
            is_free_wait, _ = self.is_node_free_space_time(current_node, current_t_exact, next_t_exact + SEARCH_BUFFER, agent_id, dwelling_occupancy)
            
            if is_free_wait:
                wait_key = (current_node, to_key_time(next_t_exact))
                tentative_g_wait = (current_t_exact - current_time) + wait_duration 
                if wait_key not in g_score or tentative_g_wait < g_score.get(wait_key, float('inf')):
                    came_from[wait_key] = (current_node, current_t_key)
                    g_score[wait_key] = tentative_g_wait
                    f_score = tentative_g_wait + heuristic(current_node, goal_node_id)
                    heapq.heappush(open_set, (f_score, current_node, next_t_exact))

        return []

    def _get_dist_manhattan(self, n1, n2):
        if n1 not in self.graph.nodes or n2 not in self.graph.nodes: return float('inf')
        pos1 = self.graph.nodes[n1]['position']
        pos2 = self.graph.nodes[n2]['position']
        return abs(pos1['x'] - pos2['x']) + abs(pos1['y'] - pos2['y'])

    def _reconstruct_path(self, came_from, current_node, current_t_key, start_node, start_time, agent_id, velocity):
        path_segments = []
        curr_key = (current_node, current_t_key)
        while curr_key in came_from:
            node_id = curr_key[0]
            arrival_time = curr_key[1]
            path_segments.append({'nodeId': node_id, 'arrival': arrival_time})
            curr_key = came_from[curr_key]
        path_segments.append({'nodeId': start_node, 'arrival': start_time})
        path_segments.reverse()
        
        final_node_ids = []
        reservation_list = []
        
        GOAL_OCCUPANCY = 0.5
        
        for i in range(len(path_segments)):
            node_id = path_segments[i]['nodeId']
            arrival = path_segments[i]['arrival']
            
            # 计算离开时间
            if i < len(path_segments) - 1:
                departure = path_segments[i+1]['arrival']
            else:
                departure = arrival + GOAL_OCCUPANCY
            
            start_res = path_segments[i-1]['arrival'] if i > 0 else arrival
            
            # [关键修复] 统一的、固定的尾部缓冲时间
            # 无论什么情况，节点都要多占用 SAFETY_TAIL_TIME (2.0s)
            end_res = departure + self.SAFETY_TAIL_TIME
            
            reservation_list.append((node_id, start_res, end_res))
            
            if i == 0 or node_id != final_node_ids[-1]:
                final_node_ids.append(node_id)
        
        self.reserve_path(reservation_list, agent_id)
        return final_node_ids

    def path_to_nodes_and_edges(self, path: list) -> tuple:
        path_nodes = []
        path_edges = []
        if not path: return path_nodes, path_edges
        clean_path = [path[0]]
        for i in range(1, len(path)):
            if path[i] != path[i-1]:
                clean_path.append(path[i])
        for node_id in clean_path:
            path_nodes.append({'nodeId': node_id, 'actions': []})
        for i in range(len(clean_path) - 1):
            start = clean_path[i]
            end = clean_path[i+1]
            edge_obj = self.edge_lookup.get((start, end))
            if not edge_obj:
                edge_obj = {'edgeId': f"{start}-{end}_dummy", 'startNodeId': start, 'endNodeId': end, 'actions': []}
            edge_copy = edge_obj.copy()
            edge_copy['actions'] = [] 
            path_edges.append(edge_copy)
        return path_nodes, path_edges

    def fleet_manager(self, current_task_id: int = 9) -> None:
        pass