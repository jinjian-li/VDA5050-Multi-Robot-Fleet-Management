import threading
import time
import math
import random
import traceback

class TaskAssignment:
    def __init__(self, graph, agents, task_management, simulation_start_time, fleet_management) -> None:
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.fleet_management = fleet_management
        
        # 1. 基础配置
        self.dwelling_nodes = []
        if hasattr(self.graph, 'dwelling_nodes'):
            self.dwelling_nodes = self.graph.dwelling_nodes
        else:
            for station in self.graph.stations.values():
                if station.get('stationType') == 'CHARGING':
                    self.dwelling_nodes.extend(station.get('interactionNodeIds', []))

        # 2. 状态缓存
        self.dwelling_occupancy = {}
        self.agent_next_hop = {}
        self.agent_current_node_cache = {} 
        self.failure_counts = {} 
        self.last_dodge_history = {} # 留着，但不再用于 DODGE 逻辑
        self.collision_cooldowns = {} # 新增，用于避免立即再次碰撞后的状态重置

        # 3. 锁与冷却
        self.safety_lockout = {} 
        self.post_task_cooldowns = {}
        self.eviction_cooldowns = {}
        self.stop_command_cooldowns = {}
        self.last_auction_time = 0
        
        # 4. 权威锁
        self.action_lock = threading.Lock()
        self.safety_override_until = 0
        
        # 5. 启动线程
        self.safety_thread = threading.Thread(target=self.safety_guard_loop, daemon=True)
        self.safety_thread.start()
        
        self.assignment_thread = threading.Thread(target=self.assign_tasks_loop, daemon=True)
        self.assignment_thread.start()

    # =========================================================================
    # 辅助函数 (A-Z)
    # =========================================================================
    
    def _check_action_done(self, agent, action_type):
        task = agent.current_task
        if not task: return False
        suffix = f"p-{task['task_id']}" if action_type == 'pick' else f"d-{task['task_id']}"
        actions = getattr(agent, 'actionStates', [])
        for a in actions:
            if a['actionId'] == suffix and a['actionStatus'] == 'FINISHED': return True
        return False
        
    def _check_idle_blockers(self, now):
        for agent in self.agents.agents:
            # 只有 IDLE 且有非 Dwelling 任务的 AGV 才需要检查它是否被阻塞
            if not agent.current_task or agent.agent_state != 'IDLE': continue
            if agent.current_task.get('task_id') == 'Dwelling': continue
            
            target = self._get_target_node(agent)
            if not target: continue
            
            obs = self._get_dynamic_obstacles(agent.agentId)
            if target in obs:
                blocker_id = obs[target]
                b_agent = next((a for a in self.agents.agents if str(a.agentId) == blocker_id), None)
                
                # 检查阻塞物 (b_agent) 是否在 Dwelling 任务中或没有任务
                if b_agent and (b_agent.current_task is None or b_agent.current_task.get('task_id') == 'Dwelling'):
                    # 检查是否处于安全锁定或驱逐冷却期
                    if now < self.safety_lockout.get(blocker_id, 0): continue
                    
                    if now > self.eviction_cooldowns.get(blocker_id, 0):
                        self.agents.logging.warning(f"Evicting IDLE blocker {blocker_id} from {target} for agent {agent.agentId}")
                        # 发送 Dwelling 任务，并排除当前阻塞的节点
                        self._send_to_dwelling(b_agent, exclude=target)
                        self.eviction_cooldowns[blocker_id] = now + 10.0 # 10秒驱逐冷却

    def _check_physical_collisions(self):
        now = time.time()
        for agent in self.agents.agents:
            aid = str(agent.agentId)
            
            # 如果 AGV 正在被锁定，跳过安全检查
            if now < self.safety_lockout.get(aid, 0): continue
            
            # 仅在移动中或准备移动时检查
            if not getattr(agent, 'driving', False) and agent.agent_state == 'IDLE': continue

            threat = self._find_nearest_threat(agent)
            
            if threat and agent.agvPosition and threat.agvPosition:
                dist = math.sqrt((agent.agvPosition['x']-threat.agvPosition['x'])**2 + (agent.agvPosition['y']-threat.agvPosition['y'])**2)
                
                if dist < 1.0: # 迫在眉睫的碰撞
                    self.agents.logging.critical(f"COLLISION IMMINENT: {aid} vs {threat.agentId}. Sending CRITICAL STOP and cancelling task.")
                    
                    # 尝试发送空订单来强制停止
                    try:
                        stop_order_id = f"CRITICAL_STOP_{aid}_{int(time.time()*100)}"
                        agent.order_interface.generate_order_message(
                            agent=agent, orderId=stop_order_id, order_updateId=0, nodes=[], edges=[]
                        )
                    except Exception as e:
                        self.agents.logging.error(f"Failed to send CRITICAL STOP order for {aid}: {e}")

                    self.safety_lockout[aid] = now + 5.0 # 强制停止 5 秒
                    
                    # 立即清理上层状态，使其进入 IDLE，同时增加冷却时间
                    if agent.current_task:
                        if agent.current_task.get('task_id') != 'Dwelling':
                            if agent.current_task.get('task_assigned'):
                                agent.current_task['task_assigned'] = False
                                agent.current_task['agent_id'] = None
                            self.agents.logging.critical(f"Task {agent.current_task.get('task_id')} cancelled due to collision threat.")
                        
                        self._finish_task(agent) 
                        self.post_task_cooldowns[aid] = now + 5.0 # 碰撞后强制冷却 5 秒

                    if now > self.stop_command_cooldowns.get(aid, 0):
                        self.stop_command_cooldowns[aid] = now + 1.0 
                        self.fleet_management.cancel_reservations(aid)
                        
                elif dist < 2.5: # 潜在碰撞：强制进入冷却状态，打破发单循环
                    # 如果 AGV 正在执行，并且有潜在碰撞风险
                    if agent.agent_state == 'EXECUTING' and agent.current_task and agent.current_task.get('task_id') != 'Dwelling':
                        
                        # 检查是否已处于冷却期
                        if now > self.post_task_cooldowns.get(aid, 0):
                            self.agents.logging.warning(f"EXECUTING AGV {aid} near threat {threat.agentId}. Applying temporary COOLDOWN.")
                            
                            # 仅应用冷却时间，不改变任务/状态，等待 AGV 状态更新或死锁检测接管
                            self.post_task_cooldowns[aid] = now + 0.5  # 短暂冷却 0.5s，打破快速重发循环
                            self.fleet_management.cancel_reservations(aid)

    def _do_task_logic(self):
        self._refresh_node_cache()
        now = time.time()
        
        for agent in self.agents.agents:
            aid = str(agent.agentId)
            if now < self.safety_lockout.get(aid, 0): continue
            if now < self.post_task_cooldowns.get(aid, 0): continue

            # --- 状态机 ---
            if agent.agent_state == 'EXECUTING':
                if getattr(agent, 'driving', False):
                    self.failure_counts[aid] = 0

                if agent.current_task:
                    is_dwelling = (agent.current_task.get('task_id') == 'Dwelling')
                    if is_dwelling:
                        curr = self.agent_current_node_cache.get(aid)
                        target = agent.current_task.get('dummy_target_node')
                        if curr == target and not getattr(agent, 'driving', True):
                            self._finish_task(agent)
                    else:
                        if not agent.loaded and self._check_action_done(agent, 'pick'):
                            self.agents.logging.info(f"Agent {aid} PICK Done.")
                            agent.loaded = True
                            agent.agent_state = 'IDLE' 
                            self.fleet_management.cancel_reservations(aid)
                            self.post_task_cooldowns[aid] = now + 1.0
                        elif agent.loaded and self._check_action_done(agent, 'drop'):
                            self.agents.logging.info(f"Agent {aid} DROP Done.")
                            agent.loaded = False
                            self._finish_task(agent)
                            self.post_task_cooldowns[aid] = now + 1.0

            elif agent.agent_state == 'IDLE':
                if agent.current_task and not agent.current_task.get('task_id') == 'Dwelling':
                    success = self._plan_mission(agent, agent.current_task)
                    if not success:
                        self.failure_counts[aid] = self.failure_counts.get(aid, 0) + 1
                        
                        # 长期死锁检测：如果规划连续失败，延长冷却时间
                        if self.failure_counts[aid] >= 5: 
                            self.agents.logging.error(f"Long-term deadlock detected for {aid}. Cancelling task and returning to dwelling.")
                            
                            if agent.current_task.get('task_assigned'):
                                agent.current_task['task_assigned'] = False
                                agent.current_task['agent_id'] = None
                                
                            self.failure_counts[aid] = 0
                            # 【强化冷却至 10.0 秒】
                            self.post_task_cooldowns[aid] = now + 10.0 
                            self._send_to_dwelling(agent)
                    else:
                        self.failure_counts[aid] = 0
                elif not agent.current_task:
                    self._send_to_dwelling(agent)

        if now - self.last_auction_time > 1.0:
            self._run_auction()
            self.last_auction_time = now
        
        self._check_idle_blockers(now)


    def _find_nearest_threat(self, agent):
        my_pos = agent.agvPosition
        if not my_pos: return None
        nearest = None
        min_d = float('inf')
        
        for other in self.agents.agents:
            if other == agent: continue
            op = other.agvPosition
            if not op: continue
            d = math.sqrt((my_pos['x']-op['x'])**2 + (my_pos['y']-op['y'])**2)
            if d < min_d:
                min_d = d
                nearest = other
        
        if min_d < 3.0: return nearest
        return None

    def _finish_task(self, agent):
        if agent.current_task and agent.current_task.get('task_id') != 'Dwelling':
            agent.current_task['task_completed'] = True
        self.fleet_management.cancel_reservations(str(agent.agentId))
        agent.current_task = None
        agent.agent_state = 'IDLE'
        if str(agent.agentId) in self.agent_next_hop:
            del self.agent_next_hop[str(agent.agentId)]

    def _get_dynamic_obstacles(self, my_agent_id):
        obs = {}
        my_aid = str(my_agent_id)
        other_positions = []
        for a in self.agents.agents:
            if str(a.agentId) == my_aid: continue
            
            curr = self.agent_current_node_cache.get(str(a.agentId))
            nxt = self.agent_next_hop.get(str(a.agentId))
            if curr: obs[curr] = str(a.agentId)
            if nxt: obs[nxt] = str(a.agentId)
            
            pos = a.agvPosition
            if pos: other_positions.append({'x': pos['x'], 'y': pos['y'], 'id': str(a.agentId)})

        if not other_positions: return obs

        SAFE_RADIUS = 2.0 # 扩大安全半径
        
        for nid, ninfo in self.graph.nodes.items():
            nx, ny = ninfo['position']['x'], ninfo['position']['y']
            for other in other_positions:
                dist = math.sqrt((nx - other['x'])**2 + (ny - other['y'])**2)
                if dist < SAFE_RADIUS:
                    obs[nid] = other['id']
                    break 
        return obs
        
    
    def _get_or_find_current_node(self, agent):
        if agent.last_node_id and agent.last_node_id in self.graph.nodes: 
            return agent.last_node_id
        agent_pos = agent.agvPosition
        if not agent_pos: return None
        min_dist = float('inf')
        nearest = None
        for nid, ninfo in self.graph.nodes.items():
            npos = ninfo.get('position', {})
            d = math.sqrt((agent_pos['x']-npos['x'])**2 + (agent_pos['y']-npos['y'])**2)
            if d < min_dist:
                min_dist = d
                nearest = nid
        if min_dist > 3.0: return None
        return nearest

    def _get_target_node(self, agent):
        if not agent.current_task: return None
        t = agent.current_task
        if t.get('task_id') == 'Dwelling': return t.get('dummy_target_node')
        s = self.graph.stations.get(t.get('startStationId'))
        g = self.graph.stations.get(t.get('goalStationId'))
        if not s or not g: return None
        return g.get('interactionNodeIds')[0] if agent.loaded else s.get('interactionNodeIds')[0]

    def _plan_mission(self, agent, task):
        aid = str(agent.agentId)
        
        # 1. 起步物理检查 (3.0m)
        my_pos = agent.agvPosition
        if my_pos:
            for other in self.agents.agents:
                if other == agent: continue
                op = other.agvPosition
                if not op: continue
                d = math.sqrt((my_pos['x']-op['x'])**2 + (my_pos['y']-op['y'])**2)
                
                if d < 3.0:
                    if str(other.agentId) < aid or getattr(other, 'driving', False):
                        return False

        curr_node = self.agent_current_node_cache.get(aid)
        if not curr_node: curr_node = self._get_or_find_current_node(agent)
        if not curr_node: return False
        
        target_node = None
        pick_node = None
        drop_node = None
        
        if task.get('task_id') == 'Dwelling':
            target_node = task.get('dummy_target_node')
            # 【新增互斥检查】
            # 如果目标 Dwelling 节点已被占用，且不是当前 AGV 占用的，则规划失败
            if target_node in self.dwelling_occupancy and self.dwelling_occupancy[target_node] != aid:
                 self.agents.logging.warning(f"Dwelling target {target_node} occupied by {self.dwelling_occupancy[target_node]}. Planning failed.")
                 return False
        else:
            s_station = self.graph.stations.get(task.get('startStationId'))
            g_station = self.graph.stations.get(task.get('goalStationId'))
            if not s_station or not g_station: return False
            pick_node = s_station.get('interactionNodeIds', [None])[0]
            drop_node = g_station.get('interactionNodeIds', [None])[0]
            target_node = drop_node if agent.loaded else pick_node

        # 2. 获取障碍物
        obs = self._get_dynamic_obstacles(aid)
        if curr_node in obs: return False

        self.fleet_management.cancel_reservations(aid)
        path = self.fleet_management.space_time_astar(
            curr_node, target_node, agent.agent_velocity, self.fleet_management.get_time_now(), aid, 
            dwelling_occupancy=obs
        )
        
        if path:
            if len(path) > 1 and path[1] in obs: return False

            path_nodes, path_edges = self.fleet_management.path_to_nodes_and_edges(path)
            if len(path) > 1: self.agent_next_hop[aid] = path[1]

            if task.get('task_id') != 'Dwelling':
                last = path_nodes[-1]['nodeId']
                if last == pick_node and not agent.loaded:
                    path_nodes[-1]['actions'] = [{'actionType': 'pick', 'actionId': f"p-{task['task_id']}", 'blockingType': 'HARD'}]
                elif last == drop_node and agent.loaded:
                    path_nodes[-1]['actions'] = [{'actionType': 'drop', 'actionId': f"d-{task['task_id']}", 'blockingType': 'HARD'}]

            order_id = f"{task.get('task_id')}_{int(time.time()*100)}"
            agent.order_interface.generate_order_message(
                agent=agent, orderId=order_id, order_updateId=0, nodes=path_nodes, edges=path_edges
            )
            agent.agent_state = 'EXECUTING'
            self.agents.logging.info(f"PLAN: {aid} -> {target_node}")
            return True
        return False

    def _refresh_node_cache(self):
        self.dwelling_occupancy = {}
        for agent in self.agents.agents:
            node = self._get_or_find_current_node(agent)
            if node:
                self.agent_current_node_cache[str(agent.agentId)] = node
                if agent.agent_state == 'IDLE':
                    self.dwelling_occupancy[node] = str(agent.agentId)

    def _run_auction(self):
        tasks = [t for t in self.task_management.task_list if not t.get('task_assigned')]
        # 排除在冷却期内的 AGV
        now = time.time()
        candidates = [a for a in self.agents.agents if a.agent_state == 'IDLE' and (not a.current_task or a.current_task.get('task_id') == 'Dwelling') and now >= self.post_task_cooldowns.get(str(a.agentId), 0)]
        
        if not tasks or not candidates: return
        
        # 引入简单的成本计算，优先将任务分配给最容易开始执行的 AGV
        bids = []
        
        for agent in candidates:
            aid = str(agent.agentId)
            curr_node = self.agent_current_node_cache.get(aid)
            if not curr_node: continue
            
            for task in tasks:
                s_station = self.graph.stations.get(task.get('startStationId'))
                if not s_station: continue
                pick_node = s_station.get('interactionNodeIds', [None])[0]
                if not pick_node: continue

                # 预先检查：是否能规划到起点
                # 注意：使用 space_time_astar 预检查会消耗大量计算资源，
                # 但这是在复杂死锁环境下确保可达性的必要手段。
                temp_obs = self._get_dynamic_obstacles(aid)
                temp_path = self.fleet_management.space_time_astar(
                    curr_node, pick_node, agent.agent_velocity, self.fleet_management.get_time_now(), 
                    # 临时任务，不实际预订
                    f"TEMP_PLAN_{aid}", 
                    dwelling_occupancy=temp_obs 
                )
                
                if temp_path:
                    # 成本估算：路径长度（米）/ 路径时间（秒）
                    # 这里我们使用路径中的节点数量作为简化成本
                    cost = len(temp_path)
                    
                    bids.append({
                        'agent': agent,
                        'task': task,
                        'cost': cost
                    })

        if not bids: return
        
        # 按成本升序排序 (最低成本优先)
        bids.sort(key=lambda x: x['cost'])
        
        # 分配任务：确保任务和 AGV 不被重复分配
        used_tasks = set()
        used_agents = set()
        
        for bid in bids:
            agent = bid['agent']
            task = bid['task']
            
            if task['task_id'] in used_tasks or agent.agentId in used_agents:
                continue
                
            # 分配任务
            agent.current_task = task
            agent.loaded = False
            task['task_assigned'] = True
            task['agent_id'] = str(agent.agentId)
            self.agents.logging.info(f"Auction: {task['task_id']} assigned to {agent.agentId} with cost {bid['cost']}.")
            
            used_tasks.add(task['task_id'])
            used_agents.add(agent.agentId)

    def _send_to_dwelling(self, agent, exclude=None):
        if not self.dwelling_nodes: return
        aid = str(agent.agentId)
        curr = self.agent_current_node_cache.get(aid)
        if not curr: curr = self._get_or_find_current_node(agent)
        
        # 如果 AGV 已经在 Dwelling 节点，且不是被驱逐者，则不需要移动
        if curr in self.dwelling_nodes and curr != exclude: return

        obs = self._get_dynamic_obstacles(aid)
        # 排除已被占用的节点，并排除被排除的节点
        candidates = [n for n in self.dwelling_nodes if n != exclude and n not in obs and n not in self.dwelling_occupancy]
        if not candidates: 
            # 如果所有节点都被占用或排除，则尝试在 Dwelling 节点中随机选择一个未被排除的。
            # 这是一种风险操作，但能防止 AGV 永久卡在原位。
            candidates = [n for n in self.dwelling_nodes if n != exclude]
            if not candidates: return
        
        # 【选择最近的 Dwelling 节点】
        if curr in self.graph.nodes:
            curr_pos = self.graph.nodes[curr]['position']
            best_target = None
            min_dist = float('inf')
            
            for n in candidates:
                if n not in self.graph.nodes: continue
                n_pos = self.graph.nodes[n]['position']
                dist = math.sqrt((curr_pos['x'] - n_pos['x'])**2 + (curr_pos['y'] - n_pos['y'])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_target = n
            
            target = best_target if best_target else random.choice(candidates)
        else:
            target = random.choice(candidates)
        # 【结束修改】
        
        task = {'task_id': 'Dwelling', 'dummy_target_node': target}
        
        if self._plan_mission(agent, task):
            agent.current_task = task
            self.agents.logging.info(f"{aid} going to sleep at {target}")


    # =========================================================================
    # 核心线程逻辑
    # =========================================================================

    def safety_guard_loop(self):
        time.sleep(1.0)
        self.agents.logging.info(">>> Safety Guard Active (V119: COMPLETE)")
        while True:
            try:
                self._check_physical_collisions()
                time.sleep(0.05) 
            except Exception as e:
                self.agents.logging.error(f"Safety Guard Loop Error: {e}")
                time.sleep(1)

    def assign_tasks_loop(self):
        time.sleep(2.0)
        self.agents.logging.info(">>> Task Manager Loop Started (V119: COMPLETE)")
        
        while True:
            try:
                with self.action_lock:
                    self._do_task_logic()
                
                time.sleep(0.2)
            except Exception as e:
                self.agents.logging.error(f"Manager Loop Error: {e}")
                self.agents.logging.error(traceback.format_exc())
                time.sleep(1)