"""
Implement prioritized planner here
"""
from single_agent_planner import calc_heuristics, simple_single_agent_astar

def run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t, priority, constraints, first_come_counter):
    """Solves paths by adding constraints based on priority. 3 versions of prioritized solving will be tested:
    - first_come: Priority given to agents who spawned earlier
    - shortest_path: Priority given to agents who have the shortest paths
    - weighted: Priority given to agents with higher weights given based on ...
    """

    if priority == 'shortest_path':


        return

###################################################################
    if priority == 'first_come':

        for ac in aircraft_lst:
            if ac.spawntime == t:
                ac.status = "taxiing"
                ac.position = nodes_dict[ac.start]["xy_pos"]
                if ac.status == "taxiing":
                    start_node = ac.start
                    goal_node = ac.goal
                    success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, ac.spawntime, ac.id, constraints)
                    if success:
                        ac.path_to_goal = path[1:]
                        next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                        ac.from_to = [path[0][0], next_node_id]
                        for j in range(len(path) - 1):
                            for i in range(len(aircraft_lst) + first_come_counter):  # Somehow a +1 fixes a colission.
                                if not i == ac.id:
                                    constraints.append({'agent': i, 'node': [path[j][0]], 'timestep': path[j][1]})
                                    constraints.append({'agent': i, 'node': [path[j + 1][0], path[j][0]], 'timestep': path[j+1][1]})

                    else:
                        # Temporary code in order to remove the node for which no path can be found. This problem occurs
                        # when the start node at the gate is already occupied. Therefore, it does not even make sense
                        # to start at this node.
                        first_come_counter = first_come_counter + 1
                        aircraft_lst.pop(aircraft_lst.index(ac))
                        continue
                        # raise Exception("No solution found for", ac.id)

        return constraints, first_come_counter



###################################################################



####################################################################
    if priority == 'weighted':
        pass