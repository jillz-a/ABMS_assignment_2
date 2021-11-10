"""
Implement prioritized planner here
"""
from single_agent_planner import calc_heuristics, simple_single_agent_astar
import numpy as np
import numpy.random as rnd


def run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t, priority, constraints, prioritize_counter, N_max_cap):
    """Solves paths by adding constraints based on priority. 3 versions of prioritized solving will be tested:
    - first_come: Priority given to agents who spawned earlier
    - shortest_path: Priority given to agents who have the shortest paths
    - weighted: Priority given to agents with higher weights given based on ...
    """

    if priority == 'shortest_path':
        if t==1:

            # This part of the code concerns with the individual planning of all aircraft. Only active when t=0, creation
            # time of the aircraft.
            lst_to_be_sorted = []
            for ac in aircraft_lst:
                start_node = ac.start
                goal_node = ac.goal
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, ac.id, [])
                if success:
                    # We want the location in the aircraft_lst and the length of the independent path.
                    lst_to_be_sorted.append([aircraft_lst.index(ac), len(path)])
                else:
                    raise Exception("No solution found for", ac.id)

    if priority == 'first_come':

        if t==1:
            # This part of the code concerns with the spawn times of all aircraft. Only active when t=1, creation
            # time of the aircraft.
            lst_to_be_sorted = []
            for ac in aircraft_lst:
                start_node = ac.start
                goal_node = ac.goal
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, ac.id, [])
                if success:
                    # We want the location in the aircraft_lst and the length of the independent path.
                    lst_to_be_sorted.append([aircraft_lst.index(ac), ac.spawntime])
                else:
                    raise Exception("No solution found for", ac.id)

    if priority == 'weighted':

        if t==1:
            # This part of the code concerns with the individual planning of all aircraft. Only active when t=0, creation
            # time of the aircraft.
            lst_to_be_sorted = []
            for ac in aircraft_lst:
                start_node = ac.start
                goal_node = ac.goal
                weight = rnd.randint(1,500)
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, ac.id, [])
                if success:
                    # We want the location in the aircraft_lst and the length of the independent path.
                    lst_to_be_sorted.append([aircraft_lst.index(ac), -weight])
                else:
                    raise Exception("No solution found for", ac.id)



    if t==1:
        # Creating an array so we can utilize .argsort()
        lst_to_be_sorted = np.array(lst_to_be_sorted)

        # Sort array based on second entry of every i in the list.
        lst_sorted = lst_to_be_sorted[lst_to_be_sorted[:, 1].argsort()]

        aircraft_lst_new = []
        for element in lst_sorted:
            index = int(element[0])
            aircraft_lst_new.append(aircraft_lst[index])

        aircraft_lst = aircraft_lst_new

        while_counter = 0
        while len(aircraft_lst) != while_counter:
            ac = aircraft_lst[while_counter]
            start_node = ac.start
            goal_node = ac.goal
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, ac.spawntime, ac.id, constraints)

            if success:
                # Part of the code that concerns with the waiting of aircraft at either the gate or runway nodes.
                while path[0][0] == path[1][0]:
                    # print(ac.id, path[0][0], path[1][0])
                    # print(path)
                    ac.spawntime = ac.spawntime + 0.5
                    success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics,
                                                              ac.spawntime, ac.id, constraints)
                ac.path = path[1:]
                ac.path_to_goal = path[1:]
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]
                for j in range(len(path) - 1):
                    for i in range(len(aircraft_lst) + prioritize_counter):
                        if not i == ac.id:
                            constraints.append({'agent': i, 'node': [path[j][0]], 'timestep': path[j][1]})
                            constraints.append({'agent': i, 'node': [path[j + 1][0], path[j][0]], 'timestep': path[j+1][1]})
                while_counter = while_counter + 1
            else:
                # Temporary code in order to remove the node for which no path can be found. This problem occurs
                # when the start node at the gate is already occupied. Therefore, it does not even make sense
                # to start at this node.
                prioritize_counter = prioritize_counter + 1
                aircraft_lst.pop(aircraft_lst.index(ac))
                continue

    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
    max_cap = 0
    for ac in aircraft_lst:
        if ac.status == 'taxiing':
            max_cap = max_cap + 1
    if max_cap > N_max_cap:
        N_max_cap = max_cap

    return constraints, prioritize_counter, aircraft_lst, N_max_cap
