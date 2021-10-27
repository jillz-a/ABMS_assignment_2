
""""Distributed planning of aircraft agents"""

import heapq
from single_agent_planner import simple_single_agent_astar


#________________________________
#Definitions


#________________________________
#Main solver

def run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, inverse_nodes_dictionary):
    """
    Distributed planner will first plan each aircraft independently.
    A 'radar' function will be implemented to keep track of aircraft locations.
    Aircraft wil have limited visibility to surrounding aircraft.
    When aircraft are in range of each other, they will plan their paths accordingly.
    """
    root = {'cost': 0,
            'constraints': [],
            'paths': [],
            'id': [],
            'collisions': []}

    """Independent planner using A* without constraints to generate initial paths"""
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing"
            start_node = ac.start
            ac.position = nodes_dict[ac.start]["xy_pos"]
            goal_node = ac.goal
            succes, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, ac.id, constraints= [])

            if succes:
                print(ac.id, path)
                root['paths'].append(path)
                root['id'].append(ac.id)
            else:
                raise Exception("No solution found for", ac.id)

    #vizualize
    for ac in aircraft_lst:
        if ac.id in root['id']:
            path = root['paths'][root['id'].index(ac.id)]
            ac.path_to_goal = path[1:]
            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
            ac.from_to = [path[0][0], next_node_id]


    pass