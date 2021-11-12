"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""
from os import environ
import os

environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import numpy as np
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from distributed_planning import run_distributed_planner
from cbs import run_CBS
# from CBS_dos import run_CBS
import numpy.random as rnd
import math

rnd.seed(2)

# %% SET SIMULATION PARAMETERS
# Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx"  # xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx"  # xlsx file with for each edge: from  (node), to (node), length

# Parameters that can be changed:
simulation_time = 10
numb_of_aircraft = 15
planner = "CBS"  # choose which planner to use (prioritized, CBS, Distributed)
priority = 'shortest_path'  # choose between 'first_come', 'shortest_path' or 'weighted'

# Visualization (can also be changed)
plot_graph = False  # show graph representation in NetworkX
visualization = True  # pygame visualization
visualization_speed = 0.1  # set at 0.1 as default


# %%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges and edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates
    """
    gates_xy = []  # lst with (x,y) positions of gates
    rwy_dep_xy = []  # lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = []  # lst with (x,y) positions of exit points of arrival runways

    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)

    # Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"], row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties

        # Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"], row["y_pos"]))

    # Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy,
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}

    # Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        from_node = edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties

    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)

    return nodes_dict, edges_dict, start_and_goal_locations


def create_graph(nodes_dict, edges_dict, plot_graph=True):
    """
    Creates networkX graph based on nodes and edges and plots
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """

    graph = nx.DiGraph()  # create directed graph in NetworkX

    # Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node,
                       node_id=nodes_dict[node]["id"],
                       xy_pos=nodes_dict[node]["xy_pos"],
                       node_type=nodes_dict[node]["type"])

    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1],
                       edge_id=edge,
                       from_node=edges_dict[edge]["from"],
                       to_node=edges_dict[edge]["to"],
                       weight=edges_dict[edge]["length"])

    # Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)

    return graph


def scorecounter(aircraft_lst):  # Calculate score of simulation run
    # average waiting time
    wait_time = []
    for aircraft in aircraft_lst:
        wait_time.append(aircraft.waiting_time)

    avg_wait_time = np.average(wait_time)

    score = np.round(avg_wait_time, 4)

    return score


def inverse_nodes_dict():
    """
    Function to go from the (x,y) position back to the node. The return is a dictionary containing xy_positions and the
    ID of the respective nodes.
    This function is required to get the current node of the aircraft without for looping over a dictionary. Saves some
    computational time.
    Almost all code is copy pasted from create_graph. I adapted it to meet our needs.
    """
    nodes_file = "nodes.xlsx"
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)

    inverse_nodes_dictionary = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "xy_pos": (row["x_pos"], row["y_pos"])}
        node_id = (row["x_pos"], row["y_pos"])
        inverse_nodes_dictionary[node_id] = node_properties
    return inverse_nodes_dictionary


# %% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
inverse_nodes_dictionary = inverse_nodes_dict()
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []  # List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict)  # visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================

# Start of while loop
running = True
escape_pressed = False
time_end = simulation_time + 5
dt = 0.1  # should be factor of 0.5 (0.5/dt should be integer)
t = 0
random = True  # True uses randomly generated aircraft, False generates 2 aircraft which collide at t = 5.0
print("Simulation Started")
while running:
    t = round(t, 2)

    # Check conditions for termination
    if t >= time_end or escape_pressed:
        running = False
        pg.quit()
        print("Simulation Stopped")
        break

        # Visualization: Update map if visualization is true
    if visualization:
        current_states = {}  # Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                current_states[ac.id] = {"ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed)

        # #Spawning: aircraft are spawned on random gate/arrival nodes and are assigned a random departure/gate node
    gate_nodes = [97, 34, 35, 36, 98]
    arrival_nodes = [37, 38]
    departure_nodes = [1, 2]
    chosen_gate_nodes = [0, 0, 0, 0, 0]
    chosen_departure_nodes = [0, 0]

    # if t==0:
    #     ac = Aircraft(1, 'A', 36, 37, 5, nodes_dict)
    #     print(ac.spawntime)
    #     spawntime = ac.spawntime
    #     ac.spawntime = spawntime + 1
    #     print(ac.spawntime)

    if t == 1 and random == True:
        start_nodes_and_time = []

        # introduce random aircraft
        for i in range(numb_of_aircraft):
            counter = 0  # if multiple aircraft spawn at same place/time, counter goes up
            arrival_or_departure = rnd.choice(['A', 'D'])
            spawn_time = rnd.randint(1, simulation_time)

            if arrival_or_departure == 'A':
                start_node = rnd.choice(arrival_nodes)
                while [start_node, spawn_time] in start_nodes_and_time:
                    start_node = rnd.choice(arrival_nodes)
                    counter = counter + 1
                    if counter >= 2:  # first check other arrival node (of total 2)
                        spawn_time = rnd.randint(0, simulation_time)
                        counter = 0

                goal_node = gate_nodes[chosen_gate_nodes.index(min(chosen_gate_nodes))]
                # print(goal_node)
                chosen_gate_nodes[gate_nodes.index(goal_node)] += 1

            if arrival_or_departure == 'D':
                start_node = rnd.choice(gate_nodes)
                while [start_node, spawn_time] in start_nodes_and_time:
                    start_node = rnd.choice(gate_nodes)
                    counter = counter + 1
                    if counter >= 5:  # first check all other gate nodes (of total 5)
                        spawn_time = rnd.randint(0, simulation_time)
                        counter = 0

                goal_node = departure_nodes[chosen_departure_nodes.index(min(chosen_departure_nodes))]
                # print(goal_node)
                chosen_departure_nodes[departure_nodes.index(goal_node)] += 1

            ac = Aircraft(i, arrival_or_departure, start_node, goal_node, spawn_time, nodes_dict)
            aircraft_lst.append(ac)
            start_nodes_and_time.append([start_node, spawn_time])

    random = True
    # Spawn aircraft for this timestep (use for example a random process)
    if t == 1 and random == False:
        ac = Aircraft(17, 'A', 37, 36, t,
                      nodes_dict)  # As an example we will create one aicraft arriving at node 37 with the goal of reaching node 36
        ac1 = Aircraft(0, 'D', 36, 37, t,
                       nodes_dict)  # As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac)
        aircraft_lst.append(ac1)

    # Do planning
    if planner == "Independent":
        # (Hint: Think about the condition that triggers (re)planning)
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

    elif planner == "Prioritized":
        if t == 0:
            constraints = []
            prioritize_counter = 0
        constraints, prioritize_counter, aircraft_lst = run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t,
                                                                                priority, constraints,
                                                                                prioritize_counter)

    elif planner == "CBS":
        if t == 0:
            constraints = []
        if t % 0.5 == 0:
            run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, inverse_nodes_dictionary)

    elif planner == "Distributed":
        if t == 0:
            constraints = {}
        if t % 0.5 == 0:
            run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints,
                                    inverse_nodes_dictionary)

    else:
        raise Exception("Planner:", planner, "is not defined.")

    # Record the amount of time an aircraft is standing still
    if t == 1:
        from_to_lst = [0] * numb_of_aircraft

    # Move the aircraft that are taxiing
    # for ac in aircraft_lst:
    #     if ac.status == "taxiing":
    #         ac.move(dt, t)
    #         if math.modf(t)[0] == 0.5 or math.modf(t)[0] == 0: #correct for run_me and planner time difference
    #             if ac.from_to == from_to_lst[ac.id]:
    #                 ac.waiting_time += 1
    #             from_to_lst[ac.id] = ac.from_to

    for ac in aircraft_lst:
        if ac.status == "taxiing":
            if ac.id==6:
                print(ac.path_to_goal)
                print(ac.from_to)
            ac.move(dt, t)

    t = t + dt
    # Calculate score of planner
    if t == time_end:
        score = scorecounter(aircraft_lst)
        print('Score = ', score)

# """
# Implement CBS here
# """
#
# import heapq
# import numpy as np
# import numpy.random as rnd
# from single_agent_planner import simple_single_agent_astar # , pop_node, push_node
# import pandas as pd
# import os
#
#
#
# def get_heading(heading, xy_start, xy_next):
#     """
#     Function blatantly stolen from the Aircraft class. In case of replanning events, no heading change may be larger
#     than 90 degrees.
#     """
#     if xy_start[0] == xy_next[0]:  # moving up or down
#         if xy_start[1] > xy_next[1]:  # moving down
#             heading = 180
#         elif xy_start[1] < xy_next[1]:  # moving up
#             heading = 0
#         else:
#             heading = heading
#
#     elif xy_start[1] == xy_next[1]:  # moving right or left
#         if xy_start[0] > xy_next[0]:  # moving left
#             heading = 90
#         elif xy_start[0] < xy_next[0]:  # moving right
#             heading = 270
#         else:
#             heading = heading
#     else:
#         raise Exception("Invalid movement")
#     return heading
#
# def get_sum_of_cost(paths):
#     rst = 0
#     for path in paths:
#         rst += len(path) - 1
#     return rst
#
# def detect_collision(path1, path2):
#     first_collision = []
#     for i in range(len(path1)-1):
#         for j in range(len(path2)-1):
#             if path1[i] == path2[j]:# for vertex collisions
#                 first_collision = [path1[i][0], path1[i][1]]
#                 return first_collision
#             elif path1[i][0] == path2[j + 1][0] and path2[j][0] == path1[i + 1][0] \
#                     and path1[i][1] == path2[j][1] and path1[i+1][1] == path2[j+1][1]:  # for edge collisions
#                 first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]
#                 return first_collision
#     if path1[i+1] == path2[j+1]:
#         first_collision = [path1[i+1][0], path1[i+1][1]]
#         return first_collision
#
#     return first_collision
#
# def detect_collisions(paths, id):
#     collision_list = []
#     if len(paths) >= 2:
#         for agent0 in range(len(paths)):
#             for agent1 in range(agent0, len(paths)):
#                 # if id[agent0] == 1 and id[agent1] == 6:
#                 #     print(id[agent0], paths[agent0])
#                 #     print(id[agent1], paths[agent1])
#                 # if id[agent0] == 6 and id[agent1] == 1:
#                 #     print(id[agent0], paths[agent0])
#                 #     print(id[agent1], paths[agent1])
#                 if agent0 == agent1:
#                     continue
#                 else:
#                     first_collision = detect_collision(paths[agent0], paths[agent1])
#                     if len(first_collision) == 0:
#                         continue
#                     else:
#                         # print('first collision = ', first_collision, type(first_collision[0][0]))
#                         if type(first_collision[0]) == tuple:
#                             collision_list.append(
#                                 {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0][0], first_collision[0][1]],
#                                  'timestep': first_collision[1]})
#                             return collision_list
#                         else:
#                             collision_list.append(
#                                 {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0]], 'timestep': first_collision[1]})
#                             return collision_list
#
#         collision_list.append(None)
#         return collision_list
#
# print(detect_collisions([[(20, 7.0), (70.0, 7.5), (19.0, 8.0), (63.0, 8.5), (12.0, 9.0), (56.0, 9.5), (11.0, 10.0), (55.0, 10.5), (10.0, 11.0), (52.0, 11.5), (39.0, 12.0), (48.0, 12.5), (3.0, 13.0), (42.0, 13.5), (40.0, 14.0), (43.0, 14.5), (4.0, 15.0), (95.0, 15.5), (1.0, 16.0)],
#                          [(64, 7.0), (64, 7.5), (64, 8.0), (20.0, 8.5), (71.0, 9.0), (21.0, 9.5), (77.0, 10.0), (27.0, 10.5), (86.0, 11.0), (32.0, 11.5), (94.0, 12.0), (36.0, 12.5)]],
#                         [1,6]))
#
#
# def standard_splitting(collision):
#     if collision == []:
#         return []
#     collision = collision[0]
#     if collision == None:
#         return []
#     if collision['node'] == [64]:
#         print(collision)
#     if len(collision['node']) == 2:
#         collision_split = [{'agent': collision['a1'], 'node': [collision['node'][0], collision['node'][1]], 'timestep': collision['timestep']+0.5},
#                            {'agent': collision['a2'], 'node': [collision['node'][1], collision['node'][0]], 'timestep': collision['timestep']-0.5}]
#     else:
#         collision_split = [{'agent': collision['a1'], 'node': collision['node'], 'timestep': collision['timestep']},
#                            {'agent': collision['a2'], 'node': collision['node'], 'timestep': collision['timestep']}]
#
#     return collision_split
#
# def get_location(path, time):
#     if time < 0:
#         return path[0]
#     elif time < len(path):
#         return path[time]
#     else:
#         return path[-1]
#
# def push_node(open_list, numb_of_generated, node):
#     if node['collisions'] == None:
#         node['collisions'] = []
#     # print('Nodes generated: ', numb_of_generated)
#     heapq.heappush(open_list, (node['cost'], len(node['collisions']), numb_of_generated , node))
#
# def pop_node(open_list):
#     _, _, _, node = heapq.heappop(open_list)
#     return node
#
#
#
# #Run Conflict Based Search program
#
# def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, dict_inverse_nodes):
#     statement = True
#     gate_nodes = [97, 34, 35, 36, 98]
#     gate_block_nodes = [97, 34, 35, 36, 98]
#     gate_intersection_dict = {97: {'nodes': [83, 88, 29, 99]},
#                               34: {'nodes': [88, 84, 89, 30, 92]},
#                               35: {'nodes': [93, 31, 85, 89, 90]},
#                               36: {'nodes': [94, 32, 86, 90, 91]},
#                               98: {'nodes': [100, 33, 91, 87]}}
#     numb_of_generated = 0
#     open_list = []
#
#     # Sometimes an aircraft would spawn directly in front of another aircraft at a gate node. This results in the fact
#     # that two aicraft cannot plan their path, since aircraft are prohibited from moving backwards. The code loops
#     # through the active aircraft, and identifies their heading and current position. Aircraft that are currently at a
#     # node from which they cannot go backwards (node in gate_block_nodes) have right of way. In the next for loop,
#     # aircraft that want to spawn at the goal node of the other aircraft receive an update of their spawntime or spawn
#     # location.
#     blocked_list = []
#     for ac in aircraft_lst:
#         if ac.status == "taxiing":
#             dict = {"id": ac.id, "Heading": ac.heading, "Position": ac.path_to_goal[0][0], "Goal_node": ac.goal}
#             if dict["Heading"] == 270 and dict["Position"] in gate_block_nodes:
#                 blocked_list.append(ac.goal)
#             if ac.type == "A":
#                 if ac.path_to_goal[0][0] in gate_intersection_dict[ac.goal]['nodes']:
#                     blocked_list.append(ac.goal)
#     boolean = False
#     for ac in aircraft_lst:
#         if ac.spawntime == t:
#             # print('Aircraft ', ac.id, ' spawned at t = ', t)
#             if ac.start in blocked_list:
#                 random_string = rnd.choice(["spawntime", "start_location"])
#                 if random_string == "spawntime":
#                     ac.spawntime = ac.spawntime + 0.5
#                 if random_string == "start_location":
#                     counter = 0
#                     while ac.start in blocked_list:
#                         ac.start = rnd.choice(gate_nodes)
#                         counter = counter + 1
#                         if counter > 4:
#                             ac.spawntime = ac.spawntime + 0.5
#                             break
#                 continue
#
#             # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
#             # replanning.
#             ac.status = "taxiing"
#             ac.position = nodes_dict[ac.start]["xy_pos"]
#             boolean = True
#     if t==9:
#         for ac in aircraft_lst:
#             if ac.spawntime == 9 or ac.id==2:
#                 print(ac.id, ac.spawntime, ac.start)
#
#     if boolean:
#         counter = 0
#         root = {'cost': 0,
#                 'constraints': [],
#                 'paths': [],
#                 'id': [],
#                 'collisions': []}
#
#         # for ac in aircraft_lst:
#         #     if ac.status == "taxiing":
#         #         # We do not need start_node since we replan aircraft at their current position.
#         #         goal_node = ac.goal
#         #         current_node = dict_inverse_nodes[ac.position]["id"]
#         #         success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
#         #                                                   t, ac.id, constraints)
#         #
#         #         if success:
#         #             if t == ac.spawntime:
#         #                 ac.path = path
#         #             root['paths'].append(path)
#         #             root['id'].append(ac.id)
#         #         else:
#         #             raise Exception("No solution found for", ac.id)
#
#         for ac in aircraft_lst:
#             if ac.status == "taxiing" and t == ac.spawntime:
#                 # We do not need start_node since we replan aircraft at their current position.
#                 goal_node = ac.goal
#                 current_node = dict_inverse_nodes[ac.position]["id"]
#                 success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
#                                                           t, ac.id, [])
#                 if success:
#                     if t == ac.spawntime:
#                         ac.path = path
#                     root['paths'].append(path)
#                     root['id'].append(ac.id)
#                     ac.attribute = False
#                 else:
#                     raise Exception("No solution found for", ac.id)
#             if ac.status == 'taxiing' and t != ac.spawntime:
#                 if ac.id == 6:
#                     print(dict_inverse_nodes[ac.position]['id'])
#                     # print(t, ac.path_to_goal)
#                 path = ac.path_to_goal
#                 # print(path)
#                 root['paths'].append(path)
#                 root['id'].append(ac.id)
#
#         root['collisions'] = detect_collisions(root['paths'], root['id'])
#         root['cost'] = get_sum_of_cost(root['paths'])
#         push_node(open_list, numb_of_generated, root)
#         numb_of_generated += 1
#         temp_lst = []
#         while len(open_list) > 0:
#
#             P = pop_node(open_list)
#             temp_lst.append(P)
#             temp_lst = temp_lst[-3:]
#             # print(P['collisions'])
#             if len(P['paths']) >= 2:
#                 P['collisions'] = detect_collisions(P['paths'], P['id']) #Check for any collissions in current node
#
#             if len(P['collisions']) == 0 or P['collisions'][0] == None:
#                 for ac in aircraft_lst:
#                     if ac.id in P['id']:
#                         path = P['paths'][P['id'].index(ac.id)]
#                         ac.path = path
#                         # ac.path_to_goal = path[1:]
#                         ac.path_to_goal = path[1:]
#                         if ac.id == 6:
#                             print(ac.path_to_goal)
#                         next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
#                         ac.from_to = [path[0][0], next_node_id]
#                 return P['paths']
#
#
#             collision = P['collisions']
#             # print(collision)
#             constraints = standard_splitting(collision)
#
#             for constraint in constraints:  # Line 12.
#                 # if constraint == {'agent': 6, 'node': [64.0], 'timestep': 7.5}:
#                 #     continue
#                 Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': [], 'id': []}  # Line 13, new node Q.
#                 Q['path_changes'] = []
#                 Q['constraints'] = list(P['constraints'])
#                 Q['id'] = P['id']
#                 Q['constraints'].append(constraint)
#                 Q['paths'] = P['paths']
#                 a_i = constraint['agent']  # Line 16, obtaining the agent in the constraint.
#                 for ac in aircraft_lst:
#                     if ac.id == a_i:
#                         current_node = dict_inverse_nodes[ac.position]["id"]
#                         goal_node = ac.goal
#                         id = ac.id
#                         heading = ac.heading
#                         old_path = ac.path
#                         path_lst = []
#                         for i in old_path:
#                             if i[0] not in path_lst:
#                                 path_lst.append(i[0])
#                         # if ac.id==6:
#                         #     print(current_node, path_lst)
#                         break
#                 index = path_lst.index(current_node)-1
#                 # if ac.id == 1:
#                 #     print(ac.path_to_goal)
#                 # -------------------------------------------------------------------
#                 # Als je deze twee lines hieronder comment, dan werkt ie wel maar gaan sommige achteruit.
#                 # Deze twee lines voegen een constraint toe, maar zorgen voor een infinite loop..?
#                 if index > 0 and len(path_lst) > 1:
#                     if {'agent': ac.id, 'node': [path_lst[index]], 'timestep': t+0.5} not in Q['constraints']:
#                         Q['constraints'].append({'agent': ac.id, 'node': [path_lst[index]], 'timestep': t+0.5})
#
#                 success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
#                                                           t, id, Q['constraints'])
#                 if success == True:
#                     # if statement == False and ac.id == 6:
#                     #     print(path)
#                     Q['paths'][Q['id'].index(a_i)] = list(path)
#                     Q['collisions'] = detect_collisions(Q['paths'], Q['id'])
#                     Q['path_changes'].append(ac.id)
#                     # print()
#                     # print('Q; ', Q['collisions'])
#                     # print()
#                     Q['cost'] = get_sum_of_cost(Q['paths'])
#                     push_node(open_list, numb_of_generated, Q)
#                     numb_of_generated += 1
#                 else:
#                     statement = False
#                     # raise Exception('Error, no path found')
#     return constraints, aircraft_lst
#
