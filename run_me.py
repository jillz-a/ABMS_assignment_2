"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""
from os import environ
import os

import seaborn as sns

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
import numpy.random as rnd


#Seed for random generator
rnd.seed(6)

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 25
numb_of_aircraft = 15
random = True #True uses randomly generated aircraft, false uses manually generated aircraft, used for verification.

planner = "Distributed" #choose which planner to use (Prioritized, CBS, Distributed)
priority = 'shortest_path' #choose between 'first_come', 'shortest_path' or 'weighted' (only for Prioritized)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

#Used for result analysis
heatmap = False
ac_pos_matrix = np.zeros((13, 15))

#%%Function definitions
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
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
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
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges and edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

def scorecounter(aircraft_lst): #Calculate score of simulation run
    #average waiting time
    wait_time = []
    for aircraft in aircraft_lst:
        wait_time.append(aircraft.waiting_time)

    avg_wait_time = np.average(wait_time)
    max_wait_time = max(wait_time)
    total_wait_time = sum(wait_time)

    score = {"total": np.round(total_wait_time, 4), "average": np.round(avg_wait_time, 4), "max": np.round(max_wait_time, 4)}

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

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
inverse_nodes_dictionary = inverse_nodes_dict()
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time + 5
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0

print("Simulation Started")
while running:
    t= round(t,2)    
       
    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
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

#________________________(Random) Aircraft Generation________________________________
    if t==1 and random == True:
        start_nodes_and_time = []

        #introduce random aircraft
        for i in range(numb_of_aircraft):
            counter = 0 #if multiple aircraft spawn at same place/time, counter goes up
            arrival_or_departure = rnd.choice(['A', 'D'])

            spawn_time = rnd.randint(1, simulation_time)

            if arrival_or_departure == 'A':
                start_node = rnd.choice(arrival_nodes)
                while [start_node, spawn_time] in start_nodes_and_time: #checks if aircraft are to spawn on top of each other
                    start_node = rnd.choice(arrival_nodes) #first check if other arrival node is free (of total 2)
                    counter = counter + 1
                    if counter >= 2:
                        spawn_time = rnd.randint(0, simulation_time) #if no other node is free, change spawn time
                        counter = 0

                goal_node = gate_nodes[chosen_gate_nodes.index(min(chosen_gate_nodes))]
                chosen_gate_nodes[gate_nodes.index(goal_node)] += 1

            if arrival_or_departure == 'D':
                start_node = rnd.choice(gate_nodes)
                while [start_node, spawn_time] in start_nodes_and_time: #checks if aircraft are to spawn on top of each other
                    start_node = rnd.choice(gate_nodes) #first check all other gate nodes (of total 5)
                    counter = counter + 1
                    if counter >= 5:
                        spawn_time = rnd.randint(0, simulation_time) #if no other node is free, change spawn time
                        counter = 0

                goal_node = departure_nodes[chosen_departure_nodes.index(min(chosen_departure_nodes))]
                chosen_departure_nodes[departure_nodes.index(goal_node)] += 1

            ac = Aircraft(i, arrival_or_departure, start_node, goal_node, spawn_time, nodes_dict)
            aircraft_lst.append(ac)
            start_nodes_and_time.append([start_node, spawn_time])

#_________________________(Manual) Aircraft Generation for Verification________________

    # Spawn aircraft for this timestep (used for verification purposes).
    if t == 1 and random == False:
        ac = Aircraft(1, 'D', 36,37, 1.0, nodes_dict)
        ac1 = Aircraft(0, 'A', 37,36, 1.5, nodes_dict)
        aircraft_lst.append(ac)
        aircraft_lst.append(ac1)

#_________________________Planner initialisation_________________________________________
    #Do planning 
    if planner == "Independent":
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

    elif planner == "Prioritized":
        if t == 0:
            constraints = []
            prioritize_counter = 0
            N_max_cap = 0
        constraints, prioritize_counter, aircraft_lst, N_max_cap = run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t, priority, constraints, prioritize_counter, N_max_cap)

    elif planner == "CBS":
        if t == 0:
            constraints = []
        if t%0.5==0:
            run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, inverse_nodes_dictionary)

    elif planner == "Distributed":
        if t == 0:
            constraints = {}
        if t%0.5==0:
            run_distributed_planner(aircraft_lst, nodes_dict, heuristics, t, constraints, inverse_nodes_dictionary)

    else:
        raise Exception("Planner:", planner, "is not defined.")


    for ac in aircraft_lst:
        if ac.status == "taxiing":
            ac.move(dt, t)

            if ac.from_to[0] == ac.from_to[1]:
                ac.waiting_time += dt


    t = t + dt

          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?

    #Calculate score of planner
    if t == time_end:
        score = scorecounter(aircraft_lst)
        print('Score for solver: ', planner)
        print('Average wait time:', score["average"]," seconds")
        print('Total wait time:', score["total"], " seconds")
        print('Maximum wait time:', score["max"], " seconds")

#path heatmap
    if heatmap == True:
        mat_axes = np.arange(1, 8.5, 0.5).tolist()

        if t % 0.5 == 0:
            for ac in aircraft_lst:
                if ac.status == 'taxiing':
                    x_pos = ac.position[0]
                    y_pos = 8 - ac.position[1]
                    mat_x_pos = mat_axes.index(x_pos)
                    mat_y_pos = mat_axes.index(y_pos)

                    ac_pos_matrix[mat_y_pos][mat_x_pos] += 1

            label = np.array([['', '', '3', '48', '39', '52', '10', '61', '17', '103', '104', '', '', '', ''],
                              ['', '', '42', '', '', '', '55', '', '68', '', '105', '', '', '', ''],
                              ['', '', '40', '', '37', '101', '11', '62', '18', '74', '24', '83', '29', '99', '97'],
                              ['', '', '43', '', '', '', '56', '', '69', '', '79', '', '88', '', ''],
                              ['1', '95', '4', '', '38', '102', '12', '63', '19', '75', '25', '84', '30', '92', '34'],
                              ['', '', '44', '', '', '', '57', '', '70', '', '80', '', '89', '', ''],
                              ['2', '96', '5', '', '', '', '13', '64', '20', '76', '26', '85', '31', '93', '35'],
                              ['', '', '45', '', '', '', '58', '', '71', '', '81', '', '90', '', ''],
                              ['', '', '41', '', '', '', '14', '65', '21', '77', '27', '86', '32', '94', '36'],
                              ['', '', '46', '', '', '', '59', '', '72', '', '82', '', '91', '', ''],
                              ['', '', '6', '49', '8', '53', '15', '66', '22', '78', '28', '87', '33', '100', '98'],
                              ['', '', '47', '', '51', '', '60', '', '73', '', '106', '', '', '', ''],
                              ['', '', '7', '50', '9', '54', '16', '67', '23', '108', '107', '', '', '', '']])


            if t == time_end:
                sns.heatmap(ac_pos_matrix, annot=label, fmt="", cmap="Greys", vmin=0, xticklabels=False, yticklabels=False)
                plt.show()