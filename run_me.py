"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import os

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
from cbs import run_CBS
import numpy.random as rnd
import math

rnd.seed(1)

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 15
numb_of_aircraft = 15
planner = "CBS" #choose which planner to use (prioritized, CBS)
priority = 'first_come' #choose between 'first_come', 'shortest_path' or 'weighted'

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

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
        - edges_dict = dictionary with edges annd edge properties
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
random = True #True uses randomly generated aircraft, False generates 2 aircraft which collide at t = 5.0
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


    # if t==0:
    #     ac = Aircraft(1, 'A', 36, 37, 5, nodes_dict)
    #     print(ac.spawntime)
    #     spawntime = ac.spawntime
    #     ac.spawntime = spawntime + 1
    #     print(ac.spawntime)

    if t==1 and random == True:
        start_nodes_and_time = []

        #introduce random aircraft
        for i in range(numb_of_aircraft):
            counter = 0 #if multiple aircraft spawn at same place/time, counter goes up
            arrival_or_departure = rnd.choice(['A', 'D'])
            spawn_time = rnd.randint(1, simulation_time)

            if arrival_or_departure == 'A':
                start_node = rnd.choice(arrival_nodes)
                while [start_node, spawn_time] in start_nodes_and_time:
                    start_node = rnd.choice(arrival_nodes)
                    counter = counter + 1
                    if counter >= 2: #first check other arrival node (of total 2)
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
                    if counter >= 5: #first check all other gate nodes (of total 5)
                        spawn_time = rnd.randint(0, simulation_time)
                        counter = 0

                goal_node = departure_nodes[chosen_departure_nodes.index(min(chosen_departure_nodes))]
                # print(goal_node)
                chosen_departure_nodes[departure_nodes.index(goal_node)] += 1

            ac = Aircraft(i, arrival_or_departure, start_node, goal_node, spawn_time, nodes_dict)
            aircraft_lst.append(ac)
            start_nodes_and_time.append([start_node, spawn_time])

    # random = False
    # Spawn aircraft for this timestep (use for example a random process)
    if t == 1 and random == False:
        ac = Aircraft(17, 'A', 37,36,t+0.5, nodes_dict) #As an example we will create one aicraft arriving at node 37 with the goal of reaching node 36
        ac1 = Aircraft(0, 'D', 36,37,t, nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac)
        aircraft_lst.append(ac1)

    #Do planning 
    if planner == "Independent":
        #(Hint: Think about the condition that triggers (re)planning)
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

    elif planner == "Prioritized":
        if t == 0:
            constraints = []
            prioritize_counter = 0
        constraints, prioritize_counter, aircraft_lst = run_prioritized_planner(aircraft_lst, nodes_dict, heuristics, t, priority, constraints, prioritize_counter)

    elif planner == "CBS":
        if t == 0:
            constraints = []
        run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, inverse_nodes_dictionary)
    #elif planner == -> you may introduce other planners here
    else:
        raise Exception("Planner:", planner, "is not defined.")

    #Record the amount of time an aircraft is standing still
    if t == 1:
        from_to_lst = [0]*numb_of_aircraft

    # Move the aircraft that are taxiing
    # for ac in aircraft_lst:
    #     if ac.status == "taxiing":
    #         ac.move(dt, t)
    #         if math.modf(t)[0] == 0.5 or math.modf(t)[0] == 0: #correct for run_me and planner time difference
    #             if ac.from_to == from_to_lst[ac.id]:
    #                 ac.waiting_time += 1
    #             from_to_lst[ac.id] = ac.from_to
                #print(from_to_lst)
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            ac.move(dt, t)
                           
    t = t + dt
    #Calculate score of planner
    if t == time_end:
        score = scorecounter(aircraft_lst)
        print('Score = ', score)
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
