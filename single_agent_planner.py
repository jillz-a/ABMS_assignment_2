"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx
import numpy as np

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def build_constraint_table(constraints, agent):
    """" Builds a constraint table for a_star function based on constraints given for every agent."""

    # constraint_table = [] #[[timestep1, [(location1)]], [timestep2, [(location2)]], ...]
    # for i in range(len(constraints)): #runs for how many constraints are applied
    #     if agent == constraints[i]['agent']:
    #         constraint_table.append([constraints[i]['timestep'], constraints[i]['node']]) #ads constraints that apply to relevant agent
    #
    #     else:
    #         constraint_table.append([-1, [(-1, -1)]]) #if no constraints for agent, add dummy values
    #
    # return constraint_table

    constraint_table = []
    for i in constraints:
        if i['agent'] != agent:
            continue
        timestep = i['timestep']


        # Code regarding the edge constraints. len(i['loc']) should be larger than 1.
        if len(i['node']) != 1:
            location_lst = []       #Temporary list to store the two constraint nodes.
            for y in range(len(i['node'])):
                location = i['node'][y]
                location_lst.append(location)
        else:
            location_lst = [i['node'][0]]
        constraint_table.append([timestep, location_lst])

    constraint_table = np.array(constraint_table)

    # Sort array on timestep, which is the 0th index in the constraint table appends.
    if len(constraint_table) != 0:
        constraint_table = constraint_table[constraint_table[:, 0].argsort()]

    return constraint_table



def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    """Checks whether the a constraint is broken."""
    #
    # for i in range(len(constraint_table)):
    #     #check for edge constraints
    #     if len(constraint_table[i][1]) == 2 and next_time == constraint_table[i][0]\
    #             and (curr_loc, next_loc) == (constraint_table[i][1][0], constraint_table[i][1][1]):
    #         return True
    #     #check for vertex constraints
    #     elif next_time == constraint_table[i][0] and next_loc == constraint_table[i][1][0]:
    #         return True


    switch = False
    for i in constraint_table:
        # Code regarding vertex constraints.
        if len(i[1])==1:
            time_step = i[0]
            constraint_location = i[1][0]
            if constraint_location == next_loc and next_time == time_step:
                return True
        # Code regarding edge constraints.
        if len(i[1])!=1:
            time_step = i[0]
            if time_step != next_time:
                continue
            constraint_location1 = i[1][0]
            if curr_loc != constraint_location1:
                continue
            constraint_location2 = i[1][1]
            if next_loc == constraint_location2:
                return True
    return switch

def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent, constraints):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # print(get_path(curr))
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)


        # Addition of code to account for a waiting step. --------------------------------------------------------------
        child = {'loc': curr['loc'],
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[curr['loc']][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}

        if is_constrained(child['parent']['loc'], child['loc'], child['timestep'], constraint_table):
            continue


        if (child['loc'], child['timestep']) in closed_list:
            existing_node = closed_list[(child['loc'], child['timestep'])]
            if compare_nodes(child, existing_node):
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
        else:
            closed_list[(child['loc'], child['timestep'])] = child
            push_node(open_list, child)

        # --------------------------------------------------------------------------------------------------------------

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}

            if is_constrained(child['parent']['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            # Code in order to check to prevent the aircraft from going backwards.
            path = get_path(curr)[-5:]
            path = list(dict(path))
            if child['loc'] in path:
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path