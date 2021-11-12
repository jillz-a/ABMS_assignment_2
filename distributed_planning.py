
""""Distributed planning of aircraft agents"""

import heapq
from single_agent_planner import simple_single_agent_astar
import pandas as pd
import numpy as np
import os
from copy import deepcopy
#________________________________
#Definitions and constants
depth_of_path = 3       # Defines to which index other a/c get information about the a/c's path.

def box_vision(heading, position, inverse_nodes_dictionary): #defines the nodes that are visible for each individual a/c
    visible_nodes = []
    dE = 1E-12  # Really small value in order to include the ends in the numpy arranges.
    x_vision = 1.5    # Vision in the x-direction.
    y_vision = 1.5    # Vision in the y-direction.

    if heading == 0: # Moving up!
        x_left = position[0] - x_vision
        x_right = position[0] + x_vision
        y_top = position[1] + y_vision
        y_bot = position[1]

    elif heading == 180: # Moving down!
        x_left = position[0] - x_vision
        x_right = position[0] + x_vision
        y_top = position[1]
        y_bot = position[1] - y_vision

    elif heading == 90: # Moving left!
        x_left = position[0] - x_vision
        x_right = position[0]
        y_top = position[1] + y_vision
        y_bot = position[1] - y_vision

    elif heading == 270: # Moving right!
        x_left = position[0]
        x_right = position[0] + x_vision
        y_top = position[1] + y_vision
        y_bot = position[1] - y_vision

    else:
        raise Exception("Invalid heading")

    x_cords = np.arange(x_left, x_right + dE, 0.5)
    y_cords = np.arange(y_bot, y_top + dE, 0.5)
    for x in x_cords:
        for y in y_cords:
            if (x, y) == position:
                continue
            if (x,y) in inverse_nodes_dictionary:
                visible_nodes.append(inverse_nodes_dictionary[(x,y)]['id'])
    return visible_nodes

def detect_collision(path1, path2):
    first_collision = []
    if len(path1) > 1 and len(path2) > 1:
        for i in range(len(path1)-1):
            for j in range(len(path2)-1):
                if path1[i] == path2[j]:# for vertex collisions
                    first_collision = [path1[i][0], path1[i][1]]
                    return first_collision
                elif path1[i + 1] == path2[j + 1]:
                    first_collision = [path1[i + 1][0], path1[i + 1][1]]
                    return first_collision

                elif path1[i][0] == path2[j + 1][0] and path2[j][0] == path1[i + 1][0] \
                        and path1[i][1] == path2[j][1] and path1[i+1][1] == path2[j+1][1]:  # for edge collisions
                    first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]
                    return first_collision

        return first_collision

    return first_collision

def detect_collisions(paths, id):
    collision_list = []
    if len(paths) >= 2:
        for agent0 in range(len(paths)):
            for agent1 in range(len(paths)):
                if agent0 == agent1:
                    continue
                else:
                    first_collision = detect_collision(paths[agent0], paths[agent1])
                    if len(first_collision) == 0:
                        continue
                    else:
                        # print('first collision = ', first_collision, type(first_collision[0][0]))
                        if type(first_collision[0]) == tuple:
                            collision_list.append(
                                {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0][0], first_collision[0][1]],
                                 'timestep': first_collision[1]+0.5})
                            return collision_list
                        else:
                            collision_list.append(
                                {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0]], 'timestep': first_collision[1]})
                            return collision_list

        # collision_list.append(None)
        return collision_list
#________________________________
#Main solver

def run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, inverse_nodes_dictionary):
    """
    Distributed planner will first plan each aircraft independently.
    A 'radar' function will be implemented to keep track of aircraft locations.
    Aircraft wil have limited visibility to surrounding aircraft.
    When aircraft are in range of each other, they will plan their paths accordingly.
    """
    radar_dict = {}
    vision = {}
    # constraints = {}

    """Make sure an aircraft cant spawn when other aircraft is close to the spawn gate."""
    gate_nodes = [97, 34, 35, 36, 98]
    gate_block_nodes = [97, 34, 35, 36, 98]
    gate_intersection_dict = {97: {'nodes': [83, 88, 29, 99]},
                              34: {'nodes': [88, 84, 89, 30, 92]},
                              35: {'nodes': [93, 31, 85, 89, 90]},
                              36: {'nodes': [94, 32, 86, 90, 91]},
                              98: {'nodes': [100, 33, 91, 87]}}


    # Sometimes an aircraft would spawn directly in front of another aircraft at a gate node. This results in the fact
    # that two aicraft cannot plan their path, since aircraft are prohibited from moving backwards. The code loops
    # through the active aircraft, and identifies their heading and current position. Aircraft that are currently at a
    # node from which they cannot go backwards (node in gate_block_nodes) have right of way. In the next for loop,
    # aircraft that want to spawn at the goal node of the other aircraft receive an update of their spawntime or spawn
    # location.
    blocked_list = []
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            dict = {"id": ac.id, "Heading": ac.heading, "Position": ac.path_to_goal[0][0], "Goal_node": ac.goal}
            if dict["Heading"] == 270 and dict["Position"] in gate_block_nodes:
                blocked_list.append(ac.goal)
            if ac.type == "A":
                if ac.path_to_goal[0][0] in gate_intersection_dict[ac.goal]['nodes']:
                    blocked_list.append(ac.goal)

    for ac in aircraft_lst:
        if ac.spawntime == t:

            if ac.start in blocked_list:
                random_string = np.random.choice(["spawntime", "start_location"])
                if random_string == "spawntime":
                    ac.spawntime = ac.spawntime + 0.5
                if random_string == "start_location":
                    counter = 0
                    while ac.start in blocked_list:
                        ac.start = np.random.choice(gate_nodes)
                        counter = counter + 1
                        if counter > 4:
                            ac.spawntime = ac.spawntime + 0.5
                            break
                continue
            print('Aircraft ', ac.id, ' spawned at t = ', t)
            # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
            # replanning.
            # ac.status = "taxiing"
            # ac.position = nodes_dict[ac.start]["xy_pos"]

    """Independent planner using A* without constraints to generate initial paths"""
    for ac in aircraft_lst:
        if ac.spawntime == t:
            # print('Aircraft ', ac.id, ' spawned at t = ', t)
            constraints[ac.id] = {'constraints': []}
            ac.status = "taxiing"
            start_node = ac.start
            ac.position = nodes_dict[ac.start]["xy_pos"]
            goal_node = ac.goal
            succes, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t,
                                                     ac.id, constraints= [])

            if succes:
                ac.path_to_goal = path[1:]
                ac.path = path
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]
            else:
                raise Exception("No solution found for", ac.id)

    """Collision detection and constraint generating for each aircraft"""
    for ac in aircraft_lst:
        ac.added_constraint = False
        if ac.status == "taxiing":
            ac_node = inverse_nodes_dictionary[ac.position]['id']
            path = ac.path_to_goal[:depth_of_path] #portion of path of a/c that will be communicated

            # Following line adds the location and path of every agent into a dictionary, basically a radar. Not all
            # information about the paths is given, only the next X(depth_of_path) amount of nodes.
            radar_dict[ac_node] = {'path': path, 'ac_id': ac.id, 'type': ac.type, 'heading': ac.heading}

            # Following line adds the visible nodes of the A/C into a dictionary.
            vision[ac.id] = {'visible_nodes': box_vision(ac.heading, ac.position, inverse_nodes_dictionary)}

    for ac in aircraft_lst:
        if ac.status =="taxiing":
            visible_nodes = vision[ac.id]['visible_nodes']

            for node in visible_nodes: #check every node visible for aircraft
                if node in radar_dict: #if node in radar dict, then there is an aircraft in the field of view of the aircraft
                    paths = [ac.path_to_goal, radar_dict[node]['path']]
                    id_lst = [ac.id, radar_dict[node]['ac_id']]

                    attempt_counter = 0 #counts the amount of attempts are needed to generate a path without collisions.
                    priority = {'right-left': [-90, -270], 'A-D': ['A', 'D'], 'large-small': [min(id_lst), max(id_lst)]}
                    boolean = False
                    while detect_collisions(paths, id_lst) is not None and len(detect_collisions(paths, id_lst)) > 0:

                        attempt_counter += 1
                        if attempt_counter >= 4: #if more than 4 attempts are needed to prevent collision, go to the other agent.
                            # priority['right-left'].reverse()
                            # priority['A-D'].reverse()
                            # priority['large-small'].reverse()
                            constraints[ac.id]['constraints'] = []
                            constraints[radar_dict[node]['ac_id']]['constraints'].append({'agent': radar_dict[node]['ac_id'], 'node': collision['node'],
                                                                      'timestep': collision['timestep']})
                            constraints[radar_dict[node]['ac_id']]['constraints'].append({'agent': radar_dict[node]['ac_id'], 'node': collision['node'][::-1],
                                                                      'timestep': collision['timestep']})
                            boolean = True


                        collision = detect_collisions(paths, id_lst)[0]

                        #If an aircraft comes from the right, it has priority.
                        if ac.heading - radar_dict[node]['heading'] == priority['right-left'][0] or\
                                ac.heading - radar_dict[node]['heading'] == priority['right-left'][1] and boolean == False:

                            constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': collision['node'],
                                                                      'timestep': collision['timestep']})
                            constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': collision['node'][::-1],
                                                                      'timestep': collision['timestep']})


                            ac.added_constraint = True
                            print('Aircraft', ac.id, 'changed path: A/C came from right.')


                        #If aircraft are to collide head on, Departing aircraft has priority.
                        elif ac.type == priority['A-D'][0] and radar_dict[node]['type'] == priority['A-D'][1] and abs(ac.heading - radar_dict[node]['heading']) == 180 \
                                and (ac.position[0] == nodes_dict[node]['x_pos'] or ac.position[1] == nodes_dict[node]['y_pos']) and boolean == False:

                            constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': collision['node'],
                                                                      'timestep': collision['timestep']})
                            constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': collision['node'][::-1],
                                                                      'timestep': collision['timestep']})




                            ac.added_constraint = True
                            print('Aircraft', ac.id, 'changed path: Departing A/C has priority.')


                        #If none of the above and of same type, then constrain a/c with smallest id.
                        # elif ac.type == radar_dict[node]['type']:
                        elif ac.type == radar_dict[node]['type'] and boolean == False:
                            ac_change = priority['large-small'][0]

                            constraints[ac_change]['constraints'].append({'agent': ac_change, 'node': collision['node'],
                                                                      'timestep': collision['timestep']})
                            constraints[ac_change]['constraints'].append({'agent': ac_change, 'node': collision['node'][::-1],
                                                                    'timestep': collision['timestep']})


                            ac.added_constraint = True
                            print('Aircraft', ac_change, 'changed path: Larger id number has priority.')

                        #Generate new path to check if there are no more collisions
                        current_node = inverse_nodes_dictionary[ac.position]['id']
                        goal_node = ac.goal
                        path_lst = []
                        print(ac.id, ac.spawntime, ac.start)
                        for i in ac.path:
                            if i[0] not in path_lst:
                                path_lst.append(i[0])

                        index = path_lst.index(current_node)-1
                        if index > 0 and len(path_lst)>1:
                            if {'agent': ac.id, 'node': [path_lst[index]], 'timestep': t + 0.5} not in constraints[ac.id]['constraints']:
                                print(constraints[ac.id]['constraints'])
                                constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': [path_lst[index]], 'timestep': t + 0.5})
                        succes, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics, t, ac.id,
                                                                 constraints[ac.id]['constraints'])
                        # for node_check in path:
                        #     if ac.last_node == node_check[0]:
                        #         constraints[ac.id]['constraints'].append(
                        #             {'agent': ac.id, 'node': [node_check[0]],
                        #              'timestep': node_check[1]})



                        if succes:
                            ac.path_to_goal = path[1:]
                            paths = [ac.path_to_goal, radar_dict[node]['path']]

                            #If new path has no collisions, update the radar_dict with new generated ac path
                            if len(detect_collisions(paths, id_lst)) == 0:
                                ac.path = path
                                ac_node = inverse_nodes_dictionary[ac.position]['id']
                                path = ac.path_to_goal[:depth_of_path]  # portion of path of a/c that will be communicated

                                #Update radar dict with the new generated path
                                radar_dict[ac_node] = {'path': path, 'ac_id': ac.id, 'type': ac.type,'heading': ac.heading}

                                # Following line adds the visible nodes of the A/C into a dictionary.
                                vision[ac.id] = {'visible_nodes': box_vision(ac.heading, ac.position, inverse_nodes_dictionary)}



    """Implement new constraints in aircraft paths"""
    for ac in aircraft_lst:
        if ac.added_constraint == True:
            current_node = inverse_nodes_dictionary[ac.position]['id']
            goal_node = ac.goal
            # old_path = ac.path
            # path_lst = []

        #     for i in old_path:
        #         if i[0] not in path_lst:
        #             path_lst.append(i[0])
        #     break
        #
        #
        # index = path_lst.index(current_node) - 1
        #
        # if index > 0 and len(path_lst) > 1:
        #     if {'agent': ac.id, 'node': [path_lst[index]], 'timestep': t + 0.5} not in constraints[ac.id]['constraints']:
        #         constraints[ac.id]['constraints'].append({'agent': ac.id, 'node': [path_lst[index]], 'timestep': t + 0.5})

            succes, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics, t, ac.id,
                                                 constraints[ac.id]['constraints'])
            if succes:
                ac.path_to_goal = path[1:]
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]



    return constraints