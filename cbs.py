"""
Implement CBS here
"""

import heapq
import numpy as np
import random
from single_agent_planner import simple_single_agent_astar # , pop_node, push_node
import pandas as pd
import os

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def detect_collision(path1, path2):
    first_collision = []

    for i in range(len(path1)-1):
        for j in range(len(path2)-1):
            if path1[i+1] == path2[j+1] or path1[i] == path2[j]:  # for vertex collisions
                first_collision = [path1[i][0], path1[i][1]]
                print('Vertex')
                return first_collision
            elif path1[i][0] == path2[j + 1][0] and path2[j][0] == path1[i + 1][0]:  # for edge collisions
                first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]
                return first_collision
                # print('Edge')
                # print(first_collision)
            #     print(path1[i], path2[j+1], path2[j], path1[i+1])

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
                            # print({'a1': agent0, 'a2': agent1, 'node': [first_collision[0][0], first_collision[0][1]],
                            #      'timestep': first_collision[1]})

                            collision_list.append(
                                {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0][0], first_collision[0][1]],
                                 'timestep': first_collision[1]})
                            return collision_list
                        else:
                            # print({'a1': agent0, 'a2': agent1, 'node': [first_collision[0]], 'timestep': first_collision[1]})
                            collision_list.append(
                                {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0]], 'timestep': first_collision[1]})
                            return collision_list

        collision_list.append(None)
        return collision_list

def standard_splitting(collision):
    if collision == []:
        return []
    collision = collision[0]
    if collision == None:
        return []

    if len(collision['node']) == 2:
        collision_split = [{'agent': collision['a1'], 'node': [collision['node'][0], collision['node'][1]], 'timestep': collision['timestep']+1},
                           {'agent': collision['a2'], 'node': [collision['node'][1], collision['node'][0]], 'timestep': collision['timestep']+1}]
    else:
        collision_split = [{'agent': collision['a1'], 'node': collision['node'], 'timestep': collision['timestep']},
                           {'agent': collision['a2'], 'node': collision['node'], 'timestep': collision['timestep']}]

    return collision_split

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]

def push_node(open_list, numb_of_generated, node):
    if node['collisions'] == None:
        node['collisions'] = []
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), numb_of_generated , node))

def pop_node(open_list):
    _, _, _, node = heapq.heappop(open_list)
    return node





def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, dict_inverse_nodes):
    # Importing the dictionary to go from (x,y) to the node number.

    numb_of_generated = 0
    open_list = []
    # These four lines create a boolean for replanning. If no new aircraft enters the area then of course we do not need
    # to replan.
    boolean = False
    for ac in aircraft_lst:
        if ac.spawntime == t:
            # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
            # replanning.
            ac.status = "taxiing"
            # print(ac.id, ac.goal)
            ac.position = nodes_dict[ac.start]["xy_pos"]
            boolean = True

    if boolean:
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'id': [],
                'collisions': []}

        for ac in aircraft_lst:
            if ac.status == "taxiing":
                # We do not need start_node since we replan aircraft at their current position.
                goal_node = ac.goal
                current_node = dict_inverse_nodes[ac.position]["id"]
                success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
                                                          t, ac.id, constraints)

                if success:
                    root['paths'].append(path)
                    root['id'].append(ac.id)
                    ac.path_to_goal = path[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path[0][0], next_node_id]
                    #root['paths'].append({'agent': ac.id, 'path': path})
                else:
                    raise Exception("No solution found for", ac.id)

        root['collisions'] = detect_collisions(root['paths'], root['id'])
        root['cost'] = get_sum_of_cost(root['paths'])
        push_node(open_list, numb_of_generated, root)
        numb_of_generated += 1

        while len(open_list) > 0:

            P = pop_node(open_list)
            # print(P['collisions'])
            if P['collisions'] == None:
                print("Is leeg")
                return P['paths']


            collision = P['collisions']

            constraints = standard_splitting(collision)
            # print(constraints)
            # print('Hier je path')

            for constraint in constraints:  # Line 12.
                Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': [], 'id': []}  # Line 13, new node Q.
                Q['constraints'] = list(P['constraints'])
                Q['id'] = P['id']
                Q['constraints'].append(constraint)
                Q['paths'] = P['paths']
                a_i = constraint['agent']  # Line 16, obtaining the agent in the constraint.
                print(a_i)
                for ac in aircraft_lst:
                    if ac.id == a_i:
                        current_node = dict_inverse_nodes[ac.position]["id"]
                        goal_node = ac.goal
                        id = ac.id
                        break

                success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
                                                          t, id, Q['constraints'])

                if success == True:
                    # print('succes')

                    for ac in aircraft_lst:
                        if ac.id == a_i:
                            ac.path_to_goal = path[1:]
                            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                            ac.from_to = [path[0][0], next_node_id]
                            break

                    Q['paths'][Q['id'].index(a_i)] = list(path)
                    Q['collisions'] = detect_collisions(Q['paths'], Q['id'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    push_node(open_list, numb_of_generated, Q)
                    numb_of_generated += 1

    return constraints, aircraft_lst

