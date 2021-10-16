"""
Implement CBS here
"""

import heapq
import numpy as np
import random
from single_agent_planner import simple_single_agent_astar # , pop_node, push_node



# def detect_collision(path1, path2):
#     first_collision = []
#
#     for loc1 in path1:
#         for loc2 in path2:
#             if loc1 == loc2:  # for vertex collisions
#                 first_collision = [loc1[0], loc1[1]]
#
#             elif loc1 == path2[path2.index(loc2) + 1] and loc2 == path1[path1.index(loc1) + 1]:  # for edge collisions
#                 first_collision = [(loc1[0], loc2[0]), loc1[1]]

    # return first_collision

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

            elif path1[i] == path2[j + 1] and path2 == path1[i + 1]:  # for edge collisions
                first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]

    return first_collision


def detect_collisions(paths):
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
                                {'a1': agent0, 'a2': agent1, 'node': [first_collision[0][0], first_collision[0][1]],
                                 'timestep': first_collision[1]})
                            return collision_list
                        else:
                            collision_list.append(
                                {'a1': agent0, 'a2': agent1, 'node': [first_collision[0]], 'timestep': first_collision[1]})
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
        collision_split = [{'agent': collision['a1'], 'node': collision['node'], 'timestep': collision['timestep']+1},
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

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints):
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
                'collisions': []}

        for ac in aircraft_lst:
            if ac.status == "taxiing":
                # We do not need start_node since we replan aircraft at their current position. Not implemented yet.
                # start_node = ac.start
                goal_node = ac.goal
                # print(ac.spawntime, ac.id, ac.position)
                print(ac.id, t, ac.spawntime)
                if t != ac.spawntime:
                    for i in ac.path_to_goal:
                        print(i)
                        if i[1] == t:
                            position = ac.path_to_goal[i][0]
                if t == ac.spawntime:
                    position = ac.start
                success, path = simple_single_agent_astar(nodes_dict, position, goal_node, heuristics,
                                                          ac.spawntime, ac.id, constraints)
                print(ac.id, ac.spawntime, path)
                if success:
                    root['paths'].append(path)
                    ac.path_to_goal = path[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path[0][0], next_node_id]
                    #root['paths'].append({'agent': ac.id, 'path': path})
                else:
                    raise Exception("No solution found for", ac.id)

        root['collisions'] = detect_collisions(root['paths'])
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
            # print('Hier je path')

            for constraint in constraints:  # Line 12.
                Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}  # Line 13, new node Q.
                Q['constraints'] = list(P['constraints'])

                Q['constraints'].append(constraint)
                Q['paths'] = P['paths']
                a_i = constraint['agent']  # Line 16, obtaining the agent in the constraint.
                test = (ac.start, goal_node, ac.spawntime, a_i, Q['constraints'])
                for ac in aircraft_lst:
                    if ac.id == a_i:
                        if t != ac.spawntime:
                            for i in ac.path_to_goal:
                                if i[1] == t:
                                    position = ac.path_to_goal[i][0]
                        if t == ac.spawntime:
                            position = ac.start
                success, path = simple_single_agent_astar(nodes_dict, position, Q['paths'][a_i][-1][0], heuristics,
                                                          ac.spawntime, a_i, Q['constraints'])
                # success, path = simple_single_agent_astar(nodes_dict, Q['paths'][a_i][0][0], Q['paths'][a_i][-1][0], heuristics,
                #                                           ac.spawntime, a_i, Q['constraints'])
                # for ac in aircraft_lst:
                #     if ac.id == a_i:
                #         success, path = simple_single_agent_astar(nodes_dict, ac.position[0], ac.goal, heuristics,
                #                                                 ac.spawntime, a_i, Q['constraints'])
                if success == True:
                    # print('succes')

                    for ac in aircraft_lst:
                        if ac.id == a_i:

                            ac.path_to_goal = path[1:]
                            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                            ac.from_to = [path[0][0], next_node_id]
                            break

                    Q['paths'][a_i] = list(path)
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    push_node(open_list, numb_of_generated, Q)
                    numb_of_generated += 1

    return constraints, aircraft_lst

