"""
Implement CBS here
"""

import heapq
import numpy as np
import numpy.random as rnd
from single_agent_planner import simple_single_agent_astar # , pop_node, push_node
import pandas as pd
import os


#Definitions
def get_heading(heading, xy_start, xy_next):
    """
    Function blatantly stolen from the Aircraft class. In case of replanning events, no heading change may be larger
    than 90 degrees.
    """
    if xy_start[0] == xy_next[0]:  # moving up or down
        if xy_start[1] > xy_next[1]:  # moving down
            heading = 180
        elif xy_start[1] < xy_next[1]:  # moving up
            heading = 0
        else:
            heading = heading

    elif xy_start[1] == xy_next[1]:  # moving right or left
        if xy_start[0] > xy_next[0]:  # moving left
            heading = 90
        elif xy_start[0] < xy_next[0]:  # moving right
            heading = 270
        else:
            heading = heading
    else:
        raise Exception("Invalid movement")
    return heading

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def detect_collision(path1, path2):
    first_collision = []
    for i in range(len(path1)-1):
        for j in range(len(path2)-1):
            if path1[i] == path2[j]:# for vertex collisions
                first_collision = [path1[i][0], path1[i][1]]
                return first_collision
            elif path1[i][0] == path2[j + 1][0] and path2[j][0] == path1[i + 1][0] \
                    and path1[i][1] == path2[j][1] and path1[i+1][1] == path2[j+1][1]:  # for edge collisions
                first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]
                return first_collision
    if path1[i+1] == path2[j+1]:
        first_collision = [path1[i+1][0], path1[i+1][1]]
        return first_collision

    return first_collision

def detect_collisions(paths, id):
    collision_list = []
    if len(paths) >= 2:
        for agent0 in range(len(paths)):
            for agent1 in range(agent0, len(paths)):
                # if id[agent0] == 1 and id[agent1] == 6:
                #     print(id[agent0], paths[agent0])
                #     print(id[agent1], paths[agent1])
                # if id[agent0] == 6 and id[agent1] == 1:
                #     print(id[agent0], paths[agent0])
                #     print(id[agent1], paths[agent1])
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
                                 'timestep': first_collision[1]})
                            return collision_list
                        else:
                            collision_list.append(
                                {'a1': id[agent0], 'a2': id[agent1], 'node': [first_collision[0]], 'timestep': first_collision[1]})
                            return collision_list

        collision_list.append(None)
        return collision_list

print(detect_collisions([[(20, 7.0), (70.0, 7.5), (19.0, 8.0), (63.0, 8.5), (12.0, 9.0), (56.0, 9.5), (11.0, 10.0), (55.0, 10.5), (10.0, 11.0), (52.0, 11.5), (39.0, 12.0), (48.0, 12.5), (3.0, 13.0), (42.0, 13.5), (40.0, 14.0), (43.0, 14.5), (4.0, 15.0), (95.0, 15.5), (1.0, 16.0)],
                         [(64, 7.0), (64, 7.5), (64, 8.0), (20.0, 8.5), (71.0, 9.0), (21.0, 9.5), (77.0, 10.0), (27.0, 10.5), (86.0, 11.0), (32.0, 11.5), (94.0, 12.0), (36.0, 12.5)]],
                        [1,6]))


def standard_splitting(collision):
    if collision == []:
        return []
    collision = collision[0]
    if collision == None:
        return []
    if collision['node'] == [64]:
        print(collision)
    if len(collision['node']) == 2:
        collision_split = [{'agent': collision['a1'], 'node': [collision['node'][0], collision['node'][1]], 'timestep': collision['timestep']+0.5},
                           {'agent': collision['a2'], 'node': [collision['node'][1], collision['node'][0]], 'timestep': collision['timestep']+0.5}]
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
    # print('Nodes generated: ', numb_of_generated)
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), numb_of_generated , node))

def pop_node(open_list):
    _, _, _, node = heapq.heappop(open_list)
    return node



#Run Conflict Based Search program

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, dict_inverse_nodes):
    statement = True
    gate_nodes = [97, 34, 35, 36, 98]
    gate_block_nodes = [97, 34, 35, 36, 98]
    gate_intersection_dict = {97: {'nodes': [83, 88, 29, 99]},
                              34: {'nodes': [88, 84, 89, 30, 92]},
                              35: {'nodes': [93, 31, 85, 89, 90]},
                              36: {'nodes': [94, 32, 86, 90, 91]},
                              98: {'nodes': [100, 33, 91, 87]}}
    numb_of_generated = 0
    open_list = []

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
    boolean = False
    for ac in aircraft_lst:
        if ac.spawntime == t:

            if ac.start in blocked_list:
                random_string = rnd.choice(["spawntime", "start_location"])
                if random_string == "spawntime":
                    ac.spawntime = ac.spawntime + 0.5
                if random_string == "start_location":
                    counter = 0
                    while ac.start in blocked_list:
                        ac.start = rnd.choice(gate_nodes)
                        counter = counter + 1
                        if counter > 4:
                            ac.spawntime = ac.spawntime + 0.5
                            break
                continue
            print('Aircraft ', ac.id, ' spawned at t = ', t)
            # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
            # replanning.
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            boolean = True
    # if t==8:
    #     for ac in aircraft_lst:
    #         print(ac.id, ac.spawntime, ac.start)

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
                    if t == ac.spawntime:
                        ac.path = path
                    root['paths'].append(path)
                    root['id'].append(ac.id)
                else:
                    raise Exception("No solution found for", ac.id)

        root['collisions'] = detect_collisions(root['paths'], root['id'])
        root['cost'] = get_sum_of_cost(root['paths'])
        push_node(open_list, numb_of_generated, root)
        numb_of_generated += 1

        while len(open_list) > 0:

            P = pop_node(open_list)

            if len(P['paths']) >= 2:
                P['collisions'] = detect_collisions(P['paths'], P['id']) #Check for any collissions in current node

            if len(P['collisions']) == 0 or P['collisions'][0] == None:
                for ac in aircraft_lst:
                    if ac.id in P['id']:
                        path = P['paths'][P['id'].index(ac.id)]
                        ac.path = path
                        # if ac.spawntime==t:
                        ac.path_to_goal = path[1:]
                        # else:
                        #     ac.path_to_goal = path
                        next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                        ac.from_to = [path[0][0], next_node_id]
                return P['paths']


            collision = P['collisions']
            print(collision)
            constraints = standard_splitting(collision)
            # print(constraints)
            for constraint in constraints:  # Line 12.
                # print('Ga weer de for loop in', constraint)
                Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': [], 'id': []}  # Line 13, new node Q.
                Q['constraints'] = list(P['constraints'])
                Q['id'] = P['id']
                Q['constraints'].append(constraint)
                Q['paths'] = P['paths']
                a_i = constraint['agent']  # Line 16, obtaining the agent in the constraint.
                for ac in aircraft_lst:
                    if ac.id == a_i:
                        current_node = dict_inverse_nodes[ac.position]["id"]
                        goal_node = ac.goal
                        id = ac.id
                        heading = ac.heading
                        old_path = ac.path
                        path_lst = []
                        for i in old_path:
                            if i[0] not in path_lst:
                                path_lst.append(i[0])
                        # if ac.id==6:
                        #     print(current_node, path_lst)
                        break
                index = path_lst.index(current_node)-1
                # if ac.id == 1:
                #     print(ac.path_to_goal)
                # -------------------------------------------------------------------
                # Als je deze twee lines hieronder comment, dan werkt ie wel maar gaan sommige achteruit.
                # Deze twee lines voegen een constraint toe, maar zorgen voor een infinite loop..?
                if index > 0 and len(path_lst) > 1:
                    if ac.id == 1:
                        print('Hier: ', path_lst[index])
                    if {'agent': ac.id, 'node': [path_lst[index]], 'timestep': t+0.5} not in Q['constraints']:
                        Q['constraints'].append({'agent': ac.id, 'node': [path_lst[index]], 'timestep': t+0.5})

                success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
                                                          t, id, Q['constraints'])
                if success == True:
                    if statement == False and ac.id == 6:
                        print(path)
                    Q['paths'][Q['id'].index(a_i)] = list(path)
                    Q['collisions'] = detect_collisions(Q['paths'], Q['id'])
                    # print()
                    # print('Q; ', Q['collisions'])
                    # print()
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    push_node(open_list, numb_of_generated, Q)
                    numb_of_generated += 1
                # else:
                #     for i in
                #     print(Q['constraints'])
                    #
                    # raise Exception('Error, no path found')
    return constraints, aircraft_lst

