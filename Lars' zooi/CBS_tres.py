import heapq
import numpy as np
import numpy.random as rnd
from single_agent_planner import simple_single_agent_astar # , pop_node, push_node
import pandas as pd
import os
import time as timer

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
                if agent0 == agent1:
                    continue
                else:
                    first_collision = detect_collision(paths[agent0], paths[agent1])
                    if len(first_collision) == 0:
                        continue
                    else:
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

def standard_splitting(collision):
    if collision == []:
        return []
    collision = collision[0]
    if collision == None:
        return []
    if len(collision['node']) == 2:
        collision_split = [{'agent': collision['a1'], 'node': [collision['node'][0], collision['node'][1]], 'timestep': collision['timestep']+0.5},
                           {'agent': collision['a2'], 'node': [collision['node'][1], collision['node'][0]], 'timestep': collision['timestep']-0.5}]
    else:
        collision_split = [{'agent': collision['a1'], 'node': collision['node'], 'timestep': collision['timestep']},
                           {'agent': collision['a2'], 'node': collision['node'], 'timestep': collision['timestep']}]

    return collision_split

def push_node(open_list, node, num_of_generated):
    #print('hier7')
    if node['collisions'] == None:
        node['collisions'] = []
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), num_of_generated, node))

def pop_node(open_list):
    #print('hier8')
    _, _, id, node = heapq.heappop(open_list)
    # print("Expand node {}".format(id))
    return node

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints, dict_inverse_nodes):

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
        #print('hier6')
        if ac.status == "taxiing":
            dict = {"id": ac.id, "Heading": ac.heading, "Position": ac.path_to_goal[0][0], "Goal_node": ac.goal}
            if dict["Heading"] == 270 and dict["Position"] in gate_block_nodes:
                blocked_list.append(ac.goal)
            if ac.type == "A":
                if ac.path_to_goal[0][0] in gate_intersection_dict[ac.goal]['nodes']:
                    blocked_list.append(ac.goal)

    boolean = False
    for ac in aircraft_lst:
        #print('hier6')
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
            # print('Aircraft ', ac.id, ' spawned at t = ', t)
            # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
            # replanning.
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            boolean = True

    for ac in aircraft_lst:
        if ac.status == "taxiing":
            if ac.spawntime==t:
                temp_lst = ac.node_last
                temp_lst.append(dict_inverse_nodes[ac.position]['id'])
                ac.node_last = temp_lst
                # print(t, ac.node_last)
            # print(t, ac.node_last)
            if ac.node_last[-1] != dict_inverse_nodes[ac.position]['id'] and ac.spawntime != t:
                ac.node_last.append(dict_inverse_nodes[ac.position]['id'])
                ac.node_last = ac.node_last[-2:]

    if boolean:
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'id': [],
                'collisions': []}
        active_ac_lst = []
        numb_of_generated = 0
        open_list = []
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                #print('hier5')
                active_ac_lst.append(ac.id)
                # We do not need start_node since we replan aircraft at their current position.
                goal_node = ac.goal
                current_node = dict_inverse_nodes[ac.position]["id"]
                success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
                                                          t, ac.id, constraints)
                if success:
                    root['paths'].append(path)
                    root['id'].append(ac.id)
                else:
                    raise Exception("No solution found for", ac.id)

        root['collisions'] = detect_collisions(root['paths'], root['id'])
        root['cost'] = get_sum_of_cost(root['paths'])
        push_node(open_list, root, numb_of_generated)
        numb_of_generated += 1

        while len(open_list) > 0:
            #print('hier1')
            if numb_of_generated > 10000:
                return False
            P = pop_node(open_list)

            if len(P['paths']) >= 2:
                P['collisions'] = detect_collisions(P['paths'], P['id']) #Check for any collissions in current node

            if len(P['collisions']) == 0 or P['collisions'][0] == None:
                for ac in aircraft_lst:
                    if ac.id in active_ac_lst:
                        path = P['paths'][P['id'].index(ac.id)]
                        ac.path_to_goal = path[1:]
                        next_node_id = ac.path_to_goal[0][0]
                        ac.from_to = [path[0][0], next_node_id]
                return aircraft_lst

            collision = P['collisions']
            # print(P['collisions'])
            constraints = standard_splitting(collision)
            # print('hi')
            for constraint in constraints:
                #print('hier2')
                Q = {'cost': 0,
                     'constraints': [],
                     'paths': P['paths'],
                     'id': P['id'],
                     'collisions': []}
                Q['constraints'] = list(P['constraints'])
                Q['constraints'].append(constraint)
                a_i = constraint['agent']

                for ac in aircraft_lst:
                    if ac.id == a_i:
                        #print('hier3')
                        goal_node = ac.goal
                        current_node = dict_inverse_nodes[ac.position]["id"]
                        break
                # if {'agent': ac.id, 'node': [ac.node_last[-2]], 'timestep': t + 0.5} not in Q['constraints'] and ac.spawntime!=t:
                #     Q['constraints'].append({'agent': ac.id, 'node': [ac.node_last[-2]], 'timestep': t + 0.5})
                success, path = simple_single_agent_astar(nodes_dict, current_node, goal_node, heuristics,
                                           t, ac.id, Q['constraints'])
                if success:
                    #print('hier4')
                    Q['paths'][Q['id'].index(a_i)] = list(path)
                    Q['collisions'] = detect_collisions(Q['paths'], Q['id'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    push_node(open_list, Q, numb_of_generated)
                    numb_of_generated += 1
                    if numb_of_generated%500==0:
                        print(numb_of_generated)
                else:
                    return False