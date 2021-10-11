"""
Implement CBS here
"""


import random
from single_agent_planner import simple_single_agent_astar, pop_node, push_node

paths = []
open_list = []

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
        for agent0 in paths:
            for agent1 in paths:
                if agent0['agent'] == agent1['agent']:
                    continue
                else:
                    first_collision = detect_collision(agent0['path'], agent1['path'])
                    if len(first_collision) == 0:
                        continue

                    else:
                        # print('first collision = ', first_collision, type(first_collision[0][0]))
                        if type(first_collision[0]) == tuple:
                            collision_list.append(
                                {'a1': agent0['agent'], 'a2': agent1['agent'], 'loc': [first_collision[0][0], first_collision[0][1]],
                                 't_step': first_collision[1]})
                            return collision_list
                        else:
                            collision_list.append(
                                {'a1': agent0['agent'], 'a2': agent1['agent'], 'loc': [first_collision[0]], 't_step': first_collision[1]})
                            return collision_list
    else:
        collision_list.append(None)

    return collision_list

def standard_splitting(collision):

    collision = collision[0]
    if collision == None:
        return None

    if len(collision['loc']) == 2:
        collision_split = [{'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']+1},
                           {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 't_step': collision['t_step']+1}]
    else:
        collision_split = [{'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']},
                           {'agent': collision['a2'], 'loc': collision['loc'], 't_step': collision['t_step']}]

    return collision_split

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints):

    root = {'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': []}

    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            start_node = ac.start
            goal_node = ac.goal
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics,
                                                      ac.spawntime, ac.id, constraints)
            if success:
                root['paths'].append(path)
                paths.append({'agent': ac.id, 'path': path})

            else:
                raise Exception("No solution found for", ac.id)

    root['collisions'] = detect_collisions(root['paths'])
    root['cost'] = get_sum_of_cost(root['paths'])
    push_node(open_list, root)




    return constraints, aircraft_lst

