"""
Implement CBS here
"""

import heapq
import random
from single_agent_planner import simple_single_agent_astar # , pop_node, push_node

# paths = []
# open_list = []

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

# def detect_collision(path1, path2):
#     first_collision = []
#
#     for i in range(len(path1)-1):
#         for j in range(len(path2)-1):
#             if path1[i+1] == path2[j+1] or path1[i] == path2[j]:  # for vertex collisions
#                 first_collision = [path1[i][0], path1[i][1]]
#
#             elif path1[i] == path2[j + 1] and path2 == path1[i + 1]:  # for edge collisions
#                 first_collision = [(path1[i][0], path2[j][0]), path1[i][1]]
#
#     return first_collision


# def detect_collisions(paths):
#     collision_list = []
#
#     if len(paths) >= 2:
#         for agent0 in paths:
#             for agent1 in paths:
#                 if agent0['agent'] == agent1['agent']:
#                     continue
#                 else:
#                     first_collision = detect_collision(agent0['path'], agent1['path'])
#                     if len(first_collision) == 0:
#                         continue
#
#                     else:
#                         # print('first collision = ', first_collision, type(first_collision[0][0]))
#                         if type(first_collision[0]) == tuple:
#                             collision_list.append(
#                                 {'a1': agent0['agent'], 'a2': agent1['agent'], 'loc': [first_collision[0][0], first_collision[0][1]],
#                                  't_step': first_collision[1]})
#                             return collision_list
#                         else:
#                             collision_list.append(
#                                 {'a1': agent0['agent'], 'a2': agent1['agent'], 'loc': [first_collision[0]], 't_step': first_collision[1]})
#                             return collision_list
#     else:
#         collision_list.append(None)
#
#     return collision_list

# def standard_splitting(collision):
#
#     collision = collision[0]
#     if collision == None:
#         return None
#
#     if len(collision['loc']) == 2:
#         collision_split = [{'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']+1},
#                            {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 't_step': collision['t_step']+1}]
#     else:
#         collision_split = [{'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']},
#                            {'agent': collision['a2'], 'loc': collision['loc'], 't_step': collision['t_step']}]
#
#     return collision_split

def standard_splitting(collision):
    return_list = []
    agent1 = collision['a1']
    agent2 = collision['a2']
    time_step = collision['time_step']

    # Vertex collision
    if len(collision['loc']) == 1:
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'time_step': time_step})
        return_list.append({'agent': agent2, 'loc': location, 'time_step': time_step})

    # Edge collision
    if len(collision['loc']) == 2:
        location = collision['loc']

        return_list.append({'agent': agent1, 'loc': location, 'time_step': time_step})
        return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'time_step': time_step})
    return return_list

def detect_collision(path1, path2):
    time = max(len(path1), len(path2))
    for time in range(1, time):
        # Vertex collisions
        node_location_1 = get_location(path1, time)
        node_location_2 = get_location(path2, time)
        if node_location_1 == node_location_2:
            return {'a1': 0, 'a2': 1, 'loc': [node_location_1], 'time_step': time}

        # Edge collisions
        node_location_t_1 = get_location(path1, time-1)
        node_location_t_2 = get_location(path2, time-1)
        if node_location_1 == node_location_t_2 and node_location_2 == node_location_t_1:
            return {'a1': 0,'a2': 1, 'loc': [node_location_t_1, node_location_1], 'time_step': time}

    return None

def detect_collisions(paths):
    collisions = []
    for agent0 in range(len(paths)):
        for agent1 in range(agent0 + 1, len(paths)):
            if agent0 == agent1:
                continue
            collision = detect_collision(paths[agent0], paths[agent1])

            if collision != None:
                collision['a1'] = agent0
                collision['a2'] = agent1
                collisions.append(collision)

    return collisions

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]

def push_node(open_list, node):
    heapq.heappush(open_list, (node['cost'], len(node['collisions']), node))


def pop_node(open_list):
    _, _, node = heapq.heappop(open_list)
    return node

def run_CBS(aircraft_lst, nodes_dict, heuristics, t, constraints):
    open_list = []

    # These four lines create a boolean for replanning. If no new aircraft enters the area then of course we do not need
    # to replan.
    boolean = False
    for ac in aircraft_lst:
        if ac.spawntime == t:
            # The two lines below trigger the visualisation and correct position of the aircraft. The last line triggers
            # replanning.
            ac.status = "taxiing"
            print(ac.id, ac.goal)
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
                success, path = simple_single_agent_astar(nodes_dict, ac.start, goal_node, heuristics,
                                                          ac.spawntime, ac.id, constraints)
                if success:
                    root['paths'].append(path)
                    # paths.append({'agent': ac.id, 'path': path})
                else:
                    raise Exception("No solution found for", ac.id)

        root['collisions'] = detect_collisions(root['paths'])
        root['cost'] = get_sum_of_cost(root['paths'])
        push_node(open_list, root)

        while len(open_list) > 0:

            P = pop_node(open_list)
            # print(P['paths'])
            print(P['collisions'])
            if len(P['collisions']) == 0:
                return P['paths']
            # print('Hier kom ik niet')
            collision = P['collisions']
            # print('Collision: ', collision)
            # print(len(P['collisions']))
            constraints = standard_splitting(collision)

            # Code below is copy pasted from my warmup exercise.
            for constraint in constraints:  # Line 12.
                Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}  # Line 13, new node Q.
                Q['constraints'] = list(P['constraints'])

                Q['constraints'].append(constraint)
                Q['paths'] = P['paths']
                a_i = constraint['agent']  # Line 16, obtaining the agent in the constraint.
                success, path = simple_single_agent_astar(nodes_dict, ac.start, goal_node, heuristics,
                                                          ac.spawntime, a_i, constraints)

                if success == True:
                    print('succes')
                    for ac in aircraft_lst:
                        if ac.id == a_i:

                            ac.path_to_goal = path[1:]
                            next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                            ac.from_to = [path[0][0], next_node_id]
                            break

                    Q['paths'][a_i] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    push_node(open_list, Q)

    return constraints, aircraft_lst

