import math
from collections import deque
import heapq

DEBUG_FLAG = False
NODES_EXPANSION_LIMIT = 200_000


# making a general class for all algorithms, similar to lab notes
class SearchPlanner:
    def __init__(self, resolution, obstacle_map, motion_model='4n'):
        self.resolution = resolution
        self.obstacle_map = obstacle_map
        self.x_width = len(obstacle_map)
        self.y_width = len(obstacle_map[0])
        # choosing between 4n and 8n
        self.motion = self.get_motion_model_4n() if motion_model == '4n' else self.get_motion_model_8n()

        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = self.x_width - 1, self.y_width - 1

    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

        def __eq__(self, other):
            return self.x == other.x and self.y == other.y

    # plannings
    def planning_bfs(self, start_x, start_y, goal_x, goal_y, search_type='graph', show_animation=False):
        start_node = self.Node(start_x, start_y, 0.0, -1, None)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1, None)
        nodes_expanded = 0
        # FIFO
        queue = deque([start_node])
        visited_nodes = {}
        # for graph search
        closed_set = set() if search_type == 'graph' else None

        while queue:
            nodes_expanded += 1

            self.debug_print(f'BFS {search_type} {nodes_expanded}')

            current = queue.popleft()
            current_id = self.calculate_grid_index(current)

            visited_nodes[current_id] = current

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                route_x, route_y = self.calculate_final_path(current)
                path = [(route_x[i], route_y[i]) for i in range(len(route_x))]
                return path, nodes_expanded, visited_nodes

            if nodes_expanded > NODES_EXPANSION_LIMIT:
                self.debug_print(f'BFS Node Limit Reached')
                return None, nodes_expanded, visited_nodes

            if search_type == 'graph':
                if current_id in closed_set:
                    continue
                closed_set.add(current_id)

            # cost can be used later for better comparison
            for change_x, change_y, cost in self.motion:
                next_x, next_y = current.x + change_x, current.y + change_y

                # boundaries + obstacles checking
                if self.min_x <= next_x <= self.max_x and self.min_y <= next_y <= self.max_y:
                    if not self.obstacle_map[next_y][next_x]:
                        neighbour = self.Node(next_x, next_y, current.cost + cost, current_id, current)
                        neighbour_id = self.calculate_grid_index(neighbour)

                        # tree needs fixing, either it gets stuck on infinite loop and return None, or other issues
                        if (search_type == 'tree') and (self.is_tree_looping(current, neighbour)):
                            continue

                        if (search_type == 'graph') and (neighbour_id in closed_set):
                            continue

                        queue.append(neighbour)
        return None, 0, visited_nodes

    # to add later
    def planning_dfs(self, start_x, start_y, goal_x, goal_y, search_type='graph', show_animation=False):
        start_node = self.Node(start_x, start_y, 0.0, -1, None)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1, None)

        # Stack LIFO
        queue = deque([start_node])
        open_set_store = {self.calculate_grid_index(start_node)}

        visited_nodes = {}
        closed_set = set()

        nodes_expanded = 0

        while queue:

            nodes_expanded += 1
            self.debug_print(f'DFS {search_type} {nodes_expanded}')

            parent = queue.pop()
            parent_id = self.calculate_grid_index(parent)

            if search_type == 'graph':
                open_set_store.remove(parent_id)

            visited_nodes[parent_id] = parent

            if parent == goal_node:
                route_x, route_y = self.calculate_final_path(parent)
                path = [(route_x[i], route_y[i]) for i in range(len(route_x))]
                return path, nodes_expanded, visited_nodes

            if nodes_expanded > NODES_EXPANSION_LIMIT:
                self.debug_print(f'DFS Node Limit Reached')
                return None, nodes_expanded, visited_nodes

            closed_set.add(parent_id)

            for change_x, change_y, cost in self.motion:
                next_x, next_y = parent.x + change_x, parent.y + change_y

                if not (self.min_x <= next_x <= self.max_x and self.min_y <= next_y <= self.max_y):
                    continue

                if self.obstacle_map[next_y][next_x]:
                    continue

                child = self.Node(next_x, next_y, parent.cost + cost, parent_id, parent)
                child_id = self.calculate_grid_index(child)

                if (child_id in open_set_store) and (search_type == 'graph'):
                    continue

                if (child_id in closed_set) and (search_type == 'graph'):
                    continue

                if (search_type == 'tree') and (self.is_tree_looping(parent, child)):
                    continue

                queue.append(child)
                open_set_store.add(child_id)

        return None, 0, visited_nodes

    def planning_ucs(self, start_x, start_y, goal_x, goal_y, search_type='graph', show_animation=False):
        start_node = self.Node(start_x, start_y, 0.0, -1, None)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1, None)

        p_queue = []
        heapq.heappush(p_queue, (start_node.cost, self.calculate_grid_index(start_node), start_node))
        lookup_dict = {}

        # open_set = dict()
        closed_set = set()
        visited_nodes = dict()

        # open_set[self.calculate_grid_index(start_node)] = start_node
        lookup_dict[self.calculate_grid_index(start_node)] = start_node

        nodes_expanded = 0

        while p_queue:
            nodes_expanded += 1
            self.debug_print(f'UCS {search_type} {nodes_expanded}')

            _, parent_id, parent = heapq.heappop(p_queue)

            # parent_id = min(open_set, key=lambda o: open_set[o].cost)
            # parent = open_set[parent_id]

            visited_nodes[parent_id] = parent

            if parent == goal_node:
                route_x, route_y = self.calculate_final_path(parent)
                path = [(route_x[i], route_y[i]) for i in range(len(route_x))]
                return path, nodes_expanded, visited_nodes

            if nodes_expanded > NODES_EXPANSION_LIMIT:
                self.debug_print(f'UCS Node Limit Reached')
                return None, nodes_expanded, visited_nodes

            # del open_set[parent_id]

            if search_type == 'graph':
                closed_set.add(parent_id)
                lookup_dict.pop(parent_id)
                # closed_set[parent_id] = parent

            for change_x, change_y, cost in self.motion:
                next_x, next_y = parent.x + change_x, parent.y + change_y

                if not (self.min_x <= next_x <= self.max_x and self.min_y <= next_y <= self.max_y):
                    continue

                if self.obstacle_map[next_y][next_x]:
                    continue

                child = self.Node(next_x, next_y, parent.cost + cost, parent_id, parent)
                child_id = self.calculate_grid_index(child)

                if search_type == 'tree' and self.is_tree_looping(parent, child):
                    continue

                if search_type == 'tree':
                    heapq.heappush(p_queue, (child.cost, child_id, child))
                    # open_set[child_id] = child
                    continue

                if child_id in closed_set and search_type == 'graph':
                    continue

                if (child_id not in lookup_dict) or (child.cost < lookup_dict[child_id].cost):
                    heapq.heappush(p_queue, (child.cost, child_id, child))
                    lookup_dict[child_id] = child
                    continue

        return None, 0, visited_nodes

    def planning_astar(self, start_x, start_y, goal_x, goal_y, heuristic_type='euclidean', search_type='graph', show_animation=False):
        start_node = self.Node(start_x, start_y, 0.0, -1, None)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1, None)

        # switching to heapq for better performance: https://docs.python.org/3/library/heapq.html
        priority_queue = []
        # can store all info alongside priority queue
        lookup_dictionary = {}
        # mentioned before how I had open/closed sets which kinda worked the same, so trying to keep only one
        closed_set = set()  # set works better than dictionary because only IDs

        start_id = self.calculate_grid_index(start_node)

        # adding the start node to the queue and the initial heuristic calculation
        heapq.heappush(priority_queue,
                       (self.calculate_heuristic(start_node, goal_node, heuristic_type), start_id, start_node))
        # just adding extra info, will see it later on also next to heappush
        lookup_dictionary[start_id] = start_node

        visited_nodes = {start_id: start_node}

        nodes_expanded = 0

        while priority_queue:
            nodes_expanded += 1
            self.debug_print(f'A* {search_type} {nodes_expanded}')

            # so this the main optimisation with heapq, instead of me using lambda with min every time which is
            # O(n), this does is O(log(n)) by popping the smallest value, and ignoring the first value which is the f(x)
            _, current_id, current = heapq.heappop(priority_queue)

            # https://gamedev.stackexchange.com/questions/210778/a-what-to-do-about-duplicate-items-in-the-min-heap-if-anything
            # basically point of this rework is to make it log(n), but heapq doesn't have way to remove entries so its
            # easy to get duplicates, so here I just skip it
            if current_id in closed_set:
                continue

            if search_type == 'graph':
                current = lookup_dictionary.pop(current_id)
                closed_set.add(current_id)

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                # goal_node.parent_index = current.parent_index
                # goal_node.cost = current.cost
                goal_node = current
                visited_nodes[current_id] = current
                break

            if nodes_expanded > NODES_EXPANSION_LIMIT:
                self.debug_print(f'A* Node Limit Reached')
                return None, nodes_expanded, visited_nodes

            for change_x, change_y, cost in self.motion:
                next_x = current.x + change_x
                next_y = current.y + change_y

                if not (0 <= next_x < self.x_width and 0 <= next_y < self.y_width):
                    continue

                if self.obstacle_map[next_y][next_x]:
                    continue

                new_cost = current.cost + cost
                neighbour = self.Node(next_x, next_y, new_cost, current_id, current)
                neighbour_id = self.calculate_grid_index(neighbour)

                if search_type == 'tree' and self.is_tree_looping(current, neighbour):
                    continue

                if search_type == 'tree':
                    heapq.heappush(priority_queue,
                                   ((new_cost + self.calculate_heuristic(neighbour, goal_node, heuristic_type)),
                                    neighbour_id, neighbour))
                    visited_nodes[neighbour_id] = neighbour
                    continue

                if search_type == 'graph' and neighbour_id in closed_set:
                    continue

                if neighbour_id not in lookup_dictionary:
                    # copying this from the start with the new cost +
                    heapq.heappush(priority_queue,
                                   ((new_cost + self.calculate_heuristic(neighbour, goal_node, heuristic_type)),
                                    neighbour_id, neighbour))
                    lookup_dictionary[neighbour_id] = neighbour
                    visited_nodes[neighbour_id] = neighbour
                elif new_cost < lookup_dictionary[neighbour_id].cost:
                    heapq.heappush(priority_queue,
                                   ((new_cost + self.calculate_heuristic(neighbour, goal_node, heuristic_type)),
                                    neighbour_id, neighbour))
                    lookup_dictionary[neighbour_id] = neighbour
                    visited_nodes[neighbour_id] = neighbour

        route_x, route_y = self.calculate_final_path(goal_node)
        path = [(route_x[i], route_y[i]) for i in range(len(route_x))]

        return path, nodes_expanded, visited_nodes

    # Checks if the child node has already been in the current path of the parent if so its a loop(true)
    @staticmethod
    def is_tree_looping(parent, child):
        while parent.parent is not None:
            if parent.parent == child:
                return True
            parent = parent.parent
        return False

    # Prints Debugging Messages If want to debug
    @staticmethod
    def debug_print(text):
        if DEBUG_FLAG:
            print(text)

    # will be used later, similar to lab files
    def calculate_heuristic(self, current_node, goal_node, heuristic_type='euclidean'):
        if heuristic_type == 'manhattan':
            return self.calculate_heuristic_manhattan(current_node, goal_node)
        elif heuristic_type == 'euclidean':
            return self.calculate_heuristic_euclidean(current_node, goal_node)
        else:
            return 0

    @staticmethod
    def calculate_heuristic_euclidean(current_node, goal_node):
        heuristic_weight = 1
        return heuristic_weight * math.hypot(current_node.x - goal_node.x, current_node.y - goal_node.y)

    @staticmethod
    def calculate_heuristic_manhattan(current_node, goal_node):
        heuristic_weight = 1
        return heuristic_weight * abs(current_node.x - goal_node.x) + abs(current_node.y - goal_node.y)

    def calculate_final_path(self, goal_node):
        route_x = [self.calculate_grid_position(goal_node.x, self.min_x)]
        route_y = [self.calculate_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        node = goal_node
        # loop = 0

        while parent_index != -1:
            # loop += 1
            # print(f'loop: {loop}')
            node = node.parent
            route_x.append(self.calculate_grid_position(node.x, self.min_x))
            route_y.append(self.calculate_grid_position(node.y, self.min_y))
            parent_index = node.parent_index

        # reverse to get from start to goal
        route_x.reverse()
        route_y.reverse()
        return route_x, route_y

    def calculate_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def calculate_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    @staticmethod
    def get_motion_model_8n():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion

    @staticmethod
    def get_motion_model_4n():
        motion = [[-1, 0, 1],
                  [0, 1, 1],
                  [1, 0, 1],
                  [0, -1, 1]]
        return motion
