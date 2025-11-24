import math
from collections import deque


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
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    # plannings
    def planning_bfs(self, start_x, start_y, goal_x, goal_y, search_type='graph'):
        start_node = self.Node(start_x, start_y, 0.0, -1)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1)

        # FIFO
        queue = deque([start_node])
        visited_nodes = {}
        # for graph search
        closed_set = set() if search_type == 'graph' else None

        while queue:
            current = queue.popleft()
            current_id = self.calculate_grid_index(current)

            visited_nodes[current_id] = current

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                route_x, route_y = self.calculate_final_path(current, visited_nodes)
                path = [(route_x[i], route_y[i]) for i in range(len(route_x))]
                return path, route_y

            if search_type == 'graph':
                if current_id in closed_set:
                    continue
                closed_set.add(current_id)

            # cost can be used later for better comparison
            for change_x, change_y, cost in self.motion:
                next_x, next_y = current.x + change_x, current.y + change_y

                # boundaries + obstacles checking
                if self.min_x <= next_x <= self.max_x and self.min_y <= next_y <= self.max_y:
                    if not self.obstacle_map[next_x][next_y]:
                        neighbour = self.Node(next_x, next_y, current.cost + cost, current_id)
                        neighbour_id = self.calculate_grid_index(neighbour)

                        # tree needs fixing, either it gets stuck on infinite loop and return None, or other issues
                        if search_type == 'tree':
                            if neighbour_id != current.parent_index:
                                queue.append(neighbour)
                        else:
                            if neighbour_id not in closed_set:
                                queue.append(neighbour)
        return None, None

    # to add later
    def planning_dfs(self, start_x, start_y, goal_x, goal_y, search_type='graph'):
        start_node = self.Node(start_x, start_y, 0.0, -1)
        goal_node = self.Node(goal_x, goal_y, 0.0, -1)

        #Stack FIFO
        queue = deque([start_node])

        visited_nodes = {}
        closed_set = set() if search_type == 'graph' else None

        while queue:
            current = queue.pop()
            # calculates the currents node index to make comparison/searching etc easier aka V in psuedo code
            current_id = self.calculate_grid_index(current)
            visited_nodes[current_id] = current

            #If Goal, Then returns the path from the state to the goal
            if (current.x, current.y) == (goal_node.x, goal_node.y):
                route_x, route_y = self.calculate_final_path(current, visited_nodes)
                path = [(route_x[i], route_y[i]) for i in range(len(route_x))]
                return path, route_y

            #Loops Over All The possible neighbour nodes based on the predefined motion (e.g 4 or 8 neighnours)
            # Adds the neighbouring nodes to the openset
            for change_x, change_y, cost in self.motion:
                #Gets the neighbours x,y
                next_x, next_y = current.x + change_x, current.y + change_y

                # boundaries + obstacles checking
                if self.min_x <= next_x <= self.max_x and self.min_y <= next_y <= self.max_y:
                    if not self.obstacle_map[next_x][next_y]:
                        # Creates A Node for the neigbour if its not an obstical
                        neighbour = self.Node(next_x, next_y, current.cost + cost, current_id)
                        # Generates it index so it can be easily searched an identified if it is already in the closed set
                        neighbour_id = self.calculate_grid_index(neighbour)

                        # tree needs fixing, either it gets stuck on infinite loop and return None, or other issues
                        if search_type == 'tree':
                            if neighbour_id != current.parent_index:
                                queue.append(neighbour)
                        else:
                            if neighbour_id not in closed_set:
                                queue.append(neighbour)
        return None, None

    def planning_ucs(self, start_x, start_y, goal_x, goal_y, search_type='graph'):
        return None

    def planning_astar(self, start_x, start_y, goal_x, goal_y, heuristic_type='euclidean', search_type='graph'):
        return None

    # will be used later, similar to lab files
    def _calc_heuristic(self, current_node, goal_node, heuristic_type='euclidean'):
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

    def calculate_final_path(self, goal_node, visited_nodes):
        route_x = [self.calculate_grid_position(goal_node.x, self.min_x)]
        route_y = [self.calculate_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            node = visited_nodes.get(parent_index)
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
