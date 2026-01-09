import numpy as np
import pandas as pd
import json
import time
import matplotlib.pyplot as plt
import matplotlib
import tracemalloc
from algorithms import SearchPlanner

from maze_generate import maze_generate

# Simplest way to run on Macs
matplotlib.use('MacOSX')

# for windows
# matplotlib.use('TkAgg')


show_animation = True
pause_time = 0.05
random_seed = 60

algorithm_titles = {'1': 'BFS', '2': 'DFS', '3': 'UCS', '4': 'A*'}
search_type_titles = {'1': 'tree', '2': 'graph'}


def load_parameters(config_file='./MapConfig/config9x9.json'):
    with open(config_file) as config_env:
        parameters = json.load(config_env)
    return parameters


def load_map_layout(map_xlsx):
    gmap = pd.read_excel(map_xlsx, header=None)
    data = gmap.to_numpy()
    data = data[::-1]
    return data


def get_cell_type(data):
    obstacle_x, obstacle_y, empty_x, empty_y = [], [], [], []
    for index_y in range(data.shape[0]):
        for index_x in range(data.shape[1]):
            if data[index_y, index_x] == 1:
                obstacle_x.append(index_x)
                obstacle_y.append(index_y)
            else:
                empty_x.append(index_x)
                empty_y.append(index_y)
    return obstacle_x, obstacle_y, empty_x, empty_y


def create_obstacle_map(data):
    return data.tolist()


# General class for animating, need to add on to it as currently only shows the final path.
class AnimatedSearch:
    def __init__(self, obstacle_map, start, goal, algorithm_planner, algorithm, search_type, fig_dim,
                 show_animation=True):
        self.obstacle_map = obstacle_map
        self.start = start
        self.goal = goal
        self.algorithm = algorithm
        self.search_type = search_type
        self.algorithm_planner = algorithm_planner
        self.width, self.height = len(obstacle_map[0]), len(obstacle_map)
        self.show_animation = show_animation

        if show_animation:
            self.fig, self.ax = plt.subplots(figsize=(fig_dim, fig_dim))
            self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(-1, self.width)
        self.ax.set_ylim(-1, self.height)
        self.ax.grid(False)
        self.ax.set_aspect('equal')

        grid = np.array(self.obstacle_map)

        empty_y, empty_x = np.where(grid == 0)
        self.ax.scatter(empty_x, empty_y, s=350, c='white',
                        marker='s', edgecolors='black')

        water_y, water_x = np.where(grid == 2)
        self.ax.scatter(water_x, water_y, s=350, c='cyan',
                        marker='s', edgecolors='black', label='Water')

        obstacle_y, obstacle_x = np.where(grid == 1)
        self.ax.scatter(obstacle_x, obstacle_y, s=350, c='black',
                        marker='s')

        self.ax.scatter(self.start[0], self.start[1], s=350,
                        c='blue', marker='s', edgecolors='black', label='Start')
        self.ax.scatter(self.goal[0], self.goal[1], s=350,
                        c='lightgreen', marker='s', edgecolors='black', label='Goal')

        # possibly add this as global const, will see later
        algorithm_titles = {'1': 'BFS', '2': 'DFS', '3': 'UCS', '4': 'A*'}
        self.ax.set_title(f"{algorithm_titles.get(self.algorithm, 'Search')} - {self.search_type.title()} Search")

    def search_with_animation(self):
        algorithm_map = {'1': self.algorithm_planner.planning_bfs, '2': self.algorithm_planner.planning_dfs,
                         '3': self.algorithm_planner.planning_ucs, '4': self.algorithm_planner.planning_astar}

        chosen_algorithm = algorithm_map.get(self.algorithm)

        # a way to track performance, need to add onto it later
        tracemalloc.start()

        path, nodes_expanded, visited_nodes = chosen_algorithm(self.start[0], self.start[1],
                                                               self.goal[0], self.goal[1],
                                                               search_type=self.search_type,
                                                               show_animation=self.show_animation)

        # fault check
        if path is None and self.show_animation:
            print("No path found!")
            return None, None, None

        # saves current and peak
        memory = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        if not self.show_animation:
            return path, memory, nodes_expanded

        if visited_nodes:
            for node_id, node in visited_nodes.items():
                if (node.x, node.y) != (self.start[0], self.start[1]) and (node.x, node.y) != (self.goal[0], self.goal[1]):
                    self.ax.scatter(node.x, node.y, s=300, c='grey', marker='s', edgecolors='black', alpha=0.5)
                    plt.pause(pause_time)

        location_x = [coord[0] for coord in path]
        location_y = [coord[1] for coord in path]

        if len(location_x) >= 2:
            for i in range(len(location_x) - 1):
                path_x = (location_x[i], location_x[i + 1])
                path_y = (location_y[i], location_y[i + 1])
                plt.plot(path_x, path_y, c='red', linewidth=4)
                plt.pause(pause_time)

        print(f"Current memory usage: {memory[0] / 1024:.3f} kB")
        print(f"Peak memory usage: {memory[1] / 1024:.3f} kB")

        return path, memory, nodes_expanded


def run_search(search_algorithm, search_type, show_animation=True,
               maze=maze_generate(14, 0.26, 0.4, random_seed=random_seed, start_region_input=1, goal_region_input=4)):
    # Need a lot of visualisation fixes if the maze becomes big, then need a for loop
    # you can change the approximate start/goal position
    # (1 - bottom left; 2 - top left; 3 - bottom right; 4 - top right; 5 - centre, empty - random)

    obstacle_map = maze.get_obstacle_map()

    algorithm_planner = SearchPlanner(1, obstacle_map, motion_model='4n')
    fig_dim = 10  # need a way to make this more dynamic

    animated = AnimatedSearch(obstacle_map, maze.maze_start, maze.maze_goal, algorithm_planner, search_algorithm,
                              {'1': 'tree', '2': 'graph'}.get(search_type),
                              fig_dim, show_animation=show_animation)

    maze.save_maze()

    # another measure to count performance
    start_time = time.time()
    path, memory, nodes_expanded = animated.search_with_animation()
    elapsed_time = time.time() - start_time

    return elapsed_time, path, memory, nodes_expanded, obstacle_map


def display_maze(search_algorithm, search_type, display_plot=True):
    try:
        elapsed_time, path, memory, nodes_expanded, obstacle_map = run_search(search_algorithm, search_type)

        if path:
            print(f"Path length: {len(path)} steps")

            cost = get_cost(path, obstacle_map)

            print(f"Total cost: {cost}")
            print(f"Total Expanded Nodes: {nodes_expanded}")
            print(f"Time: {elapsed_time}s")
        else:
            print("Unable to find a path")

        if display_plot:
            plt.legend()
            plt.show()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


def get_cost(path, obstacle_map=None):
    if obstacle_map is None or len(path) <= 1:
        return len(path) - 1 if len(path) > 0 else 0

    total_cost = 0
    for i in range(0, len(path)):
        coord_x, coord_y = path[i]

        cell_value = obstacle_map[coord_y][coord_x]

        total_cost += 3 if cell_value == 2 else 1

    return total_cost


def results_iterator(mazes, search_types=('1', '2'), algorithms_types=('1', '2', '3', '4')):
    search_results = list()
    try:
        for algorithm in algorithms_types:
            for search in search_types:
                for maze in mazes:
                    elapsed_time, path, memory, nodes_expanded, _ = run_search(algorithm, search, False, maze)
                    path_length = len(path) if path is not None else np.nan
                    search_results.append((algorithm, search, elapsed_time, path_length, nodes_expanded, memory[1],
                                           maze.size, maze.obstacle_density, maze.water_density, maze.start_region, maze.goal_region, random_seed,
                                           maze))

        results_df = pd.DataFrame(search_results,
                                  columns=['Search_Algorithm', 'Search_Type', 'Time', 'Path_Size', 'Num_Nodes_Expanded',
                                           'Peak_Memory_Usage', 'Maze_Size', 'Maze_Density', 'Water_Density', 'Start_Region',
                                           'Goal_Region', 'Random_Seed', 'Maze_Object'])

        results_df['Algorithm_And_Search_Name'] = results_df["Search_Type"].map(search_type_titles) + '_' + results_df[
            "Search_Algorithm"].map(algorithm_titles)
        results_df['Search_Algorithm_Name'] = results_df["Search_Algorithm"].map(algorithm_titles)
        results_df['Search_Type_Name'] = results_df["Search_Type"].map(search_type_titles)

        results_df = results_df[
            ['Algorithm_And_Search_Name', 'Maze_Size', 'Maze_Density', 'Water_Density', 'Path_Size', 'Num_Nodes_Expanded', 'Time',
             'Peak_Memory_Usage', 'Start_Region', 'Goal_Region', 'Search_Type', 'Search_Algorithm', 'Search_Type_Name',
             'Search_Algorithm_Name', 'Random_Seed', 'Maze_Object']]

    except Exception as e:
        print(f"Error: {e}")

    return results_df


def create_maze_list(sizes, densities, water_density, regions, random_seed=random_seed):
    maze_list = []
    for size in sizes:
        for obstacle_density in densities:
            for start_region in regions:
                for end_region in regions:
                    if start_region == end_region:
                        continue
                    maze_list.append(maze_generate(size, obstacle_density, water_density, start_region, end_region, random_seed))

    return maze_list


def graph_results():
    sizes = [10, 15, 20]
    obstacle_density = [0.0, 0.25, 0.5]
    water_density = 0.35
    regions = list(range(1, 6))
    mazes = create_maze_list(sizes, obstacle_density, water_density, regions)

    results_df = results_iterator(mazes)

    results_df.to_csv('results_df.csv', index=False)

