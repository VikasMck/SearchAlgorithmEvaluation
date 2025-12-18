import numpy as np
import pandas as pd
import json
import time
import matplotlib.pyplot as plt
import matplotlib
import tracemalloc
import seaborn as sns
from algorithms import SearchPlanner

from maze_generate import maze_generate,Maze

# Simplest way to run on Macs
matplotlib.use('MacOSX')

show_animation = True
pause_time = 0.2

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
    return [[bool(data[index_x][index_y]) for index_y in range(data.shape[0])]
            for index_x in range(data.shape[1])]


# General class for animating, need to add on to it as currently only shows the final path.
class AnimatedSearch:
    def __init__(self, obstacle_map, start, goal, algorithm_planner, algorithm, search_type, fig_dim, show_animation = True):
        self.obstacle_map = obstacle_map
        self.start = start
        self.goal = goal
        self.algorithm = algorithm
        self.search_type = search_type
        self.algorithm_planner = algorithm_planner
        self.width, self.height = len(obstacle_map), len(obstacle_map[0])
        self.show_animation = show_animation

        if show_animation:
            self.fig, self.ax = plt.subplots(figsize=(fig_dim, fig_dim))
            self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(-1, self.width)
        self.ax.set_ylim(-1, self.height)
        self.ax.grid(True)
        self.ax.set_aspect('equal')

        grid = np.array(self.obstacle_map)

        empty_y, empty_x = np.where(~grid)
        self.ax.scatter(empty_x, empty_y, s=300, c='blue',
                        marker='s', edgecolors='black')

        obstacle_y, obstacle_x = np.where(grid)
        self.ax.scatter(obstacle_x, obstacle_y, s=300, c='gray',
                        marker='s')

        self.ax.scatter(self.start[0], self.start[1], s=400,
                        c='green', marker='s', edgecolors='black', label='Start')
        self.ax.scatter(self.goal[0], self.goal[1], s=400,
                        c='red', marker='s', edgecolors='black', label='Goal')

        # possibly add this as global const, will see later
        algorithm_titles = {'1': 'BFS', '2': 'DFS', '3': 'UCS', '4': 'A*'}
        self.ax.set_title(f"{algorithm_titles.get(self.algorithm, 'Search')} - {self.search_type.title()} Search")

    def search_with_animation(self):
        algorithm_map = {'1': self.algorithm_planner.planning_bfs, '2': self.algorithm_planner.planning_dfs,
                         '3': self.algorithm_planner.planning_ucs, '4': self.algorithm_planner.planning_astar}

        chosen_algorithm = algorithm_map.get(self.algorithm)

        # a way to track performance, need to add onto it later
        tracemalloc.start()

        path = chosen_algorithm(self.start[0], self.start[1],
                                self.goal[0], self.goal[1],
                                search_type=self.search_type)

        # fault check
        if path is None:
            print("No path found!")
            return None

        # saves current and peak
        memory = tracemalloc.get_traced_memory()
        tracemalloc.stop()


        if not self.show_animation:
            return path, memory

        print(f"Peak memory usage: {memory}")

        location_x = [coord[0] for coord in path]
        location_y = [coord[1] for coord in path]

        if len(location_x) >= 2:
            for i in range(len(location_x) - 1):
                path_x = (location_x[i], location_x[i + 1])
                path_y = (location_y[i], location_y[i + 1])
                plt.plot(path_x, path_y, "-y", linewidth=4)
                plt.pause(pause_time)

        return path, memory


def run_search(search_algorithm, search_type, show_animation = True, maze = maze_generate(15, 0.5) ):

    # Need a lot of visualisation fixes if the maze becomes big, then need a for loop
    # you can change the approximate start/goal position
    # (1 - bottom left; 2 - top left; 3 - bottom right; 4 - top right; 5 - centre, empty - random)


    obstacle_map = create_obstacle_map(pd.DataFrame(maze.generated_maze))
    algorithm_planner = SearchPlanner(1, obstacle_map, motion_model='4n')
    fig_dim = 10 # need a way to make this more dynamic

    animated = AnimatedSearch(obstacle_map, maze.maze_start, maze.maze_goal, algorithm_planner, search_algorithm,
                                  {'1': 'tree', '2': 'graph'}.get(search_type),
                                  fig_dim, show_animation=show_animation)

    # another measure to count performance
    start_time = time.time()
    path, memory = animated.search_with_animation()
    elapsed_time = time.time() - start_time

    return elapsed_time, path, memory


def display_maze(search_algorithm, search_type, displayPlot = True):
    try:
        elapsed_time, path, memory = run_search(search_algorithm, search_type)

        if path:
            print(f"Path length: {len(path)} steps")
            print(f"Total cost: {get_cost(path)}")
            print(f"Time: {elapsed_time}s")
        else:
            print("Unable to find a path")

        if displayPlot:
            plt.legend()
            plt.show()

    except Exception as e:
        print(f"Error: {e}")


def get_cost(path):
    return sum([1 for _ in path])


def results_iterator (iterations = 1, search_types = ('2'), algorithms_types = ('1', '2', '3', '4')):
    search_results = list()
    try:
        for algorithm in algorithms_types:
            for search in search_types:
                attempt = 0
                for iteration in range(iterations):
                    attempt += 1
                    elapsed_time, path, memory = run_search(algorithm, search, False)
                    # text = f'{search_type_titles.get(search)}_{algorithm_titles.get(algorithm)}'
                    search_results.append((algorithm,search,attempt,elapsed_time, len(path), memory[1]))

    except Exception as e:
        print(f"Error: {e}")

    return search_results


def graph_results():
    search_results = results_iterator(iterations=3)

    results_df = pd.DataFrame(search_results, columns=['Search_Algorithm','Search_Type','Attempts','Time', 'Path', 'Peak_Memory_Usage'])

    results_df['Algorithm_And_Search_Name'] = results_df["Search_Type"].map(search_type_titles) + '_'+ results_df["Search_Algorithm"].map(algorithm_titles)
    results_df['Search_Algorithm_Name'] = results_df["Search_Algorithm"].map(algorithm_titles)
    results_df['Search_Type_Name'] = results_df["Search_Type"].map(search_type_titles)

    # print(results_df['Algorithm_Name'])
    # print(results_df.describe())

    line_memory = sns.lineplot(y = 'Peak_Memory_Usage', x = 'Attempts', data = results_df, hue = 'Algorithm_And_Search_Name', marker = 'o')
    # plt.ylim(results_df['Time'].min(), results_df['Time'].max())
    plt.legend(title = 'Search Algorithms')
    plt.title('Memory Usage Per Attempt Of Each Search Algorithm')
    plt.ylabel("Peak Memory Usage (Bytes)")
    plt.xticks(results_df['Attempts'].unique())
    plt.show()

    bat_path = sns.barplot(y='Path', x='Search_Algorithm_Name', data=results_df, hue='Search_Type_Name')
    plt.legend(title = 'Search Types')
    plt.title('Comparing Path Lengths Across Search Algorithms')
    plt.ylabel("Path Length")
    plt.xlabel("Search Algorithms")
    plt.yticks(results_df['Path'].unique())
    plt.show()













