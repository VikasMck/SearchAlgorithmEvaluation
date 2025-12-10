import random
from collections import deque

# 4n without cost
motion = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def maze_generate(maze_size, maze_density=None):

    generated_maze = []

    # fill everything with 1s
    for _ in range(maze_size):
        generated_maze.append([1] * maze_size)

    # fill the middle with 0s, that makes the outer wall in the simplest way and it is
    # easier to ensure a path exists later on
    for i in range(1, maze_size-1):
        for j in range(1, maze_size-1):
            generated_maze[i][j] = 0

    # this I made so there's randomness in maze vastness, and went down a rabbit hole of learning
    # about percolation theory in 2d mazes where after 0.59 threshold mazes are rare to be possible.
    if maze_density is None:
        maze_type = random.randint(1, 3)
        if maze_type == 1:
            wall_density = random.uniform(0.5, 0.20)
        elif maze_type == 2:
            wall_density = random.uniform(0.20, 0.35)
        else:
            wall_density = random.uniform(0.35, 0.55)
    else:
        wall_density = maze_density
    # headache, similarly to how I filled the inner maze with 0s, it visits each cell and the random.random
    # generated 0.0 to 1.0 value and then it gets compared with the wall_density, so the bigger the number is
    # the more likely to create a wall, hence the ~0.59 threshold in percolation which is a
    # super long theory of how it works.
    for i in range(1, maze_size-1):
        for j in range(1, maze_size-1):
            if random.random() < wall_density:
                generated_maze[i][j] = 1

    # no matter what I changed, this approach is the only one that gives ok results, otherwise the mazes are too easy
    # or just flat out horrible
    def random_maze_region(region):
        if region == 1:
            return random.randint(1, 3), random.randint(1, 3)
        elif region == 2:
            return random.randint(1, 3), random.randint(maze_size - 4, maze_size - 2)
        elif region == 3:
            return random.randint(maze_size - 4, maze_size - 2), random.randint(1, 3)
        elif region == 4:
            return random.randint(maze_size - 4, maze_size - 2), random.randint(maze_size - 4, maze_size - 2)
        else:
            maze_center = maze_size // 2
            return random.randint(maze_center - 1, maze_center + 1), random.randint(maze_center - 1, maze_center + 1)

    start_region = random.randint(1, 5)
    goal_region = random.randint(1, 5)
    while goal_region == start_region:
        goal_region = random.randint(1, 5)

    maze_start = random_maze_region(start_region)
    maze_goal = random_maze_region(goal_region)

    generated_maze[maze_start[0]][maze_start[1]] = 0
    generated_maze[maze_goal[0]][maze_goal[1]] = 0

    # to avoid unsolvable mazes I just reuse the bfs, just take away the cost. Not sure why I
    # have cost in actual BFs in algorithms
    def bfs_check_path_exists():
        class Node:
            def __init__(self, x, y, parent_index):
                self.x = x
                self.y = y
                self.parent_index = parent_index

        min_x = 0
        min_y = 0
        x_width = maze_size

        def calculate_grid_index(node):
            return (node.y - min_y) * x_width + (node.x - min_x)

        start_node = Node(maze_start[1], maze_start[0], -1)
        goal_node = Node(maze_goal[1], maze_goal[0], -1)

        queue = deque([start_node])
        visited_nodes = {}
        closed_set = set()

        while queue:
            current = queue.popleft()
            current_id = calculate_grid_index(current)

            visited_nodes[current_id] = current

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                path = []
                node = current
                while node.parent_index != -1:
                    path.append((node.y, node.x))
                    node = visited_nodes[node.parent_index]
                path.append((maze_start[0], maze_start[1]))
                path.reverse()
                return path

            if current_id in closed_set:
                continue
            closed_set.add(current_id)

            for change_x, change_y in motion:
                next_x, next_y = current.x + change_x, current.y + change_y

                if 0 <= next_x < maze_size and 0 <= next_y < maze_size:
                    if generated_maze[next_y][next_x] == 0:
                        neighbour = Node(next_x, next_y, current_id)
                        neighbour_id = calculate_grid_index(neighbour)
                        if neighbour_id not in closed_set:
                            queue.append(neighbour)

        return None

    # 10 tries to get a path
    for attempt in range(10):
        path = bfs_check_path_exists()
        # if path is already found just exit, otherwise we carving it
        if path:
            break
        # so now we will try to clear up a path
        y_coord, x_coord = maze_start
        while (y_coord, x_coord) != maze_goal:
            generated_maze[y_coord][x_coord] = 0
            for change_y, change_x in motion:
                neighbour_y, neighbour_x = y_coord + change_y, x_coord + change_x
                if 1 <= neighbour_y < maze_size-1 and 1 <= neighbour_x < maze_size-1:
                    # adding randomness to clearing the maze, so it is not just straight
                    if random.random() < 0.5:
                        generated_maze[neighbour_y][neighbour_x] = 0
                        if y_coord < maze_goal[0]:
                            y_coord += 1
                        elif y_coord > maze_goal[0]:
                            y_coord -= 1
                        elif x_coord < maze_goal[1]:
                            x_coord += 1
                        elif x_coord > maze_goal[1]:
                            x_coord -= 1

        generated_maze[maze_goal[0]][maze_goal[1]] = 0

    return generated_maze, maze_start, maze_goal


def print_maze(generated_maze, maze_start, maze_goal):
    for row in generated_maze:
        print(', '.join(map(str, row)))
    print(f"\nStart: {maze_start}")
    print(f"Goal: {maze_goal}")


# for i in range(10, 25, 5):
#     for j in range(1, 5):
#         generated_maze, maze_start, maze_goal = maze_generate(i, j * 0.1)
#         print_maze(generated_maze, maze_start, maze_goal)

