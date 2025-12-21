# Week 5 - Search - Tree search VS Graph search
# Depth First Search (DFS)
import matplotlib
# Please check line 165 and line 179 to try out tree search and graph search
# Tips: You only need either tree search or graph search
# Please feel free to implement everything from scratch if you can
# It is absolutely fine if you cannot understand everything. We have four weeks to understand the codes

import matplotlib.pyplot as plt
matplotlib.use('MacOSX')
import math
import numpy as np
import pandas as pd
import json
import pysnooper


# For visualisation purposes!
show_animation = True
pause_time = 0.0000000000000000000000000000000000001


class DepthFirstSearchPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for Depth-First planning

        This is both used for TREE search and GRAPH search

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)


        self.motion = self.get_motion_model_4n()



    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            """
            This is used to manage nodes (or locations), anything can be abstracted as a node
            """
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost  # cost will be used in the future
            self.parent_index = parent_index  # index is the grid location
            self.parent = parent              # this is a node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)
        def __eq__(self, other):
            return self.x == other.x and self.y == other.y

    def planning(self, sx, sy, gx, gy):
        """
        Depth First search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                           self.calc_xy_index(sy, self.min_y), 0.0, -1, None)

        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                          self.calc_xy_index(gy, self.min_y), 0.0, -1, None)

        plt.scatter(start_node.x, start_node.y, s=400, c='red', marker='s')
        plt.scatter(goal_node.x, goal_node.y, s=400, c='green', marker='s')                           

        open_set, closed_set = dict(), dict()

        open_set[self.calc_grid_index(start_node)] = start_node

        closed_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            
            # DFS
            # current = open_set.pop(list(open_set.keys())[-1])
            # turns it into a BFS
            current = open_set.pop(list(open_set.keys())[0])
            c_id = self.calc_grid_index(current)

            # Visualisation
            if show_animation:  

                # plt.scatter(self.calc_grid_position(current.x, self.min_x),
                #             self.calc_grid_position(current.y, self.min_y),
                #             s=300, c='yellow', marker='s')   

                plt.scatter(current.x,
                            current.y,
                            s=300, c='yellow', marker='s')   

                #Press the ESC key to terminate the visualisation.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                            lambda event:
                                            [exit(0) if event.key == 'escape'
                                            else None])

                # Pause some time to help with observation
                plt.pause(pause_time)

                if len(closed_set)>=2:
                    locx, locy = self.calc_final_path(current, closed_set)
                    for i in range(len(locx)-1):
                        px = (locx[i], locx[i+1])
                        py = (locy[i], locy[i+1])
                        plt.plot(px, py, "-m", linewidth=4)
                        plt.pause(pause_time)

            # If Found The Goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")

                plt.scatter(current.x,
                            current.y,
                            s=300, c='yellow', marker='s') 
                plt.pause(pause_time)
                
                locx, locy = self.calc_final_path(current, closed_set)
                if len(locx)>=2:
                    for i in range(len(locx)-1):
                        px = (locx[i], locx[i+1])
                        py = (locy[i], locy[i+1])
                        plt.plot(px, py, "-m", linewidth=4)
                        plt.pause(pause_time)


                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                print("cost :" ,goal_node.cost)
                break

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                current.y + self.motion[i][1],
                                current.cost + self.motion[i][2], c_id, None)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    plt.scatter(node.x,
                                node.y,
                                s=100, c='black', marker='s')
                    plt.pause(pause_time)                      
                    continue

            ######################Graph Search ##########################

                # if n_id not in closed_set:
                #     open_set[n_id] = node
                #     closed_set[n_id] = node
                #     node.parent = current
                #
                #
                #     plt.scatter(node.x,
                #                 node.y,
                #                 s=100, c='b', marker='s')
                #     plt.pause(pause_time)


            ######################Tree Search ##########################



                if self.is_tree_looping(current, node):
                    continue

                open_set[n_id] = node
                closed_set[n_id] = node
                node.parent = current

                    
                plt.scatter(node.x,
                            node.y,
                            s=100, c='b', marker='s')
                plt.pause(pause_time)

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    # @pysnooper.snoop()
    def calc_final_path(self, goal_node, closedset):
        # generate final path

        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
            
        n = closedset[goal_node.parent_index]  # colsedset is a dict, use index to retrieve node

        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            n = n.parent

        return rx, ry

    ##get coordinates from index in grid system
    def calc_grid_position(self, index, minp):
        pos = index * self.reso + minp
        return pos

    ##get index from coordinates based on x or y 
    ## use in determining start node
    ## input 'position' in [m]
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    ##get index from coordinates based on x and y
    ## input node.x and node.y are grid coordinates
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.xwidth + (node.x - self.min_x)

    ##verify node if it safe or not when exploring 
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

############################### modified #####################################
        self.xwidth = round((self.maxx - self.min_x) / self.reso) + 1
        self.ywidth = round((self.maxy - self.min_y) / self.reso) + 1
############################### modified #####################################        

        # obstacle map generation
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]

        
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.min_y)             
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
 
 ############################### modified #####################################                   
                    if d < self.rr:                # Be careful with this line, you can change '<' to '<=' to see the differrences of new_array defined laterr!
                        self.obmap[ix][iy] = True
                        # break
                    else:
                        continue
                   
        new_array = np.zeros(np.shape(self.obmap))
        for i in range(new_array.shape[0]):
            for j in range(new_array.shape[1]): 
                if self.obmap[i][j]==True: 
                    new_array[i][j]=1        
        print(new_array.T)
############################### modified #####################################

    @staticmethod
    def get_motion_model_4n():
        """
        Get all possible 4-connectivity movements.
        :return: list of movements with cost [[dx, dy, cost]]
        """
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1]]
        return motion

    def is_tree_looping(self, parent, node):
        while parent.parent is not None:
            if parent.parent == node:
                return True
            parent = parent.parent
        return False




def main():
    # Obstacle and free grid positions
    ox, oy = [], [] #obstacle
    fx, fy = [], [] #free  


    ##get config setting from json file
    config_file = '/Users/mathewhiggins/Documents/University/Masters Year/Artificial Intelligence Principles - 6G7V0011_2526_9F/Week 5 /Lab_Week_5-20251029/MapConfig/config16x22.json'

    # start and goal position
    with open(config_file) as config_env:
        param = json.load(config_env)

    sx = param['sx']  # [m]
    sy = param['sy']   # [m]
    gx = param['gx']   # [m]
    gy = param['gy']   # [m]
    grid_size = param['grid_size']  # [m]
    robot_radius = param['robot_radius']   # [m]
    map_xlsx = param['map_xlsx']
    fig_dim = param['fig_dim']

    # padas is used to read map in .xlsx file
    gmap = pd.read_excel(map_xlsx,header=None)
    data = gmap.to_numpy()

    # Up side down: origin of image is up left corner, while origin of figure is bottom left corner
    data = data[::-1]   


    """
    ^ y
    |
    |[['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', 'o', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+']]
    |--------------------------------------------------> x
                Cartesian coordinate system

   
    |--------------------------------------------------> x
    |[['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', 'o', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+'],
    | ['+', '+', '+', '+', '+', '+', '+']]
    |
    V y

    How matrix stored in Python, y is the first dimension, i.e., row, and x is the second dimension, i.e., column

    
    For instance, the coordinate of 'o'  is [iy ix], in this case [3, 4]

    """


    # Generate obstacles
    for iy in range(data.shape[0]):  # the first dimension is y coordinate
        for ix in range(data.shape[1]):  # the second dimension is x coordinate
            if data[iy, ix] == 1:
                ox.append(ix)            # keep consist with Cartesian, easier for human to understand
                oy.append(iy)            # This also keeps the calc_obstacle_map(self, ox, oy) align with Cartesian coordinate system
            else:
                fx.append(ix)
                fy.append(iy)



    if show_animation:  
        plt.figure(figsize=(fig_dim ,fig_dim))
        plt.xlim(-1, data.shape[1])
        plt.ylim(-1, data.shape[0])
        plt.xticks(np.arange(0,data.shape[1],1))
        plt.yticks(np.arange(0,data.shape[0],1))
        plt.grid(True)
        plt.scatter(ox, oy, s=300, c='gray', marker='s')
        plt.scatter(fx, fy, s=300, c='cyan', marker='s')  

    ###always use 4n motion model
    dfs = DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = dfs.planning(sx, sy, gx, gy)

    ##reverse because the path found by the algorithm is organised from goal to start
    rx.reverse()  
    ry.reverse()


    # Convert list rx and ry into int
    new_rx = map(int, rx)
    new_ry = map(int, ry)


    path_found = [list(a) for a in zip(new_rx, new_ry)]

    ##output path 
    print("Path from start to goal", path_found)


    if show_animation:  
        for i in range(len(rx)-1):
            px = (rx[i], rx[i+1])
            py = (ry[i], ry[i+1])
            plt.plot(px, py, "-k")
            plt.pause(0.1)
        plt.show()   

if __name__=='__main__':
    main()