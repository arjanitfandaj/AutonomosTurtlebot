

'''

Author: Arjanit Fandaj(@arjanitfandaj)

All right reserved.

Contact: Arjanitfandaj@gmail.com





'''


from os import EX_CANTCREAT
from threading import main_thread
import matplotlib.pyplot as plt
import math
from datetime import datetime
import csv
show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        
        c = self.x_width
        e = self.y_width
        
        try:
        # the generated map which we do it on the main part this part only calculates the position where the 
            # self.obstacle_map = [[False for _ in range(e)] for _ in range(c)]
            self.obstacle_map = [[False for _ in range(e)] for _ in range(c)]
            for ix in range(c):
                x = self.calc_position(ix, int(self.min_x))
                for iy in range(e):
                    y = self.calc_position(iy, int(self.min_y))
                    for iox, ioy in zip(ox, oy):
                        d = math.hypot(iox - x, ioy - y)
                        if d <= self.robot_radius:
                            self.obstacle_map[ix][iy] = True
                            break
        except Exception as t:
            print(str(t))

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

class Main():
    
    def main(self):
        print(__file__ + " start!!")
        c = datetime.now().second

        # start and goal position which we want on meters eitherway, because if your write 0.7 and after converting will take 0 anyway.
        sx = 0  # [m]
        sy = 0  # [m]
        gx = float(input('Give goal x:'))
        gy = float(input('Give goal y:'))
        # gx = 5  # [m]
        # gy = 0 # [m]
        grid_size = 1  # [m]
        robot_radius = 0.4 # [m]

        # set obstacle positions
        ox, oy = [], []

        for i in range(-10, 10):
            ox.append(i)
            oy.append(-5.0)
        for i in range(-5, 5):
            ox.append(10.0)
            oy.append(i)
        for i in range(-10, 11):
            ox.append(i)
            oy.append(5.0)
        for i in range(-5, 5):
            ox.append(-10.0)
            oy.append(i)


        #obstacle

        for i in range(1,3):
            ox.append(i)
            oy.append(1.024)
        for i in range(1,3):
            ox.append(i)
            oy.append(0.474)
        
        for i in range(1,3):
            ox.append(i)
            oy.append(-0.92)
        for i in range(1,3):
            ox.append(i)
            oy.append(-1.42)
        
        for i in range(3,5):
            ox.append(i)
            oy.append(0.59)
        for i in range(3,5):
            ox.append(i)
            oy.append(0.21)
        
        for i in range(3,5):
            ox.append(i)
            oy.append(-1.1)
        
        for i in range(3,5):
            ox.append(i)
            oy.append(-1.81)



        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k",marker='.',markersize=10)
            # plt.plot(ax,ay,linestyle='dashed',markersize=20)
            # plt.subplots(figsize=(18,10))
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            # plt.plot(3,1)

        dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(sx, sy, gx, gy)
        print('Koordinatat qe duhet te leviz roboti:')
        
        # for i in range(len(c)):
        #     print(c[i],':',d[i])
        # return c,d
        filename = 'x_y.txt'
        
        f = open(filename,'w')

        writer = csv.writer(f)
        writer.writerow(rx)
        writer.writerow(ry)
        f.close()

                    
            # for i in ry:
            #     f.write(str(i))
        if show_animation:  # pragma: no cover
            
            plt.plot(rx, ry, "-r")
            
            plt.pause(0.01)
            p = datetime.now().second
            plt.title('Dijkstra')
            # print('koha e zgjatur per kerkimin e rruges',p-c)
            plt.show()
            # plt.waitforbuttonpress(0)
            # plt.close()
            
        else:
            pass



        




if __name__ == '__main__':
    try:
        main = Main()
        print(main.main())
    except Exception as e:
        print(str(e))
    
