from typing import List, Tuple

import matplotlib.lines as lines
from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon as plotpol
from shapely.geometry.polygon import Polygon, LineString
import matplotlib.animation as animation


class Plotter:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()

    def add_obstacles(self, obstacles: List[Polygon]):
        for obstacle in obstacles:
            self.ax.add_patch(plt.Polygon(obstacle.exterior.coords, color='b'))

    def add_robot(self, point, distance_to_vertex):
        x, y = point
        self.distance_to_vertex = distance_to_vertex
        target = Polygon([(x - distance_to_vertex, y), (x, y + distance_to_vertex), (x + distance_to_vertex, y), (x, y - distance_to_vertex)])
        self.ax.add_patch(plt.Polygon(target.exterior.coords, color='g'))

    def add_c_space_obstacles(self, obstacles: List[Polygon]):
        for obstacle in obstacles:
            self.ax.add_patch(plt.Polygon(obstacle.exterior.coords, color='r', alpha=0.5))

    def add_visibility_graph(self, edges: List[LineString]):
        for edge in edges:
            plt.plot(list(edge.xy[0]), list(edge.xy[1]), color='black', linestyle='dashed')

    def add_shorterst_path(self, edges):
        if len(edges) > 0:
            current_vertex = edges[0]
            for edge in edges[1:]:
                plt.plot([current_vertex[0], edge[0]], [current_vertex[1], edge[1]], color='yellow', linewidth=5,
                        alpha=0.4)
                self.add_robot(edge, self.distance_to_vertex)
                current_vertex = edge

    def show_graph(self):
        plt.autoscale()
        plt.show()

    def animate_path(self, path, distance_to_vertex):
        """
        Animates the robot's movement along the shortest path.
        """
        # Set up the static parts of the plot
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.autoscale()

        # The robot patch
        robot_patch = plt.Polygon([(0,0)], color='g', animated=True)
        self.ax.add_patch(robot_patch)

        # Interpolate the path for smoother animation
        interpolated_path = []
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]
            line = LineString([p1, p2])
            length = line.length
            # Add 10 intermediate points for every unit of distance
            num_points = int(length * 10)
            if num_points > 0:
                for j in range(num_points + 1):
                    point = line.interpolate(float(j) / num_points, normalized=True)
                    interpolated_path.append((point.x, point.y))
            else:
                interpolated_path.append(p1)
        if len(path) > 0:
            interpolated_path.append(path[-1])


        def update(frame):
            """
            Update function for the animation.
            """
            x, y = interpolated_path[frame]
            robot_vertices = [(x - distance_to_vertex, y), (x, y + distance_to_vertex), (x + distance_to_vertex, y), (x, y - distance_to_vertex)]
            robot_patch.set_xy(robot_vertices)
            return robot_patch,

        ani = animation.FuncAnimation(self.fig, update, frames=len(interpolated_path), blit=True, repeat=True, interval=20)
        plt.show()