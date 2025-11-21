import argparse
import os
from typing import List, Tuple
import math
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
from shapely.geometry import MultiPoint



def minkowski_sum_convex_hull(P, R):
    sums = []
    for (px, py) in P:
        for (qx, qy) in R:
            sums.append((px + qx, py + qy))

    hull = MultiPoint(sums).convex_hull

    # Convert hull (Polygon) to list of coordinates
    return list(hull.exterior.coords)[:-1]

def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
 # Extract shape vertices (remove duplicate last vertex)
    P = list(original_shape.exterior.coords)[:-1]

    # Robot  R centered at origin
    R = [
        (-r, 0),    
        (0, -r),    
        (r,  0),     
        (0,  r)      
    ]

    mink = minkowski_sum_convex_hull(P, R)

    # Return as Shapely polygon
    return Polygon(mink)


def side(a,b,p):
    # calculates which side of the line ab p is on. 0 if collinear, positive and negative depends on which side of the line.
    (ax, ay) = a
    (bx, by) = b
    (px, py) = p
    return (ay-by)*(bx-px) - (ax-bx)*(by-py)

def intersect(a,b,c,d):
    # returns true if the line segments ab and cd intersect.
    # the lines intersect iff both pairs are on opposite sides of both lines.
    # will always return false if any 3 points are on the same line.
    # print all points
    # print(a,b,c,d)
    side1 = side(a,b,c)
    side2 = side(a,b,d)
    side3 = side(c,d,a)
    side4 = side(c,d,b)
    # print ((side1*side2 < 0) and (side3*side4 < 0))
    return (side1*side2 < 0) and (side3*side4 < 0) # both pairs are on opposite sides of the other line

def is_between(v,w,c):
    # return true if c is on the line segment ab
    return side(v, w, c) == 0 and (
            (v[0] < c[0]) == (c[0] < w[0])) and (
            (v[1] < c[1]) == (c[1] < w[1]))


def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the C-space obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    # gameplan: for all lines:
    #   for all poly in polygons:
    #       check if all vertices in poly are on the same side. if they are, then that poly does not intersect the line.
    vertices = []
    edges = []
    # initialize vertices
    for poly in obstacles:
        for v in poly.exterior.coords:
            vertices.append(v)
    # iterate on all vertex pairs. add an edge if visible.
    for i, poly1 in enumerate(obstacles):
        for poly2 in obstacles[i+1:]: # start from index after i
            for j, v in enumerate(poly1.exterior.coords):
                if j > 0:
                    edges.append(LineString((poly1.exterior.coords[j-1], v)))
                for l, w in enumerate(poly2.exterior.coords):
                    visible = True
                    for obstacle in obstacles: # may also collide with same poly so should check all polys
                        # for line in poly:
                        coords = obstacle.exterior.coords
                        n = len(coords)
                        for i in range(n - 1):
                            c = coords[i]
                            d = coords[i + 1]
                            if intersect(v, w, c, d):
                                visible = False
                                break
                        # edge case where obstacle is poly1 or poly2:
                        # if third point is on the line segment (side == 0 and between x and y coords of the pair):
                        #   check that this or previous or next point is v or w (to avoid crossing the polygon in the middle)
                        if obstacle == poly1:
                            for k in range(n - 1):
                                c = coords[k]
                                if is_between(v, w, c) and not (k == j-1 or k == j or k == j+1):
                                    visible = False
                                    break
                        if obstacle == poly2:
                            for k in range(n - 1):
                                c = coords[k]
                                if is_between(v, w, c) and not (k == l - 1 or k == l or k == l + 1):
                                    visible = False
                                    break
                    if visible:
                        edges.append(LineString((v, w)))
    print("len edges = " + str(len(edges)))
    return edges


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = None, None


    # step 4: Animate the shortest path
    if shortest_path:
        plotter_anim = Plotter()
        plotter_anim.add_obstacles(workspace_obstacles)
        plotter_anim.add_c_space_obstacles(c_space_obstacles)
        plotter_anim.add_visibility_graph(lines)
        plotter_anim.add_robot(source, dist)
        plotter_anim.add_robot(dest, dist)
        plotter_anim.add_shorterst_path(shortest_path)
        # plotter_anim.show_graph()
        plotter_anim.animate_path(shortest_path, dist)