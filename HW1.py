import argparse
import os
from typing import List, Tuple
import math
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString


def ensure_ccw(points):
    area = 0
    for i in range(len(points)):
        x1,y1 = points[i]
        x2,y2 = points[(i+1) % len(points)]
        area += x1*y2 - x2*y1
    if area < 0:
        points.reverse()
    return points

def rotate_to_lowest(poly):
    """Rotate polygon so polygon[0] is the lowest vertex."""
    lowest = min(range(len(poly)), key=lambda i: (poly[i][1], poly[i][0]))
    return poly[lowest:] + poly[:lowest]


def edge_angle(p, q):
    ang = math.atan2(q[1]-p[1], q[0]-p[0])
    if ang < 0:
        ang += 2*math.pi
    return ang


def minkowski_sum_convex(P, Q):
    """
    Compute Minkowski sum of convex CCW polygons P and Q.
    Returns list of vertices.
    """

    # rotate so each starts at lowest vertex
    print("P before CCW:", P)
    print("Q before CCW:", Q)
    P = ensure_ccw(P)
    Q = ensure_ccw(Q)
    print("P after CCW:", P)
    print("Q after CCW:", Q)

    P = rotate_to_lowest(P)
    Q = rotate_to_lowest(Q)

    n = len(P)
    m = len(Q)

    # Extend by 2 extra vertices (line 2 in algorithm)
    P_ext = P + [P[0], P[1]]
    Q_ext = Q + [Q[0], Q[1]]

    i = 0
    j = 0
    result = []


    for _ in range(n + m):
        print(f"Step {_}: i={i}, j={j}")
        print(f"n, m", n, m)

        vx, vy = P_ext[i]
        wx, wy = Q_ext[j]
        result.append((vx + wx, vy + wy))

        print(f"i: {i}, j: {j}, added: ({vx + wx}, {vy + wy})")
        angle_P = edge_angle(P_ext[i], P_ext[i+1])
        angle_Q = edge_angle(Q_ext[j], Q_ext[j+1])

        if angle_P < angle_Q:
             if i < n:
                i += 1
        elif angle_P > angle_Q:
             if j < m:
                j += 1
        else:
            print("Equal angles")
            if i < n:
                i += 1
            if j < m:
                j += 1

    return result


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

    mink = minkowski_sum_convex(P, R)

    # Return as Shapely polygon
    return Polygon(mink)


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the C-space obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    raise NotImplementedError()


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