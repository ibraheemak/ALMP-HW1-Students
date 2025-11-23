import argparse
import os
from typing import List, Tuple
import math
from Plotter import Plotter
import shapely
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


def line_is_not_blocked(line: LineString, obstacles: List[Polygon]) -> bool:
    # check if line intersects any of the obstacles
    for obstacle in obstacles:  # may also collide with vertex in the same poly so should check all polys
        if shapely.intersects(obstacle, line) and not shapely.touches(obstacle, line):
            return False
    return True

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the C-space obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vertices: List[Tuple[float, float]] = []
    if source and dest:
        if source not in vertices:
            vertices.append(source)
        if dest not in vertices:
            vertices.append(dest)
    edges: List[LineString] = []
    # initialize vertices
    for poly in obstacles:
        for v in poly.exterior.coords:
            vertices.append(v)
    # add edges between all pairs of vertices if visible
    for i, v in enumerate(vertices):
        for w in vertices[i+1:]:
            line = LineString((v, w))
            if line_is_not_blocked(line, obstacles):
                edges.append(line)
    return edges


def get_shortest_path(obstacles: List[Polygon], visibility_graph: List[LineString], source: Tuple[float, float], dest: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
    """
    Get the shortest path from source to dest using Djikstra's algorithm
    :return: A tuple of the list of points in the shortest path, and the cost of the path
    """
    # build list of edges with distances
    edges = {}
    for line in visibility_graph:
        p1 = line.coords[0]
        p2 = line.coords[1]
        dist = math.dist(p1, p2)
        if p1 not in edges:
            edges[p1] = []
        if p2 not in edges:
            edges[p2] = []
        edges[p1].append((p2, dist))
        edges[p2].append((p1, dist))

    # Dijkstra's algorithm. copied from wikipedia pseudocode
    dist = {}
    prev = {}
    Q = set()  # priority queue
    for v in obstacles:
        for point in v.exterior.coords:
            dist[point] = math.inf
            prev[point] = None
            if point != source:
                Q.add(point)
    # manually add source and dest
    dist[source] = 0
    prev[source] = None
    dist[dest] = math.inf
    prev[dest] = None
    Q.add(source)
    while Q:
        u = min(Q, key=lambda vertex: dist[vertex])  # vertex in Q with smallest dist
        Q.remove(u)
        for (v, length) in edges.get(u, []):
            alt = dist[u] + length
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    # reconstruct path
    s = []
    u = dest
    # if us i in prev or u is source
    if (u in prev and prev[u]) or u == source:
        while u:
            s.insert(0, u)
            u = prev[u]
    return s, dist[dest]


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


def run_test(test: dict):
    """
    Run a test case from a test dictionary.
    :param test: Dictionary with keys:
        - 'robot': dict with 'center' (list) and 'd' (float)
        - 'query': list [x, y] for destination
        - 'obstacles': list of polygons (each polygon is list of [x, y] points)
    :return: Dictionary with 'path' (list of points), 'cost' (float), and visualization data
    """
    # Extract test data
    robot_center = test['robot']['center']
    dist = test['robot']['d']
    source = tuple(robot_center)
    dest = tuple(test['query'])
    obstacles_data = test['obstacles']
    
    # Convert obstacles to Shapely polygons
    workspace_obstacles = [Polygon(obs) for obs in obstacles_data]
    
    # Step 1: Compute C-space obstacles
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    
    # Step 2: Build visibility graph with source and dest
    lines = get_visibility_graph(c_space_obstacles, source, dest)
    
    # Step 3: Find shortest path
    shortest_path, cost = get_shortest_path(c_space_obstacles, lines, source, dest)
    
    return {
        'path': shortest_path,
        'cost': cost,
        'workspace_obstacles': workspace_obstacles,
        'c_space_obstacles': c_space_obstacles,
        'visibility_graph': lines,
        'source': source,
        'dest': dest,
        'dist': dist
    }


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
    shortest_path, cost = get_shortest_path(c_space_obstacles, lines, source, dest)


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