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
    return (ay - by) * (bx - px) - (ax - bx) * (by - py)

def intersect(a,b,c,d):
    # returns true if the line segments ab and cd intersect.
    # the lines intersect iff both pairs are on opposite sides of both lines.
    # will always return false if any 3 points are on the same line.
    side1 = side(a, b, c)
    side2 = side(a, b, d)
    side3 = side(c, d, a)
    side4 = side(c, d, b)
    return (side1 * side2 < 0) and (side3 * side4 < 0)  # both pairs are on opposite sides of the other line

def is_between(v,w,c) -> bool:
    # return true if c is on the line segment vw
    return side(v, w, c) == 0 and (
            (v[0] < c[0]) == (c[0] < w[0])) and (
            (v[1] < c[1]) == (c[1] < w[1]))

def cross_poly_edge(v, w, n, j, coords) -> bool:
    # edge case where obstacle is in poly1 or poly2 (the line of visibility is "inside" the polygon,
    #   so it intersects another edge of the polygon. but it comes out exactly on another vertex, so it
    #   is not discovered in our other checks because it is technically collinear):
    for k in range(n - 1):
        c = coords[k]
        if is_between(v, w, c) and not (k == j - 1 or k == j or k == j + 1):
            return True
    return False

def check_if_blocks(v, w, obstacle: Polygon) -> bool:
    # check if line vw intersects any of the obstacles
    coords = obstacle.exterior.coords
    n = len(coords)
    for k in range(n - 1):  # for edge in obstacle:
        c = coords[k]
        d = coords[k + 1]
        if intersect(v, w, c, d):  # vw intersects cd
            return True
    return False

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the C-space obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vertices: List[Tuple[float, float]] = [source, dest]
    edges: List[LineString] = []
    # initialize vertices
    for poly in obstacles:
        for v in poly.exterior.coords:
            vertices.append(v)
    # iterate on all vertex pairs. add an edge if visible.
    for i, poly1 in enumerate(obstacles):
        for poly2 in obstacles[i+1:]:  # start from index after i
            for j, v in enumerate(poly1.exterior.coords):  # for every vertex in poly1
                if j > 0:
                    edges.append(LineString((poly1.exterior.coords[j-1], v)))  # add paths across the edges of the polygon
                for l, w in enumerate(poly2.exterior.coords):  # for every vertex in poly2
                    visible = True  # visible unless blocked
                    # iterate on all line segments of all polygons to see if they block visibility
                    for obstacle in obstacles:  # may also collide with vertex in the same poly so should check all polys
                        if check_if_blocks(v, w, obstacle):
                            visible = False
                            break
                        # edge case: check if line goes through vertex of poly1 or poly2
                        coords = obstacle.exterior.coords
                        n = len(coords)
                        if (obstacle == poly1 and cross_poly_edge(v, w, n, j, coords)) or (
                            obstacle == poly2 and cross_poly_edge(v, w, n, l, coords)):
                            visible = False
                            break
                    if visible:
                        edges.append(LineString((v, w)))
    # add edges for source and dest
    if source and dest:
        for poly in obstacles:
            for v in poly.exterior.coords:
                for w in [source, dest]:
                    visible = True
                    for obstacle in obstacles:
                        if check_if_blocks(v, w, obstacle):
                            visible = False
                            break
                    if visible:
                        edges.append(LineString((v, w)))
        # finally check source to dest visibility
        visible_source_dest = True
        for obstacle in obstacles:
            if check_if_blocks(source, dest, obstacle):
                visible_source_dest = False
                break
        if visible_source_dest:
            edges.append(LineString((source, dest)))
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