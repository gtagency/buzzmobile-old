"""Provides functions for use in the obstacle detection node."""

from math import cos, sin, sqrt
from itertools import product

# Largest distance between directly connected points.
# Further points may still be grouped if a path exists between them.
GROUPING_DISTANCE = 1


def polar2cart(coords):
    """
    Convert list of polar coordinates to cartesian.

    Args:
    coords -- iterable of polar points in form [(rho,phi),...]
    """
    return [(rho*cos(phi), rho*sin(phi)) for rho,phi in coords]


def _ccw(p1, p2, p3):
    """
    Return direction of vector P1P2 X P1P3.

    Helper function for graham_scan.

    Args:
    p1,p2,p3 -- 2D points represented as tuples (x,y)
    """
    return (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])


def graham_scan(points):
    """
    Return convex hull.

    Args:
    points -- iterable of 2D points in form [(x,y),...]
    """
    upper_hull = []
    lower_hull = []
    points = sorted(points)
    for p in points:
        while len(upper_hull) > 1 and _ccw(upper_hull[-2], upper_hull[-1], p) <= 0:
            upper_hull.pop()
        while len(lower_hull) > 1 and _ccw(lower_hull[-2], lower_hull[-1], p) >= 0:
            lower_hull.pop()
        upper_hull.append(p)
        lower_hull.append(p)
    return upper_hull + lower_hull[-2:0:-1]


def _grouping_metric(p1, p2):
    """
    Return distance between two points.

    Calculates distance using Euclidean metric.
    
    Args:
    p1, p2 -- 2D points represented as tuples (x,y)
    """
    return sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

#TODO find a better way or implement this to be faster.
def get_edges(points):
    """
    Return list of point pairs whose distance is within the grouping threshold.

    Args:
    points -- iterable of 2D points in form [(x,y),...]
    """
    d = _grouping_metric
    pairs = product(points, repeat=2)
    edges = [(a,b) for a,b in pairs if d(a,b) <= GROUPING_DISTANCE]
    return edges
