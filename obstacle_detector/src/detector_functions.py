"""Provides functions for use in the obstacle detection node."""
import ctypes
from math import cos, sin, sqrt, pi

import networkx as nx

# Largest distance between directly connected points.
# Further points may still be grouped if a path exists between them.
GROUPING_DISTANCE = 1
# Minimum number of points needed to be a group.
MIN_GROUP_SIZE = 5
START_ANGLE, END_ANGLE = -pi/4, pi/4

DETECTOR_LIB = ctypes.CDLL("detector_node_lib.so")
DETECTOR_LIB.get_edges.argtypes = [ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.c_int, ctypes.c_double,
                                   ctypes.POINTER(ctypes.c_int)]

def closest_point(points):
    if not points:
        return float('inf')
    return min(_grouping_metric(p,(0,0)) for p in points)


def clean_scan(angle_min, angle_max, angle_inc,
               rmin, rmax, ranges, step=1):
    """
    Return laser scan in in form [(rho,phi),...]
    Throws out any ranges too close/far.

    Args:
    angle_min -- Minimum angle of scan(seems to be -pi/2 for Hokuyo).
    angle_inc -- Change in angle between ranges(seems to be pi/720 for Hokuyo).
    rmin -- Minimum range(ranges less than this should be discarded).
    rmax -- Maximum range(ranges greater than this should be discarded).
    ranges -- Iterable of numbers representing ranges.
    step -- Skips step angles each time. Defaults to one(no skips).
    """
    start = int(round(abs((angle_min-START_ANGLE)/angle_inc)))
    end = len(ranges) - int(round(abs((angle_max-END_ANGLE)/angle_inc)))
    return [(ranges[i], angle_min + angle_inc*i) for i in range(start,end,step) if rmin < ranges[i] < rmax]


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


def _get_edges(points):
    length = int((len(points)*(len(points)-1))/2)
    x,y = zip(*points)

    xin = (ctypes.c_double * len(points))(*x)
    yin = (ctypes.c_double * len(points))(*y)

    x1out = (ctypes.c_double * length)()
    y1out = (ctypes.c_double * length)()
    x2out = (ctypes.c_double * length)()
    y2out = (ctypes.c_double * length)()

    INTP = ctypes.POINTER(ctypes.c_int)
    ptr_num = ctypes.c_int(42)
    ptr_addr = ctypes.addressof(ptr_num)
    ptr = ctypes.cast(ptr_addr, INTP)

    DETECTOR_LIB.get_edges(xin, yin, x1out, y1out, x2out, y2out,
                           ctypes.c_int(len(points)),
                           ctypes.c_double(GROUPING_DISTANCE), ptr)
    num_edges = ptr[0]
    edges = list(zip(((x1out[i],y1out[i]) for i in range(num_edges)),
                     ((x2out[i],y2out[i]) for i in range(num_edges))))
    return edges
    

def group_points(points):
    """
    Return list of groups of points in form [[(x,y),...],[(x,y),...],...].

    Points are grouped by connecting all points within GROUPING_DISTANCE
    of each other. The groups are labeled using Networkx's connected component
    function on the edge list.

    Args:
    points -- iterable of 2D points in form [(x,y),...]
    """
    g = nx.Graph()
    g.add_edges_from(_get_edges(points))
    groups = nx.connected_components(g)
    groups = [group for group in groups if len(group) >= MIN_GROUP_SIZE]
    return groups
