

"""  Module with useful functions used within the package. """

import numpy


def listify(obj):
    """ Returns an object as a list if it isn't already. """

    return obj if isinstance(obj, (list, type(None))) else [obj]


def distance_2d(point_a, point_b):
    """ Calculates euclidean distance between two points in the plane. """

    a = (point_a.x, point_a.y)
    b = (point_b.x, point_b.y)

    return numpy.linalg.norm(a - b)


def distance_3d(point_a, point_b):
    """ Calculates euclidean distance between two points in space. """

    a = (point_a.x, point_a.y, point_a.z)
    b = (point_b.x, point_b.y, point_a.z)

    return numpy.linalg.norm(a - b)
