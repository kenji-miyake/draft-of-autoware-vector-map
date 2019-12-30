import math

from shapely.geometry import Point


def normalize_radian(rad: float):
    """Return the normalized azimuth

    Arguments:
        azimuth {float} -- azimuth

    >>> round(normalize_radian(3.14), 6)
    3.14
    >>> round(normalize_radian(-3.14), 6)
    -3.14
    >>> round(normalize_radian(3.15), 6)
    -3.133185
    >>> round(normalize_radian(-3.15), 6)
    3.133185
    """

    v = math.fmod(rad, 2 * math.pi)
    if -math.pi < v and v <= math.pi:
        return v
    else:
        return v - math.copysign(2 * math.pi, v)


def calc_azimuth(p1: Point, p2: Point):
    """Return the distance between p1 and p2

    Arguments:
        p1 {Point} -- base point
        p2 {Point} -- target point

    >>> from shapely.geometry import Point
    >>> round(calc_azimuth(Point(0, 0), Point(1, math.sqrt(3))), 6)
    1.047198
    >>> round(calc_azimuth(Point(0, 0), Point(1, -1)), 6)
    -0.785398
    """

    return math.atan2(p2.y - p1.y, p2.x - p1.x)


def calc_distance(p1: Point, p2: Point):
    """Return the distance between p1 and p2

    Arguments:
        p1 {Point} -- base point
        p2 {Point} -- target point

    >>> from shapely.geometry import Point
    >>> round(calc_distance(Point(0, 0), Point(1, 0)), 6)
    1.0
    >>> round(calc_distance(Point(0, 0), Point(3, -4)), 6)
    5.0
    """

    return math.hypot(p1.x - p2.x, p1.y - p2.y)


def calc_lateral_offset(p1: Point, p2: Point, azimuth_rad: float):
    """Return the lateral offset of p2 from p1

    Arguments:
        p1 {Point} -- base point
        p2 {Point} -- target point
        azimuth_rad {float} -- angle of X-axis in rad

    >>> from shapely.geometry import Point
    >>> round(calc_lateral_offset(Point(0, 0), Point(1, 2), 0), 6)
    2.0
    >>> round(calc_lateral_offset(Point(0, 0), Point(2, -1), 0), 6)
    -1.0
    """

    d = calc_distance(p1, p2)
    azimuth_rad_diff = calc_azimuth(p1, p2) - azimuth_rad

    return d * math.sin(azimuth_rad_diff)


def calc_longitudinal_offset(p1: Point, p2: Point, azimuth_rad: float):
    """Return the longitudinal offset of p2 from p1

    Arguments:
        p1 {Point} -- base point
        p2 {Point} -- target point
        azimuth_rad {float} -- angle of X-axis in rad

    >>> from shapely.geometry import Point
    >>> round(calc_longitudinal_offset(Point(0, 0), Point(1, 2), 0), 6)
    1.0
    >>> round(calc_longitudinal_offset(Point(0, 0), Point(2, -1), 0), 6)
    2.0
    """

    d = calc_distance(p1, p2)
    azimuth_rad_diff = calc_azimuth(p1, p2) - azimuth_rad

    return d * math.cos(azimuth_rad_diff)


if __name__ == "__main__":
    import doctest

    doctest.testmod()
