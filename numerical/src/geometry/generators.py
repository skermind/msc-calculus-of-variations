# generators.py

import numpy as np
from .path import Path


def straight_line_path(start, end, num_points=50):
    """
    Generate an initial path using linear interpolation
    between two boundary points.

    Creates a discrete approximation:

        gamma_h = {p0,p1,...,pN}

    satisfying:

        gamma(0) = start
        gamma(1) = end

    Parameters
    ----------
    start : tuple
        Starting point (x,y).

    end : tuple
        Ending point (x,y).

    num_points : int
        Number of points used to discretise the curve.

    Returns
    -------
    Path
        Discrete straight-line path.
    """

    x_values = np.linspace(
        start[0],
        end[0],
        num_points
    )

    y_values = np.linspace(
        start[1],
        end[1],
        num_points
    )

    points = list(
        zip(x_values, y_values)
    )

    return Path(points)