# generators.py

import numpy as np
from .path import Path
from tests.constraints import valid_path_terrain
from optimisation.moves import segment_shift_move, segment_shift_move_impassible
import copy
from cost_functions.terrain_cost import  total_cost


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

def find_feasible_initial_path(
    path,
    terrain,
    sigma=5,
    alpha=10,
    max_attempts=10000
):

    current_path = copy.deepcopy(path)

    # Return immediately if the supplied path is already feasible
    if valid_path_terrain(current_path, terrain):
        print("Initial path is already feasible")
        return current_path

    for attempt in range(max_attempts):

        current_path = segment_shift_move_impassible(
            current_path,
            terrain,
            sigma,
            alpha
        )

        if valid_path_terrain(current_path, terrain):
            print(f"Feasible path found after {attempt + 1} attempts")
            return current_path

    raise ValueError(
        "Could not find feasible initial path"
    )