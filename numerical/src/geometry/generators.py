# generators.py

import numpy as np
from .path import Path
from tests.constraints import valid_path_terrain
from optimisation.moves import segment_shift_move, path_intersects_impassable, smooth_obstacle_shift
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

    window=8,

    amplitude_step=1.0,

    max_amplitude=30.0,

    max_attempts=10

):

    """

    Construct a smooth feasible initial path around impassable terrain.

    Unlike the stochastic segment-shift approach, this method deliberately

    creates a smooth local detour around each obstacle.

    """

    current_path = copy.deepcopy(path)

    # ---------------------------------------------------------

    # Already feasible

    # ---------------------------------------------------------

    if not path_intersects_impassable(current_path, terrain):

        print("Initial path is already feasible")

        return current_path

    # ---------------------------------------------------------

    # Repeatedly repair the path

    #

    # This allows multiple obstacles to be handled sequentially.

    # ---------------------------------------------------------

    for attempt in range(max_attempts):

        new_path = smooth_obstacle_shift(

            current_path,

            terrain,

            window=window,

            amplitude_step=amplitude_step,

            max_amplitude=max_amplitude

        )

        if new_path is None:

            break

        current_path = new_path

        # -----------------------------------------------------

        # Check whether the entire path is now feasible

        # -----------------------------------------------------

        if not path_intersects_impassable(

            current_path,

            terrain

        ):

            print(

                f"Smooth feasible path found after "

                f"{attempt + 1} repair(s)"

            )

            return current_path

    raise ValueError(

        "Could not construct a smooth feasible initial path"

    )