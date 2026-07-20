"""
Functions for generating and managing populations of candidate paths.

A population consists of multiple feasible paths and their associated
terrain costs.
"""

import pandas as pd

from optimisation.moves import shift_move
from cost_functions.terrain_cost import terrain_cost



def create_population(initial_path, terrain, n_paths, sigma):
    """
    Create an initial population of paths by applying random shifts.

    Parameters
    ----------
    initial_path : Path
        Starting feasible path.

    terrain : Terrain
        Terrain object used for cost calculation.

    n_paths : int
        Number of candidate paths.

    sigma : float
        Standard deviation of random path perturbations.

    Returns
    -------
    paths_df : pandas.DataFrame
        Dataframe containing path objects.

    costs_df : pandas.DataFrame
        Dataframe containing path costs.
    """

    paths = []
    costs = []

    for path_id in range(n_paths):

        # Create a random candidate
        candidate = initial_path

        # Apply several random moves
        for _ in range(10):
            candidate = shift_move(candidate, sigma)

        # Calculate cost
        cost = terrain_cost(candidate, terrain)

        paths.append(
            {
                "path_id": path_id,
                "path": candidate
            }
        )

        costs.append(
            {
                "path_id": path_id,
                "cost": cost
            }
        )

    paths_df = pd.DataFrame(paths)

    costs_df = pd.DataFrame(costs)

    return paths_df, costs_df