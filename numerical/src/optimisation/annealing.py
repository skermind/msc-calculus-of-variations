"""
Simulated annealing optimisation for path planning problems.

The algorithm searches for a low-cost path by accepting both improving
and occasional worsening moves. The probability of accepting worse moves
decreases as the temperature cools.
"""

import numpy as np
import pandas as pd
import copy

from optimisation.moves import shift_move, segment_shift_move
from cost_functions.terrain_cost import terrain_cost, terrain_curvature_cost, total_cost
from tests.constraints import valid_path_order

def simulated_annealing(
    initial_path,
    terrain,
    iterations=10000,
    initial_temperature=10,
    cooling_rate=0.995,
    sigma=1.0,
    alpha=1.0
):
    """
    Perform simulated annealing to minimise terrain path cost.

    Parameters
    ----------
    initial_path : Path
        Initial feasible path.

    terrain : Terrain
        Terrain object used to evaluate cost.

    iterations : int
        Number of optimisation iterations.

    initial_temperature : float
        Starting temperature.

    cooling_rate : float
        Multiplicative cooling factor.

    sigma : float
        Standard deviation of random path perturbations.

    Returns
    -------
    best_path : Path
        Lowest cost path discovered.

    best_cost : float
        Cost of the best path.

    history : list
        Cost history during optimisation.
    """

    # Current solution
    current_path = initial_path
    current_cost = terrain_cost(current_path, terrain)

    # Best solution found
    best_path = current_path
    best_cost = current_cost

    temperature = initial_temperature

    history = []
    path_history = []

    for i in range(iterations):
        # Generate neighbour
        candidate_path = shift_move(
            current_path,
            terrain,
            sigma,
            alpha
        )
        candidate_cost = terrain_cost(
            candidate_path,
            terrain
        )
        # Cost difference
        delta = candidate_cost - current_cost
        # Accept better solutions
        if delta < 0:
            accept = True
        # Accept worse solutions probabilistically
        else:

            probability = np.exp(
                -delta / temperature
            )
            accept = np.random.random() < probability


        if accept:
            current_path = candidate_path
            current_cost = candidate_cost

            # store accepted path
            path_history.append(current_path)


        # Update best solution
        if current_cost < best_cost:
            best_path = current_path
            best_cost = current_cost


        # Cool temperature
        temperature *= cooling_rate
        history.append(
            {
                "iteration": i,
                "current_cost": current_cost,
                "best_cost": best_cost,
                "temperature": temperature
            }
        )


    return best_path, best_cost, history, path_history

def simulated_segment_annealing(
    initial_path,
    terrain,
    iterations=10000,
    initial_temperature=10,
    cooling_rate=0.995,
    sigma=1.0,
    alpha=1.0,
    segment_length=5
):
    """
    Perform simulated annealing to minimise terrain path cost.

    Parameters
    ----------
    initial_path : Path
        Initial feasible path.

    terrain : Terrain
        Terrain object used to evaluate cost.

    iterations : int
        Number of optimisation iterations.

    initial_temperature : float
        Starting temperature.

    cooling_rate : float
        Multiplicative cooling factor.

    sigma : float
        Standard deviation of random path perturbations.

    Returns
    -------
    best_path : Path
        Lowest cost path discovered.

    best_cost : float
        Cost of the best path.

    history : list
        Cost history during optimisation.
    """

    # Current solution
    current_path = initial_path
    current_cost = terrain_cost(current_path, terrain)

    # Best solution found
    best_path = current_path
    best_cost = current_cost

    temperature = initial_temperature

    history = []
    path_history = []

    for i in range(iterations):
        # Generate neighbour
        candidate_path = segment_shift_move(
            current_path,
            terrain,
            sigma,
            alpha,
            segment_length
        )
        candidate_cost = terrain_cost(
            candidate_path,
            terrain
        )

        # check ordering
        if not valid_path_order(candidate_path):
            accept = False

        # Cost difference
        delta = candidate_cost - current_cost
        # Accept better solutions
        if delta < 0:
            accept = True
        # Accept worse solutions probabilistically
        else:

            probability = np.exp(
                -delta / temperature
            )
            accept = np.random.random() < probability


        if accept:
            current_path = candidate_path
            current_cost = candidate_cost

            # store accepted path
            path_history.append(current_path)


        # Update best solution
        if current_cost < best_cost:
            best_path = current_path
            best_cost = current_cost


        # Cool temperature
        temperature *= cooling_rate
        history.append(
            {
                "iteration": i,
                "current_cost": current_cost,
                "best_cost": best_cost,
                "temperature": temperature
            }
        )


    return best_path, best_cost, history, path_history

def simulated_segment_annealing_curvature(
    initial_path,
    terrain,
    iterations=10000,
    initial_temperature=10,
    cooling_rate=0.995,
    sigma=1.0,
    alpha=1.0,
    segment_length=5,
    terrain_weight=1,
    lam=1,
    mu=1
):
    """
    Perform simulated annealing to minimise terrain path cost.

    Parameters
    ----------
    initial_path : Path
        Initial feasible path.

    terrain : Terrain
        Terrain object used to evaluate cost.

    iterations : int
        Number of optimisation iterations.

    initial_temperature : float
        Starting temperature.

    cooling_rate : float
        Multiplicative cooling factor.

    sigma : float
        Standard deviation of random path perturbations.

    Returns
    -------
    best_path : Path
        Lowest cost path discovered.

    best_cost : float
        Cost of the best path.

    history : list
        Cost history during optimisation.
    """

    # Current solution
    current_path = copy.deepcopy(initial_path)
    current_cost = total_cost(
                        current_path,
                        terrain,
                        terrain_weight,
                        lam,
                        mu
                    )

    # Best solution found
    best_path = copy.deepcopy(current_path)
    best_cost = current_cost

    temperature = initial_temperature

    history = []
    path_history = []

    for i in range(iterations):
        # Generate neighbour
        candidate_path = segment_shift_move(
            current_path,
            terrain,
            sigma,
            alpha,
            segment_length
        )
        
        candidate_cost = total_cost(
            candidate_path,
            terrain,
            terrain_weight,
            lam,
            mu
        )

        accept = False

        # Only accept paths satisfying the ordering constraint
        if valid_path_order(candidate_path):

            delta = candidate_cost - current_cost

            # Accept better solutions
            if delta < 0:
                accept = True

            # Accept worse solutions probabilistically
            else:
                probability = np.exp(
                    -delta / temperature
                )
                accept = np.random.random() < probability


        if accept:
            current_path = copy.deepcopy(candidate_path)
            current_cost = candidate_cost

            # store accepted path
            path_history.append(current_path)


        # Update best solution
        if current_cost < best_cost:
            best_path = copy.deepcopy(current_path)
            best_cost = current_cost


        # Cool temperature
        temperature *= cooling_rate
        history.append(
            {
                "iteration": i,
                "current_cost": current_cost,
                "best_cost": best_cost,
                "temperature": temperature
            }
        )


    return best_path, best_cost, history, path_history

def path_history_to_df(path_history):

    rows = []

    for iteration, path in enumerate(path_history):

        for point_id, point in enumerate(path.points):

            rows.append(
                {
                    "iteration": iteration,
                    "point_id": point_id,
                    "x": point[0],
                    "y": point[1]
                }
            )

    return pd.DataFrame(rows)