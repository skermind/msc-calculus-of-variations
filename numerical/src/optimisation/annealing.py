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
from cost_functions.terrain_cost import terrain_cost, terrain_curvature_cost, total_cost, terrain_length_cost
from tests.constraints import valid_path_order, valid_path_terrain, geometric_guard

def simulated_annealing(
    initial_path,
    terrain,
    iterations=10000,
    initial_temperature=10,
    cooling_rate=0.995,
    sigma=1.0,
    alpha=1.0,
    terrain_weight=1,
    lam=0,
    mu=0,
    debug=False
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

    terrain_weight : float
        Weight for terrain cost (w_c). Defaults to 1 for backward compatibility.

    lam : float
        Weight for curvature cost (lambda). Defaults to 0.

    mu : float
        Weight for length cost (mu). Defaults to 0.

    debug : bool
        If True, print lightweight debugging information for the first few iterations.

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
    current_cost = total_cost(current_path, terrain, terrain_weight, lam, mu)

    if debug:
        print("=== Simulated Annealing Parameters ===")
        print(f"Terrain weight (w_c): {terrain_weight}")
        print(f"Curvature weight (lambda): {lam}")
        print(f"Length weight (mu): {mu}")
        print(f"Initial cost: {current_cost}")
        print("======================================")

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
        candidate_cost = total_cost(
            candidate_path,
            terrain,
            terrain_weight,
            lam,
            mu
        )

        if debug and i < 10:
            print("\nIteration:", i)
            print("Current cost:", current_cost)
            print("Candidate cost:", candidate_cost)
            print("Delta:", candidate_cost - current_cost)
            print("Candidate components:")
            print("Terrain:", terrain_cost(candidate_path, terrain))
            print("Curvature:", terrain_curvature_cost(candidate_path))
            print("Length:", terrain_length_cost(candidate_path))

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
    segment_length=5,
    terrain_weight=1,
    lam=0,
    mu=0,
    debug=False
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

    terrain_weight : float
        Weight for terrain cost (w_c). Defaults to 1 for backward compatibility.

    lam : float
        Weight for curvature cost (lambda). Defaults to 0.

    mu : float
        Weight for length cost (mu). Defaults to 0.

    debug : bool
        If True, print lightweight debugging information for the first few iterations.

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
    current_cost = total_cost(current_path, terrain, terrain_weight, lam, mu)

    if debug:
        print("=== Segment Simulated Annealing Parameters ===")
        print(f"Terrain weight (w_c): {terrain_weight}")
        print(f"Curvature weight (lambda): {lam}")
        print(f"Length weight (mu): {mu}")
        print(f"Initial cost: {current_cost}")
        print("==============================================")

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
        candidate_cost = total_cost(
            candidate_path,
            terrain,
            terrain_weight,
            lam,
            mu
        )

        if debug and i < 10:
            print("\nIteration:", i)
            print("Current cost:", current_cost)
            print("Candidate cost:", candidate_cost)
            print("Delta:", candidate_cost - current_cost)
            print("Candidate components:")
            print("Terrain:", terrain_cost(candidate_path, terrain))
            print("Curvature:", terrain_curvature_cost(candidate_path))
            print("Length:", terrain_length_cost(candidate_path))

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
    mu=1,
    max_candidate_attempts=4,
    retry_sigma_decay=0.7,
    debug=False
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

    terrain_weight, lam, mu : floats
        Weights for the terrain, curvature and length costs respectively.

    max_candidate_attempts : int
        How many times to try generating a feasible candidate before giving up for an iteration.

    retry_sigma_decay : float
        Factor to reduce sigma when a candidate is identical or violates quick checks.

    debug : bool
        If True, print lightweight debugging information for the first few iterations.

    Returns
    -------
    best_path : Path
        Lowest cost path discovered.

    best_cost : float
        Cost of the best path.

    history : list
        Cost history during optimisation.
    """
    if debug:
        print("=== Annealing Parameters ===")
        print(f"Terrain weight (w_c): {terrain_weight}")
        print(f"Curvature weight (lambda): {lam}")
        print(f"Length weight (mu): {mu}")
        print("============================")

    # Current solution
    current_path = copy.deepcopy(initial_path)
    current_cost = total_cost(
                        current_path,
                        terrain,
                        terrain_weight,
                        lam,
                        mu
                    )

    if debug:
        print(f"Initial cost: {current_cost}")

    # Best solution found
    best_path = copy.deepcopy(current_path)
    best_cost = current_cost

    temperature = initial_temperature

    history = []
    path_history = []

    accepted_count = 0
    attempted_count = 0

    base_step = np.median(np.linalg.norm(np.diff(initial_path.points, axis=0), axis=1))

    for i in range(iterations):
        accept = False
        candidate_path = None
        candidate_cost = None

        local_sigma = sigma
        current_diff = np.diff(current_path.points, axis=0)
        current_seg_len = np.linalg.norm(current_diff, axis=1)
        current_abs_dy = np.abs(current_diff[:, 1])
        current_max_seg = current_seg_len.max()
        current_max_dy = current_abs_dy.max()
        adaptive_base_step = max(base_step, current_max_seg / 3.0, current_max_dy / 2.5)

        for _ in range(max_candidate_attempts):
            proposal = segment_shift_move(
                current_path,
                terrain,
                local_sigma,
                alpha,
                segment_length
            )

            if np.array_equal(proposal.points, current_path.points):
                local_sigma *= retry_sigma_decay
                continue

            if (
                valid_path_order(proposal)
                and valid_path_terrain(proposal, terrain)
                and geometric_guard(proposal, adaptive_base_step)
            ):
                proposal_diff = np.diff(proposal.points, axis=0)
                proposal_seg_len = np.linalg.norm(proposal_diff, axis=1)
                proposal_abs_dy = np.abs(proposal_diff[:, 1])

                if (
                    proposal_seg_len.max() > 1.05 * current_max_seg
                    or proposal_abs_dy.max() > 1.05 * current_max_dy
                ):
                    local_sigma *= retry_sigma_decay
                    continue

                candidate_path = proposal
                candidate_cost = total_cost(
                    candidate_path,
                    terrain,
                    terrain_weight,
                    lam,
                    mu
                )
                break

            local_sigma *= retry_sigma_decay

        attempted_count += 1

        if candidate_path is not None:
            if debug and i < 20:
                print("\nIteration:", i)
                print("Current cost:", current_cost)
                print("Candidate cost:", candidate_cost)
                print("Delta:", candidate_cost - current_cost)
                print("Candidate sigma:", local_sigma)
                print("Candidate components:")
                print("Terrain:", terrain_cost(candidate_path, terrain))
                print("Curvature:", terrain_curvature_cost(candidate_path))
                print("Length:", terrain_length_cost(candidate_path))

            delta = candidate_cost - current_cost

            if delta < 0:
                accept = True
            else:
                probability = np.exp(-delta / temperature)
                accept = np.random.random() < probability

        if accept:
            current_path = copy.deepcopy(candidate_path)
            current_cost = candidate_cost
            accepted_count += 1
            path_history.append(copy.deepcopy(current_path))

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

    if debug:
        print(f"Accepted {accepted_count} / {attempted_count} proposals ({(accepted_count/attempted_count*100) if attempted_count else 0:.2f}%)")

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