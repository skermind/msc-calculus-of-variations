from copy import deepcopy
import random
import numpy as np

def shift_move(path, terrain, sigma, alpha: float):
    """
    Randomly perturb a single interior waypoint.

    A copy of the supplied path is created and one randomly selected
    interior waypoint is displaced by a two-dimensional Gaussian random
    vector. The start and end points remain fixed so the boundary
    conditions are always satisfied.

    Parameters
    ----------
    path : Path
        Current candidate path.

    sigma : float
        Standard deviation of the Gaussian perturbation.

    Returns
    -------
    Path
        A new candidate path with one perturbed waypoint.
    """

    # Create a copy so the original path is unchanged
    candidate = deepcopy(path)

    # Number of waypoints
    n_points = len(candidate.points)

    # A path with fewer than three points has no interior point to move
    if n_points < 3:
        return candidate

    # Get costs at each interior point
    point_costs = []
    for i in range(1, n_points-1):
        x, y = candidate.points[i]
        cost = terrain.get_cost_at_coordinate(
            x,
            y
        )
        point_costs.append(cost)

    point_costs = np.array(point_costs)

    weights = point_costs ** alpha

    # Convert costs into probabilities
    probabilities = weights / weights.sum()

    # Select weighted random point
    idx = np.random.choice(
        np.arange(1, n_points-1),
        p=probabilities
    )

    # Sample a Gaussian displacement
    dx = np.random.normal(0.0, sigma)
    dy = np.random.normal(0.0, sigma)

    # Apply the perturbation
    candidate.points[idx][0] += dx
    candidate.points[idx][1] += dy

    return candidate


def segment_shift_move(path, terrain, sigma, alpha: float, segment_length: int = 5):
    """
    Randomly perturb a contiguous segment of interior waypoints.

    A copy of the supplied path is created. A starting waypoint is selected
    probabilistically based on the terrain cost of the segment locations, and
    a configurable number of neighbouring interior waypoints are displaced by
    the same two-dimensional Gaussian random vector. The start and end points
    remain fixed so the boundary conditions are preserved.

    Parameters
    ----------
    path : Path
        Current candidate path.

    terrain : Terrain
        Terrain object used to calculate waypoint costs.

    sigma : float
        Standard deviation of the Gaussian perturbation.

    alpha : float
        Controls the strength of the cost-based selection bias.

    segment_length : int
        Number of neighbouring interior waypoints to move.

    Returns
    -------
    Path
        A new candidate path with a shifted segment.
    """

    candidate = deepcopy(path)

    n_points = len(candidate.points)

    if n_points < segment_length + 2:
        return candidate

    # Valid segment starting locations (avoid fixed boundary points)
    possible_starts = np.arange(1, n_points - segment_length)

    # Calculate average terrain cost for each possible segment
    segment_costs = []

    for start in possible_starts:
        costs = []

        for idx in range(start, start + segment_length):
            x, y = candidate.points[idx]
            costs.append(
                terrain.get_cost_at_coordinate(x, y)
            )

        segment_costs.append(np.mean(costs))

    segment_costs = np.array(segment_costs)

    weights = segment_costs ** alpha
    probabilities = weights / weights.sum()

    # Select a segment start using weighted probability
    start_position = np.random.choice(
        len(possible_starts),
        p=probabilities
    )

    start_idx = possible_starts[start_position]

    # Apply the same displacement to the whole segment
    displacement = np.random.normal(
        0.0,
        sigma,
        size=2
    )

    for idx in range(start_idx, start_idx + segment_length):
        candidate.points[idx][0] += displacement[0]
        candidate.points[idx][1] += displacement[1]

    return candidate