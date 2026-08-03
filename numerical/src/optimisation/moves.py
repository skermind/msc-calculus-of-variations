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

    terrain_weight = 1.0
    curvature_weight = 1.0

    # Calculate a combined terrain and curvature score for each segment
    segment_costs = []

    for start in possible_starts:

        terrain_costs = []
        curvature_costs = []

        for idx in range(start, start + segment_length):

            x, y = candidate.points[idx]
            terrain_costs.append(
                terrain.get_cost_at_coordinate(x, y)
            )

            if 1 <= idx < n_points - 1:
                p_prev = candidate.points[idx - 1]
                p = candidate.points[idx]
                p_next = candidate.points[idx + 1]

                v1 = p - p_prev
                v2 = p_next - p

                n1 = np.linalg.norm(v1)
                n2 = np.linalg.norm(v2)

                if n1 > 0 and n2 > 0:
                    cosine = np.dot(v1, v2) / (n1 * n2)
                    cosine = np.clip(cosine, -1.0, 1.0)
                    curvature_costs.append(np.pi - np.arccos(cosine))

        terrain_score = np.mean(terrain_costs)
        curvature_score = np.mean(curvature_costs) if curvature_costs else 0.0

        segment_costs.append(
            terrain_weight * terrain_score +
            curvature_weight * curvature_score
        )

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

        candidate.points[idx][0] = np.clip(
            candidate.points[idx][0],
            0,
            terrain.width - 1
        )

        candidate.points[idx][1] = np.clip(
            candidate.points[idx][1],
            0,
            terrain.height - 1
        )

    # Reject moves that violate monotonic x-ordering
    x_values = candidate.points[:, 0]
    if not np.all(np.diff(x_values) > 0):
        return path

    return candidate

def segment_shift_move_impassible(path, terrain, sigma, alpha: float, segment_length: int = 5):
    """
    Randomly perturb a contiguous segment of interior waypoints that
    intersects impassable terrain.

    A copy of the supplied path is created. Each candidate segment is scored
    according to how many sampled points along its constituent line segments
    intersect impassable terrain. The segment with the highest score is then
    selected (breaking ties uniformly at random) and displaced by the same
    two-dimensional Gaussian random vector. The start and end points remain
    fixed so that the boundary conditions are preserved.

    This move is intended for constructing an initial feasible path rather
    than optimising the objective functional.

    Parameters
    ----------
    path : Path
        Current candidate path.

    terrain : Terrain
        Terrain object providing the impassable terrain mask.

    sigma : float
        Standard deviation of the Gaussian perturbation.

    alpha : float
        Controls the strength of the bias towards segments intersecting
        impassable terrain.

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

    # Score each segment by how much of its polyline intersects impassable terrain

    segment_scores = []

    for start in possible_starts:
        score = 0

        for idx in range(start, min(start + segment_length, n_points - 1)):

            p1 = candidate.points[idx]
            p2 = candidate.points[idx + 1]

            for t in np.linspace(0.0, 1.0, 20):
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])

                if terrain.is_impassable(x, y):
                    score += 1

        segment_scores.append(score)

    segment_scores = np.array(segment_scores)

    max_score = segment_scores.max()

    if max_score == 0:
        start_position = np.random.randint(len(possible_starts))
    else:
        worst_segments = np.where(segment_scores == max_score)[0]
        start_position = np.random.choice(worst_segments)

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

        candidate.points[idx][0] = np.clip(
            candidate.points[idx][0],
            0,
            terrain.width - 1
        )

        candidate.points[idx][1] = np.clip(
            candidate.points[idx][1],
            0,
            terrain.height - 1
        )

    # Reject moves that violate monotonic x-ordering
    x_values = candidate.points[:, 0]
    if not np.all(np.diff(x_values) > 0):
        return path

    return candidate