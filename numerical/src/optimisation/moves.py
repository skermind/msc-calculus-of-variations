from copy import deepcopy
import copy
import random
import numpy as np
from cost_functions.terrain_cost import terrain_curvature_cost, terrain_length_cost

def _normalised_selection_probabilities(scores: np.ndarray, alpha: float) -> np.ndarray:
    """Convert non-negative scores into a stable probability vector."""
    safe_scores = np.clip(scores, 0.0, None)
    weights = np.power(safe_scores, alpha)
    total = weights.sum()

    if not np.isfinite(total) or total <= 0.0:
        return np.full_like(weights, 1.0 / len(weights), dtype=float)

    return weights / total


def _smooth_segment_weights(
    n_points: int,
    start_idx: int,
    segment_length: int,
    blend_width: int
) -> np.ndarray:
    """
    Build a smooth displacement profile for the selected segment.

    The profile is zero at the segment boundaries and reaches a maximum in the
    middle, so the path bends smoothly rather than being translated rigidly.
    """
    weights = np.zeros(n_points, dtype=float)
    end_idx = start_idx + segment_length - 1

    if segment_length <= 1:
        return weights

    for idx in range(start_idx, end_idx + 1):
        t = (idx - start_idx) / (segment_length - 1)
        weights[idx] = np.sin(np.pi * t)

    if blend_width <= 0:
        return weights

    for idx in range(1, n_points - 1):
        if start_idx <= idx <= end_idx:
            continue

        distance = min(abs(idx - start_idx), abs(idx - end_idx))
        if distance <= blend_width:
            weights[idx] = 0.5 * (1.0 + np.cos(np.pi * distance / blend_width))

    return weights


def _max_feasible_scale(
    points: np.ndarray,
    deltas: np.ndarray,
    terrain
) -> float:
    """
    Largest scale in [0, 1] keeping every moved point inside terrain bounds.
    """
    lower_bounds = np.array([0.0, 0.0], dtype=float)
    upper_bounds = np.array(
        [float(terrain.width - 1), float(terrain.height - 1)],
        dtype=float
    )
    max_scale = 1.0

    moving_indices = np.where(np.any(deltas != 0.0, axis=1))[0]

    for idx in moving_indices:
        point = points[idx]
        delta = deltas[idx]

        for dim in range(2):
            component = delta[dim]
            if component > 0.0:
                max_scale = min(max_scale, (upper_bounds[dim] - point[dim]) / component)
            elif component < 0.0:
                max_scale = min(max_scale, (lower_bounds[dim] - point[dim]) / component)

    return float(np.clip(max_scale, 0.0, 1.0))


def _apply_smooth_segment_displacement(
    candidate,
    terrain,
    start_idx: int,
    segment_length: int,
    sigma: float,
    displacement: np.ndarray
) -> bool:
    """
    Apply a smooth, tapered segment move while preserving boundary feasibility.
    """
    n_points = len(candidate.points)
    blend_width = max(1, segment_length // 2)
    weights = _smooth_segment_weights(
        n_points=n_points,
        start_idx=start_idx,
        segment_length=segment_length,
        blend_width=blend_width
    )

    amplitude = float(np.random.normal(0.0, sigma))
    if np.random.random() < 0.5:
        amplitude *= -1.0

    deltas = np.zeros((n_points, 2), dtype=float)
    deltas[:, 1] = weights * amplitude

    scale = _max_feasible_scale(candidate.points, deltas, terrain)

    if scale <= 0.0:
        return False

    original_points = candidate.points.copy()
    candidate.points = candidate.points + (scale * deltas)

    moved_indices = np.where(weights > 0.0)[0]
    if moved_indices.size == 0:
        return False

    segment_start = max(0, moved_indices.min() - 1)
    segment_end = min(len(candidate.points) - 1, moved_indices.max() + 1)

    original_lengths = np.linalg.norm(
        np.diff(original_points[segment_start:segment_end + 1], axis=0),
        axis=1
    )
    moved_lengths = np.linalg.norm(
        np.diff(candidate.points[segment_start:segment_end + 1], axis=0),
        axis=1
    )

    if np.any(moved_lengths > 4.0 * np.maximum(original_lengths, 1e-9)):
        candidate.points = original_points
        return False

    return True


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

    # Convert costs into probabilities
    probabilities = _normalised_selection_probabilities(point_costs, alpha)

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

    Segment selection is probabilistic and biased towards local regions with
    high combined terrain, curvature, and length contribution.
    """

    candidate = deepcopy(path)
    n_points = len(candidate.points)

    if n_points < segment_length + 2:
        return candidate

    possible_starts = np.arange(1, n_points - segment_length)

    terrain_scores = []
    curvature_scores = []
    length_scores = []

    for start in possible_starts:
        end = start + segment_length - 1

        local_terrain = 0.0
        local_length = 0.0
        for idx in range(start, min(end, n_points - 1)):
            p1 = candidate.points[idx]
            p2 = candidate.points[idx + 1]
            segment_len = np.linalg.norm(p2 - p1)
            midpoint = 0.5 * (p1 + p2)
            local_terrain += terrain.get_cost_at_coordinate(midpoint[0], midpoint[1]) * segment_len
            local_length += segment_len

        local_curvature = 0.0
        for idx in range(max(1, start), min(end, n_points - 2) + 1):
            p_prev = candidate.points[idx - 1]
            p_curr = candidate.points[idx]
            p_next = candidate.points[idx + 1]
            bend = p_next - 2.0 * p_curr + p_prev
            local_curvature += np.linalg.norm(bend) ** 2

        terrain_scores.append(local_terrain)
        curvature_scores.append(local_curvature)
        length_scores.append(local_length)

    terrain_scores = np.array(terrain_scores, dtype=float)
    curvature_scores = np.array(curvature_scores, dtype=float)
    length_scores = np.array(length_scores, dtype=float)

    def _normalise(values: np.ndarray) -> np.ndarray:
        v_min = values.min()
        v_max = values.max()
        if v_max <= v_min:
            return np.zeros_like(values)
        return (values - v_min) / (v_max - v_min)

    terrain_norm = _normalise(terrain_scores)
    curvature_norm = _normalise(curvature_scores)
    length_norm = _normalise(length_scores)

    # Combined local cost signal used only for selecting where to move.
    segment_selection_scores = terrain_norm + curvature_norm + length_norm

    probabilities = _normalised_selection_probabilities(
        segment_selection_scores + 1e-12,
        alpha
    )

    start_position = np.random.choice(len(possible_starts), p=probabilities)
    start_idx = possible_starts[start_position]

    if not _apply_smooth_segment_displacement(
        candidate=candidate,
        terrain=terrain,
        start_idx=start_idx,
        segment_length=segment_length,
        sigma=sigma,
        displacement=np.array([0.0, 0.0])
    ):
        return path

    x_values = candidate.points[:, 0]
    if not np.all(np.diff(x_values) > 0):
        return path

    return candidate

def path_intersects_impassable(path, terrain, samples_per_segment=20):

    """

    Check whether any part of the polyline intersects impassable terrain.

    This checks the line between waypoints rather than just the waypoints

    themselves.

    """

    points = path.points

    for i in range(len(points) - 1):

        p1 = points[i]

        p2 = points[i + 1]

        for t in np.linspace(0.0, 1.0, samples_per_segment):

            x = p1[0] + t * (p2[0] - p1[0])

            y = p1[1] + t * (p2[1] - p1[1])

            if terrain.is_impassable(x, y):

                return True

    return False

def smooth_obstacle_shift(
    path,
    terrain,
    window=8,
    amplitude_step=1.0,
    max_amplitude=30.0,
    samples_per_segment=20
):
    """
    Construct a smooth local detour around impassable terrain.
    The path is locally displaced in the y-direction using a sinusoidal
    bump. The displacement is zero at the edges of the affected region
    and reaches its maximum near the centre.
    Both upward and downward detours are tested. The smallest feasible
    displacement is preferred, with curvature used to break ties.
    Parameters
    ----------
    path : Path
        Current path.
    terrain : Terrain
        Terrain object containing the impassable mask.
    window : int
        Number of waypoints before and after the obstacle affected region.
    amplitude_step : float
        Amount by which the displacement amplitude is increased.
    max_amplitude : float
        Maximum displacement that will be attempted.
    samples_per_segment : int
        Number of samples used when checking line/terrain intersections.
    Returns
    -------
    Path or None
        Smooth feasible path, or None if no feasible path was found.
    """
    original = copy.deepcopy(path)
    points = original.points
    n_points = len(points)
    # ---------------------------------------------------------
    # Find path segments that intersect impassable terrain
    # ---------------------------------------------------------
    blocked_segments = []

    for i in range(n_points - 1):
        p1 = points[i]
        p2 = points[i + 1]
        intersects = False
        for t in np.linspace(0.0, 1.0, samples_per_segment):

            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])

            if terrain.is_impassable(x, y):

                intersects = True
                break

        if intersects:

            blocked_segments.append(i)

    # Nothing to fix
    if not blocked_segments:
        return original
    # ---------------------------------------------------------
    # Determine local region around obstacle
    # ---------------------------------------------------------

    first_blocked = blocked_segments[0]

    last_blocked = blocked_segments[-1]

    start_idx = max(1, first_blocked - window)

    end_idx = min(n_points - 2, last_blocked + 1 + window)

    if end_idx <= start_idx:

        return None

    # ---------------------------------------------------------

    # Generate candidate detours

    # ---------------------------------------------------------

    candidates = []

    amplitudes = np.arange(

        amplitude_step,

        max_amplitude + amplitude_step,

        amplitude_step

    )

    for direction in (-1, 1):

        for amplitude in amplitudes:

            candidate = copy.deepcopy(original)

            # ---------------------------------------------

            # Smooth sinusoidal displacement

            # ---------------------------------------------

            region_length = end_idx - start_idx

            for j, idx in enumerate(

                range(start_idx, end_idx + 1)

            ):

                if region_length == 0:

                    weight = 0.0

                else:

                    t = j / region_length

                    weight = np.sin(np.pi * t)

                candidate.points[idx, 1] += (

                    direction

                    * amplitude

                    * weight

                )

            # ---------------------------------------------

            # Check x ordering

            # ---------------------------------------------

            x_values = candidate.points[:, 0]

            if not np.all(np.diff(x_values) > 0):

                continue

            # ---------------------------------------------

            # Check obstacle avoidance

            # ---------------------------------------------

            if path_intersects_impassable(

                candidate,

                terrain,

                samples_per_segment=samples_per_segment

            ):

                continue

            # ---------------------------------------------

            # Calculate smoothness

            # ---------------------------------------------

            curvature = terrain_curvature_cost(candidate)

            length = terrain_length_cost(candidate)

            candidates.append(

                {

                    "path": candidate,

                    "amplitude": amplitude,

                    "curvature": curvature,

                    "length": length,

                    "direction": direction

                }

            )

            # We have found the smallest feasible amplitude

            # for this direction, so move to the other direction.

            break

    # ---------------------------------------------------------

    # No feasible detour

    # ---------------------------------------------------------

    if not candidates:

        return None

    # ---------------------------------------------------------

    # Choose smoothest candidate

    #

    # Primary objective: curvature

    # Secondary objective: path length

    # ---------------------------------------------------------

    candidates.sort(

        key=lambda c: (

            c["curvature"],

            c["length"]

        )

    )

    best = candidates[0]

    return best["path"]