import numpy as np


def terrain_cost(path, terrain):
    """
    Approximate:
        J(gamma)=∫_gamma c(x,y) ds
    using a discrete path.
    """

    total_cost = 0.0

    points = path.points

    for i in range(len(points)-1):

        p1 = points[i]
        p2 = points[i+1]

        segment_length = np.linalg.norm(
            p2-p1
        )

        midpoint = (p1+p2)/2

        cost = terrain.get_cost_at_coordinate(
            midpoint[0],
            midpoint[1]
        )
        total_cost += cost * segment_length
    return total_cost


def terrain_curvature_cost(path):
    """
    Approximate:
        J_curve(gamma)=∫_gamma κ(s)^2 ds
    using a discrete path.

    The curvature is approximated using the second finite difference,

        κ_i ≈ p_{i+1} - 2p_i + p_{i-1},

    which measures the deviation of each interior point from a straight
    line joining its neighbours. Straight sections contribute zero,
    whilst sharp bends incur a larger penalty.
    """

    total_cost = 0.0

    points = path.points

    for i in range(1, len(points) - 1):

        p_prev = points[i - 1]
        p_curr = points[i]
        p_next = points[i + 1]

        curvature = p_next - 2 * p_curr + p_prev

        total_cost += np.linalg.norm(curvature) ** 2

    return total_cost

def total_cost(path, terrain, lam):
    return (
        terrain_cost(path, terrain)
        + lam * terrain_curvature_cost(path)
    )