import numpy as np


def terrain_cost(path, terrain):
    """
    Calculate the terrain cost of a discrete path.

    Approximates:

        J(gamma) = ∫_gamma c(x,y) ds

    using:

        J(gamma_h) ≈ Σ c(p_i)|p_(i+1)-p_i|

    Parameters
    ----------
    path : Path
        Discrete path.

    terrain : Terrain
        Terrain cost field.

    Returns
    -------
    float
        Total path cost.
    """

    total_cost = 0.0

    points = path.points

    for i in range(len(points)-1):

        p1 = points[i]
        p2 = points[i+1]

        distance = np.linalg.norm(
            p2-p1
        )

        # For now use uniform terrain
        cost = terrain.cost

        total_cost += cost * distance

    return total_cost