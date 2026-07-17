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