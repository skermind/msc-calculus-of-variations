import numpy as np


def valid_path_order(path):
    """
    Check that path progresses monotonically in x.

    Returns
    -------
    bool
        True if all interior points maintain ordering.
    """

    x_values = path.points[:,0]

    return np.all(
        np.diff(x_values) > 0
    )

def valid_path_terrain(path, terrain, samples=20):
    """
    Check that the path does not enter impassable terrain.

    Both waypoints and the line segments connecting consecutive waypoints
    are checked by sampling intermediate points along each segment.

    Parameters
    ----------
    path : Path
        Candidate path to validate.

    terrain : Terrain
        Terrain object containing the impassable terrain mask.

    samples : int
        Number of intermediate points sampled along each path segment.

    Returns
    -------
    bool
        True if the entire path remains outside impassable terrain.
    """

    for i in range(len(path.points) - 1):

        start = path.points[i]
        end = path.points[i + 1]

        for t in np.linspace(0, 1, samples):

            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])

            if terrain.is_impassable(x, y):
                return False

    return True