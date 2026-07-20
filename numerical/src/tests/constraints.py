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