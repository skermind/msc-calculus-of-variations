import numpy as np


class Terrain:
    """
    Represents a 2D terrain cost field over a rectangular domain.

    The continuous domain is defined as:

        Ω = [0, width] × [0, height]

    The domain is discretised into a numerical grid for computation.

    Each grid point stores a terrain cost value:

        - 0 : low cost terrain
        - 1 : high cost terrain

    Initially, the terrain is assumed to have a uniform cost.

    Parameters
    ----------
    width : float, optional
        Physical width of the domain.

    height : float, optional
        Physical height of the domain.

    resolution : int, optional
        Number of grid points used in each spatial direction.
        The resulting grid has dimensions:

            resolution × resolution

    cost : float, optional
        Uniform cost value assigned to every point in the terrain.

    Attributes
    ----------
    width : float
        Physical width of the domain.

    height : float
        Physical height of the domain.

    resolution : int
        Number of grid points per dimension.

    cost : float
        Base terrain cost.

    grid : numpy.ndarray
        Two-dimensional array containing the discretised terrain cost field.
    """

    def __init__(
        self,
        width=1,
        height=1,
        resolution=100,
        cost=0.1
    ):
        """
        Initialise a uniform terrain cost field.

        Parameters
        ----------
        width : float
            Physical width of the domain.

        height : float
            Physical height of the domain.

        resolution : int
            Number of grid points per dimension.

        cost : float
            Cost assigned to every point in the domain.
        """

        self.width = width
        self.height = height
        self.resolution = resolution
        self.cost = cost

        self.grid = np.full(
            (resolution, resolution),
            cost
        )

    def get_cost(self, i, j):
        """
        Return the terrain cost at a given grid location.

        Parameters
        ----------
        i : int
            Row index of the grid.

        j : int
            Column index of the grid.

        Returns
        -------
        float
            Cost value at the specified grid point.
        """

        return self.grid[i, j]