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
    
    def terrain_cost(path, terrain):
        """
        Calculate the terrain cost of a discrete path.
        The continuous variational problem is defined by the functional:
            J(gamma) = ∫_gamma c(x,y) ds
        where:
            c(x,y) : Ω → [0,1]
        is the terrain cost field and ds represents an infinitesimal
        element of path length.
        This function approximates the integral numerically using a
        discrete path:
            gamma_h = {p_0, p_1, ..., p_N}
        giving:
            J(gamma_h) ≈ Σ c(p_i)|p_(i+1)-p_i|
        Parameters
        ----------
        path : Path
            A discrete approximation of the continuous curve gamma.
            The path should contain an ordered sequence of points:
                [(x_0,y_0), (x_1,y_1), ..., (x_N,y_N)]
            where the first and last points satisfy the boundary
            conditions.
        terrain : Terrain
            A terrain cost field used to evaluate c(x,y).
        Returns
        -------
        float
            Total accumulated terrain cost along the path.
        Notes
        -----
        The current implementation assumes that:
        - Terrain cost is independent of direction.
        - The path cost increases with both terrain difficulty
        and path length.
        - Costs are normalised to the interval [0,1].

        """

        total_cost = 0.0
        points = path.points
        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])

            # Length of the current path segment
            segment_length = np.linalg.norm(p2 - p1)

            # Terrain cost at current point
            point_cost = terrain.get_cost(
                i,
                i + 1
            )
            total_cost += point_cost * segment_length

        return total_cost