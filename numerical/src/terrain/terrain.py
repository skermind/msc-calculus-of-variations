import numpy as np
import pandas as pd


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

        self.impassable = np.zeros(
            (resolution, resolution),
            dtype=bool
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

    def add_hill(self, centre, peak_cost=0.8, radius=20):
        """
        Add a smooth circular hill to the terrain cost field.

        The hill increases the terrain cost smoothly from the background
        cost at the edge of the radius to the peak cost at the centre.

        Parameters
        ----------
        centre : tuple
            Coordinates of the hill centre as (x, y).

        peak_cost : float
            Maximum cost at the centre of the hill.

        radius : float
            Radius of influence of the hill in physical coordinates.

        Notes
        -----
        The terrain cost is updated using a Gaussian profile:

            c(x,y) = background + (peak - background) * exp(-d^2/(2*sigma^2))

        where d is the distance from the hill centre.
        """

        x_centre, y_centre = centre

        # Convert physical coordinates into grid coordinates
        x = np.linspace(0, self.width, self.resolution)
        y = np.linspace(0, self.height, self.resolution)

        X, Y = np.meshgrid(x, y)

        distance_squared = (
            (X - x_centre) ** 2
            + (Y - y_centre) ** 2
        )

        # Choose sigma so the hill influence is approximately radius sized
        sigma = radius / 3

        hill = (peak_cost - self.cost) * np.exp(
            -distance_squared / (2 * sigma ** 2)
        )

        # Only add the hill inside the chosen radius
        hill[distance_squared > radius ** 2] = 0

        # Combine with existing terrain without reducing existing costs
        self.grid = np.maximum(
            self.grid,
            self.cost + hill
        )
    
    def add_impassable_square(self, bottom_left, top_right):
        """
        Add a square region of impassable terrain.

        Parameters
        ----------
        bottom_left : tuple
            Lower-left coordinate of the square as (x, y).

        top_right : tuple
            Upper-right coordinate of the square as (x, y).
        """

        x_min, y_min = bottom_left
        x_max, y_max = top_right

        x = np.linspace(0, self.width, self.resolution)
        y = np.linspace(0, self.height, self.resolution)

        X, Y = np.meshgrid(x, y)

        mask = (
            (X >= x_min)
            & (X <= x_max)
            & (Y >= y_min)
            & (Y <= y_max)
        )

        self.impassable[mask] = True

    def is_impassable(self, x, y):
        """
        Return whether physical coordinate (x,y) lies in an impassable region.
        """

        i = int(
            y / self.height * (self.resolution - 1)
        )

        j = int(
            x / self.width * (self.resolution - 1)
        )

        return self.impassable[i, j]
    
    def to_dataframe(self):
        """
        Convert terrain cost grid into a pandas DataFrame.

        Returns
        -------
        pandas.DataFrame
            Two-dimensional table containing terrain costs.
        """

        return pd.DataFrame(self.grid)
    
    def get_cost_at_coordinate(self, x, y):
        """
        Return terrain cost at physical coordinate (x,y).
        """

        i = int(
            y / self.height * (self.resolution - 1)
        )

        j = int(
            x / self.width * (self.resolution - 1)
        )

        return self.get_cost(i, j)