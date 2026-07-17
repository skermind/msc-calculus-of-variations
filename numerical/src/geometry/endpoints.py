import pandas as pd

class Endpoints:
    """
    Defines fixed boundary conditions for a variational path problem.

    The curve gamma is constrained by:

        gamma(0) = start
        gamma(1) = end

    Parameters
    ----------
    start : tuple
        Starting coordinate (x, y).

    end : tuple
        Ending coordinate (x, y).

    Attributes
    ----------
    start : tuple
        Initial boundary point.

    end : tuple
        Final boundary point.
    """

    def __init__(self, start, end):
        self.start = start
        self.end = end
    
    def to_dataframe(self):
        """
        Convert boundary points into a pandas DataFrame.

        Returns
        -------
        pandas.DataFrame
            Table containing endpoint coordinates.
        """

        return pd.DataFrame({
            "point": [
                "start",
                "end"
            ],
            "x": [
                self.start[0],
                self.end[0]
            ],
            "y": [
                self.start[1],
                self.end[1]
            ]
        })