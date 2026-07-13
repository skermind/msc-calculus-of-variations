import numpy as np


class Path:
    """
    Discrete approximation of a curve gamma.
    The continuous curve:
        gamma : [0,1] -> Ω
    is approximated numerically by:
        gamma_h = {p0,p1,...,pN}
    where each point:
        p_i = (x_i,y_i)
    Parameters
    ----------
    points : array-like
        Ordered sequence of coordinates defining the path.
        Example:
        [
            (0,0),
            (1,2),
            (3,4)
        ]
    """

    def __init__(self, points):
        self.points = np.array(points)


    def length(self):
        """
        Calculate the total length of the discrete path.
        Approximates:
            L(gamma) = ∫ |gamma'(t)| dt
        using the sum:
            L(gamma_h) ≈ Σ |p_(i+1)-p_i|
        Returns
        -------
        float
            Total path length.
        """
        segments = np.diff(
            self.points,
            axis=0
        )

        lengths = np.linalg.norm(
            segments,
            axis=1
        )

        return np.sum(lengths)