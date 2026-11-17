"""Smooth path retiming with conservative continuous derivative bounds."""
import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import CubicSpline
from scipy.interpolate import BPoly, PPoly
from scipy.optimize import brentq

# Time-blend derivatives through jerk vanish at both ends of each move.
BLEND = Polynomial([0, 0, 0, 0, 35, -84, 70, -20])


def polynomial_max(poly):
    roots = poly.deriv().roots()
    points = [0., 1.] + [r.real for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < 1]
    return float(np.max(np.abs(poly(points))))


def composed_bounds(spline):
    """Bound the composed trajectory derivatives using Bernstein convex hulls.

    Each spline interval is composed with the septic clock in local [0, 1]
    coordinates. Bernstein derivative coefficients bound the whole interval,
    avoiding the loose product of independent derivative maxima.
    """
    inverse_knots = np.array([0.] + [brentq(lambda u: BLEND(u) - s, 0., 1.)
                                      for s in spline.x[1:-1]] + [1.])
    bounds = np.zeros((2, spline.c.shape[-1]))
    for i, (start, end) in enumerate(zip(inverse_knots[:-1], inverse_knots[1:])):
        width = end - start
        h = BLEND(Polynomial([start, width])) - spline.x[i]
        for j in range(spline.c.shape[-1]):
            c = spline.c[:, i, j]
            composed = ((c[0] * h + c[1]) * h + c[2]) * h + c[3]
            for order in (1, 2):
                p = composed.deriv(order) / width**order
                bernstein = BPoly.from_power_basis(PPoly(p.coef[::-1, None], [0., 1.]))
                bounds[order - 1, j] = max(bounds[order - 1, j], np.abs(bernstein.c).max())
    return bounds


def retime(path, velocity_limit, acceleration_limit, dt, requested_duration=None,
           parameterization="uniform"):
    # Preserve the planner's uniform time parameter. Arc-length parameters
    # amplify float32 quantization near a stationary endpoint into huge curvature.
    if parameterization == "arc_length":
        distances = np.linalg.norm(np.diff(path, axis=0), axis=1)
        if np.any(distances < 1e-7):
            raise ValueError("Arc-length retiming needs distinct, non-stationary guide samples")
        s = np.r_[0., np.cumsum(distances)]
        s /= s[-1]
    elif parameterization == "uniform":
        s = np.linspace(0., 1., len(path))
    else:
        raise ValueError(f"Unknown path parameterization: {parameterization}")
    spline = CubicSpline(s, path, bc_type="clamped")
    velocity_bound, acceleration_bound = composed_bounds(spline)
    duration = 1.03 * max(np.max(velocity_bound / velocity_limit),
                          np.sqrt(np.max(acceleration_bound / acceleration_limit)), dt)
    n = int(np.ceil(duration / dt))
    if requested_duration is not None:
        requested_n = int(round(requested_duration / dt))
        if requested_n < n:
            raise ValueError(f"Move requires at least {n * dt:.2f}s; requested {requested_duration:.2f}s")
        n = requested_n
    duration = n * dt
    u = np.linspace(0., 1., n + 1)
    h = np.clip(BLEND(u), 0., 1.)
    hd, hdd = BLEND.deriv()(u)[:, None], BLEND.deriv(2)(u)[:, None]
    q = spline(h)
    qd = spline(h, 1) * hd / duration
    qdd = (spline(h, 2) * hd**2 + spline(h, 1) * hdd) / duration**2
    q[[0, -1]] = path[[0, -1]]
    qd[[0, -1]] = 0.
    qdd[[0, -1]] = 0.
    return q, qd, qdd, spline, duration
