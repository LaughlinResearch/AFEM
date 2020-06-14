# This file is part of AFEM which provides an engineering toolkit for airframe
# finite element modeling during conceptual design.
#
# Copyright (C) 2016-2018 Laughlin Research, LLC
# Copyright (C) 2019-2020 Trevor Laughlin
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
from __future__ import division, division

from OCCT.BSplCLib import BSplCLib
from numpy import array, diff, float64, floor, hstack, sqrt, sum, zeros
from numpy.linalg import norm


def local_to_global_param(a, b, *args):
    if args[0] is None:
        return None
    local_u = args
    is_list = True
    if len(local_u) == 1:
        is_list = False
    global_u = []
    for ui in local_u:
        if ui < 0.:
            ui = 0.
        if ui > 1.:
            ui = 1.
        global_u.append(a + ui * (b - a))
    if is_list:
        return global_u
    else:
        return global_u[0]


def global_to_local_param(a, b, *args):
    if args[0] is None:
        return None
    global_u = args
    is_list = True
    if len(global_u) == 1:
        is_list = False
    local_u = []
    for ui in global_u:
        u = (ui - a) / (b - a)
        if u < 0.:
            u = 0.
        if u > 1.:
            u = 1.
        local_u.append(u)
    if is_list:
        return local_u
    else:
        return local_u[0]


def homogenize_array1d(cp, w):
    _w = w.reshape(-1, 1)
    return hstack((cp * _w, _w))


def homogenize_array2d(cp, w):
    n, m, _ = cp.shape
    cpw = zeros((n, m, 4), dtype=float)
    for i in range(n):
        for j in range(m):
            cpw[i, j, :3] = cp[i, j] * w[i, j]
            cpw[i, j, 3] = w[i, j]
    return cpw


def dehomogenize_array1d(cpw):
    w = cpw[:, -1]
    cp = cpw[:, :-1] / cpw[:, -1].reshape(-1, 1)
    return cp, w


def dehomogenize_array2d(cpw):
    cp = cpw[:, :, :3]
    w = cpw[:, :, -1]
    n, m = cp.shape[0:2]
    for i in range(n):
        for j in range(m):
            cp[i, j] /= w[i, j]
    return cp, w


# def build_knot_vector_from_occ(tcol_knots, tcol_mult, p, is_periodic):
#     """
#     Build knot sequence from OCC data.
#     """
#     n = CLib.bsplclib_KnotSequenceLength(tcol_mult, p, is_periodic)
#     tcol_knots_seq = TColStd_Array1OfReal(1, n)
#     CLib.bsplclib_KnotSequence(tcol_knots, tcol_mult, p, is_periodic,
#                                tcol_knots_seq)
#     return to_np_from_tcolstd_array1_real(tcol_knots_seq)


def uniform_parameters(n, a=0., b=1.):
    """
    Generate uniform parameters.

    :param int n: Number of parameters.
    :param float a: Lower bound.
    :param float b: Upper bound.

    :return: Parameters between [a, b].
    :rtype: numpy.ndarray
    """
    u = zeros(n, dtype=float64)
    u[0] = a
    u[-1] = b
    for i in range(1, n - 1):
        u[i] = a + i * (b - a) / n
    return u


def chord_parameters(pnts, a=0., b=1.):
    """
    Generate parameters using chord length method.

    :param list(point_like) pnts: List of ordered points.
    :param float a: Lower bound.
    :param float b: Upper bound.

    :return: Parameters between [a, b].
    :rtype: numpy.ndarray
    """
    pnts = array(pnts, dtype=float64)
    n = len(pnts)
    dtotal = sum(norm(diff(pnts, axis=0), axis=1))
    u = zeros(n, dtype=float64)
    u[0] = a
    u[-1] = b
    if dtotal <= 0.:
        return u
    for i in range(1, n - 1):
        di = norm(pnts[i] - pnts[i - 1]) / dtotal
        u[i] = u[i - 1] + di
    return u


def centripetal_parameters(pnts, a=0., b=1.):
    """
    Generate parameters using centripetal method.

    :param list[(point_like) pnts: List of ordered points.
    :param float a: Lower domain.
    :param float b: Upper domain.

    :return: Parameters between [a, b].
    :rtype: numpy.ndarray
    """
    pnts = array(pnts, dtype=float64)
    n = len(pnts)
    dtotal = sum(norm(diff(pnts, axis=0), axis=1))
    u = zeros(n, dtype=float64)
    u[0] = a
    u[-1] = b
    if dtotal <= 0.:
        return u
    for i in range(1, n - 1):
        di = sqrt((norm(pnts[i] - pnts[i - 1]) / dtotal))
        u[i] = u[i - 1] + di
    return u


def reparameterize_knots(u1, u2, tcol_knots):
    """
    Reparameterize the knot values between *u1* and *u2*
    """
    BSplCLib.Reparametrize_(u1, u2, tcol_knots)


def find_span(n, p, u, uk):
    """
    Determine the knot span index.

    :param int n: Number of control points - 1.
    :param int p: Degree.
    :param float u: Parameter.
    :param ndarray uk: Knot vector.

    :return: Knot span.
    :rtype: int

    *Reference:* Algorithm A2.1 from "The NURBS Book".
    """
    # Special case
    if u >= uk[n + 1]:
        return n
    if u <= uk[p]:
        return p

    # Do binary search
    low = p
    high = n + 1
    mid = int(floor((low + high) / 2.))
    while u < uk[mid] or u >= uk[mid + 1]:
        if u < uk[mid]:
            high = mid
        else:
            low = mid
        mid = int(floor((low + high) / 2.))
    return mid


def basis_funs(i, u, p, uk):
    """
    Compute the non-vanishing basis functions.

    :param int i: Knot span index.
    :param float u: Parameter.
    :param int p: Degree.
    :param ndarray uk: Knot vector.

    :return: Non-vanishing basis functions.
    :rtype: ndarray

    Reference: Algorithm A2.2 from "The NURBS Book"
    """
    bf = [0.0] * (p + 1)
    bf[0] = 1.0
    left = [0.0] * (p + 1)
    right = [0.0] * (p + 1)
    for j in range(1, p + 1):
        left[j] = u - uk[i + 1 - j]
        right[j] = uk[i + j] - u
        saved = 0.0
        for r in range(0, j):
            temp = bf[r] / (right[r + 1] + left[j - r])
            bf[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        bf[j] = saved
    return array(bf, dtype=float)
