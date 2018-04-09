# This file is part of AFEM which provides an engineering toolkit for airframe
# finite element modeling during conceptual design.
#
# Copyright (C) 2016-2018  Laughlin Research, LLC (info@laughlinresearch.com)
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
from math import sqrt

from OCCT.Extrema import (Extrema_ExtCC, Extrema_ExtCS, Extrema_ExtPC,
                          Extrema_ExtPS, Extrema_ExtSS)

from afem.geometry.check import CheckGeom

__all__ = ["DistancePointToCurve", "DistancePointToSurface",
           "DistanceCurveToCurve", "DistanceCurveToSurface",
           "DistanceSurfaceToSurface"]


class DistancePointToCurve(object):
    """
    Calculate the extrema between a point and a curve.

    :param point_like pnt: The point.
    :param afem.geometry.entities.Curve crv: The curve.
    :param float tol: The tolerance.

    :raise RuntimeError: If ``Extrema_ExtPC`` fails.

    Usage:

    >>> from afem.geometry import *
    >>> c = NurbsCurveByPoints([(0. ,0., 0.), (10., 0., 0.)]).curve
    >>> p = Point(5., 1., 0.)
    >>> dist = DistancePointToCurve(p, c)
    >>> dist.nsol
    3
    """

    def __init__(self, pnt, crv, tol=1.0e-10):
        tool = Extrema_ExtPC(pnt, crv.adaptor, tol)

        if not tool.IsDone():
            msg = 'Extrema between point and curve failed.'
            raise RuntimeError(msg)

        self._nsol = tool.NbExt()
        results = []
        for i in range(1, self._nsol + 1):
            di = sqrt(tool.SquareDistance(i))
            ext_pnt = tool.Point(i)
            gp_pnt = ext_pnt.Value()
            ui = ext_pnt.Parameter()
            pi = CheckGeom.to_point(gp_pnt)
            results.append((di, ui, pi))

        results.sort(key=lambda tup: tup[0])

        self._dmin = results[0][0]
        self._dmax = results[-1][0]
        self._dist = [row[0] for row in results]
        self._prms = [row[1] for row in results]
        self._pnts = [row[2] for row in results]

    @property
    def nsol(self):
        """
        :return: The number of solutions.
        :rtype: int
        """
        return self._nsol

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._dmin

    @property
    def dmax(self):
        """
        :return: The maximum distance.
        :rtype: float
        """
        return self._dmax

    @property
    def distances(self):
        """
        :return: Sorted distances.
        :rtype: list(float)
        """
        return self._dist

    @property
    def parameters(self):
        """
        :return: Sorted parameters on curve.
        :rtype: list(float)
        """
        return self._prms

    @property
    def points(self):
        """
        :return: Sorted points on curve.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts


class DistancePointToSurface(object):
    """
    Calculate the extrema between a point and a surface.

    :param point_like pnt: The point.
    :param afem.geometry.entities.Surface srf: The surface.
    :param float tol: The tolerance.

    :raise RuntimeError: If ``Extrema_ExtPS`` fails.
    """

    def __init__(self, pnt, srf, tol=1.0e-10):
        tool = Extrema_ExtPS(pnt, srf.adaptor, tol, tol)

        if not tool.IsDone():
            msg = 'Extrema between point and surface failed.'
            raise RuntimeError(msg)

        self._nsol = tool.NbExt()
        results = []
        for i in range(1, self._nsol + 1):
            di = sqrt(tool.SquareDistance(i))
            ext_pnt = tool.Point(i)
            gp_pnt = ext_pnt.Value()
            ui, vi = ext_pnt.Parameter()
            pi = CheckGeom.to_point(gp_pnt)
            results.append((di, (ui, vi), pi))

        results.sort(key=lambda tup: tup[0])

        self._dmin = results[0][0]
        self._dmax = results[-1][0]
        self._dist = [row[0] for row in results]
        self._prms = [row[1] for row in results]
        self._pnts = [row[2] for row in results]

    @property
    def nsol(self):
        """
        :return: The number of solutions.
        :rtype: int
        """
        return self._nsol

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._dmin

    @property
    def dmax(self):
        """
        :return: The maximum distance.
        :rtype: float
        """
        return self._dmax

    @property
    def distances(self):
        """
        :return: Sorted distances.
        :rtype: list(float)
        """
        return self._dist

    @property
    def parameters(self):
        """
        :return: Sorted parameters on surface. This is a list of tuples
         containing the (u, v) locations.
        :rtype: list(tuple(float))
        """
        return self._prms

    @property
    def points(self):
        """
        :return: Sorted points on surface.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts


class DistanceCurveToCurve(object):
    """
    Calculate the extrema between two curves.

    :param afem.geometry.entities.Curve crv1: The first curve.
    :param afem.geometry.entities.Curve crv2: The second curve.
    :param float tol: The tolerance.

    :raise RuntimeError: If ``Extrema_ExtCC`` fails.
    """

    def __init__(self, crv1, crv2, tol=1.0e-10):
        tool = Extrema_ExtCC(crv1.adaptor, crv2.adaptor, tol, tol)

        if not tool.IsDone():
            msg = 'Extrema between two curves failed.'
            raise RuntimeError(msg)

        self._nsol = tool.NbExt()
        self._parallel = tool.IsParallel()
        results = []
        for i in range(1, self._nsol + 1):
            di = sqrt(tool.SquareDistance(i))
            gp_pnt1, gp_pnt2 = tool.Points(i)
            p1 = CheckGeom.to_point(gp_pnt1)
            p2 = CheckGeom.to_point(gp_pnt2)
            results.append((di, p1, p2))

        results.sort(key=lambda tup: tup[0])

        self._dmin = results[0][0]
        self._dmax = results[-1][0]
        self._dist = [row[0] for row in results]
        self._pnts1 = [row[1] for row in results]
        self._pnts2 = [row[2] for row in results]

    @property
    def nsol(self):
        """
        :return: The number of solutions.
        :rtype: int
        """
        return self._nsol

    @property
    def is_parallel(self):
        """
        :return: *True* if curves were parallel.
        :rtype: bool
        """
        return self._parallel

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._dmin

    @property
    def dmax(self):
        """
        :return: The maximum distance.
        :rtype: float
        """
        return self._dmax

    @property
    def distances(self):
        """
        :return: Sorted distances.
        :rtype: list(float)
        """
        return self._dist

    @property
    def points1(self):
        """
        :return: Sorted points on first curve.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts1

    @property
    def points2(self):
        """
        :return: Sorted points on second curve.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts2


class DistanceCurveToSurface(object):
    """
    Calculate the extrema between a curve and surface.

    :param afem.geometry.entities.Curve crv: The curve.
    :param afem.geometry.entities.Surface srf: surface.
    :param float tol: The tolerance.

    :raise RuntimeError: If the extrema algorithm fails.
    """

    def __init__(self, crv, srf, tol=1.0e-10):
        tool = Extrema_ExtCS(crv.adaptor, srf.adaptor, tol, tol)

        if not tool.IsDone():
            msg = 'Extrema between curve and surface failed.'
            raise RuntimeError(msg)

        self._nsol = tool.NbExt()
        self._parallel = tool.IsParallel()
        results = []
        for i in range(1, self._nsol + 1):
            di = sqrt(tool.SquareDistance(i))
            gp_pnt1, gp_pnt2 = tool.Points(i)
            p1 = CheckGeom.to_point(gp_pnt1)
            p2 = CheckGeom.to_point(gp_pnt2)
            results.append((di, p1, p2))

        results.sort(key=lambda tup: tup[0])

        self._dmin = results[0][0]
        self._dmax = results[-1][0]
        self._dist = [row[0] for row in results]
        self._pnts1 = [row[1] for row in results]
        self._pnts2 = [row[2] for row in results]

    @property
    def nsol(self):
        """
        :return: The number of solutions.
        :rtype: int
        """
        return self._nsol

    @property
    def is_parallel(self):
        """
        :return: *True* if the curve and surface were parallel.
        :rtype: bool
        """
        return self._parallel

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._dmin

    @property
    def dmax(self):
        """
        :return: The maximum distance.
        :rtype: float
        """
        return self._dmax

    @property
    def distances(self):
        """
        :return: Sorted distances.
        :rtype: list(float)
        """
        return self._dist

    @property
    def points1(self):
        """
        :return: Sorted points on the curve.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts1

    @property
    def points2(self):
        """
        :return: Sorted points on the surface.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts2


class DistanceSurfaceToSurface(object):
    """
    Calculate the extrema between two surfaces.

    :param afem.geometry.entities.Surface srf1: The first surface.
    :param afem.geometry.entities.Surface srf2: The second surface.
    :param float tol: The tolerance.

    :raise RuntimeError: If ``Extrema_ExtSS`` fails.
    """

    def __init__(self, srf1, srf2, tol=1.0e-10):
        tool = Extrema_ExtSS(srf1.adaptor, srf2.adaptor, tol, tol)

        if not tool.IsDone():
            msg = 'Extrema between curve and surface failed.'
            raise RuntimeError(msg)

        self._nsol = tool.NbExt()
        self._parallel = tool.IsParallel()
        results = []
        for i in range(1, self._nsol + 1):
            di = sqrt(tool.SquareDistance(i))
            gp_pnt1, gp_pnt2 = tool.Points(i)
            p1 = CheckGeom.to_point(gp_pnt1)
            p2 = CheckGeom.to_point(gp_pnt2)
            results.append((di, p1, p2))

        results.sort(key=lambda tup: tup[0])

        self._dmin = results[0][0]
        self._dmax = results[-1][0]
        self._dist = [row[0] for row in results]
        self._pnts1 = [row[1] for row in results]
        self._pnts2 = [row[2] for row in results]

    @property
    def nsol(self):
        """
        :return: The number of solutions.
        :rtype: int
        """
        return self._nsol

    @property
    def is_parallel(self):
        """
        :return: *True* if the curve and surface were parallel.
        :rtype: bool
        """
        return self._parallel

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._dmin

    @property
    def dmax(self):
        """
        :return: The maximum distance.
        :rtype: float
        """
        return self._dmax

    @property
    def distances(self):
        """
        :return: Sorted distances.
        :rtype: list(float)
        """
        return self._dist

    @property
    def points1(self):
        """
        :return: Sorted points on the first surface.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts1

    @property
    def points2(self):
        """
        :return: Sorted points on the second surface.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts2
