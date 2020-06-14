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
from OCCT.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCCT.Extrema import Extrema_ExtPC
from OCCT.GeomAPI import GeomAPI_IntCS
from OCCT.GeomInt import GeomInt_IntSS
from OCCT.IntTools import IntTools_EdgeEdge
from OCCT.ShapeFix import ShapeFix_ShapeTolerance
from OCCT.TopAbs import TopAbs_VERTEX
from numpy import float64, inf, mean, sqrt, zeros
from scipy.spatial import KDTree

from afem.adaptor.entities import AdaptorCurve
from afem.geometry.check import CheckGeom
from afem.geometry.entities import Curve, Point

__all__ = ["CurveIntersector", "IntersectCurveCurve",
           "IntersectCurveSurface", "SurfaceIntersector",
           "IntersectSurfaceSurface"]


class CurveIntersector(object):
    """
    Base class for handling curve intersection methods and results.
    """

    def __init__(self, c1, c2):
        self._c1 = c1
        self._c2 = c2
        self._npts = 0
        self._results = []
        self._kdt = None

    def _set_results(self, npts, results):
        """
        Set curve intersection results.
        """
        if npts > 0:
            self._npts = npts
            # Replace ndarrays with point instances and build kd-tree.
            data = zeros((npts, 3), dtype=float64)
            for i in range(npts):
                data[i] = results[i][1]
            self._results = results
            self._kdt = KDTree(data)

    @property
    def npts(self):
        """
        :return: Number of intersection points.
        :rtype: int
        """
        return self._npts

    @property
    def success(self):
        """
        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        return self.npts > 0

    @property
    def points(self):
        """
        :return: List of intersection points.
        :rtype: list(afem.geometry.entities.Point)
        """
        if self._npts <= 0:
            return []
        return [results[1] for results in self._results]

    @property
    def parameters(self):
        """
        :return: List of intersection parameters. For a curve-curve
            intersection this will be a list of tuples containing the
            parameters of each curve [(u1, u2), (u1, u2), ...]. For a
            curve-surface intersection this will be a list of tuples
            containing the parameters for the surface and then the curve
            [(u, v, t), (u, v, t), ...].
        :rtype: list(tuple(float))
        """
        if self._npts <= 0:
            return []
        return [results[0] for results in self._results]

    def point(self, indx=1):
        """
        Return the point result by index.

        :param int indx: Index for point.

        :return: Intersection point.
        :rtype: afem.geometry.entities.Point
        """
        return self._results[indx - 1][1]

    def query_point(self, p0, distance_upper_bound=inf):
        """
        Find the intersection result nearest to the provided point.

        :param point_like p0: Point to search from.
        :param float distance_upper_bound: Return only results within this
            distance.

        :return: Distance to nearest intersection result and its index (d, i).
            Returns (None, None) if no results are available.
        :rtype: tuple
        """
        if not self.success:
            return None, None
        p0 = CheckGeom.to_point(p0)
        d, i = self._kdt.query(p0.xyz, 1, 0., 2, distance_upper_bound)
        return d, i


class IntersectCurveCurve(CurveIntersector):
    """
    Curve-curve intersection. This method converts the curves to edges and
    performs the intersections that way. This proved to be more robust than
    OpenCASCADE's native curve-curve intersection tool.

    :param afem.geometry.entities.Curve crv1: The first curve.
    :param afem.geometry.entities.Curve crv2: The second curve.
    :param float itol: The intersection tolerance.
    """

    def __init__(self, crv1, crv2, itol=1.0e-7):
        super(IntersectCurveCurve, self).__init__(crv1, crv2)

        # Build edges from curve
        e1 = BRepBuilderAPI_MakeEdge(crv1.object).Edge()
        e2 = BRepBuilderAPI_MakeEdge(crv2.object).Edge()

        # Set tolerance to be half intersection tolerance
        shp_tol = ShapeFix_ShapeTolerance()
        tol = itol / 2.
        shp_tol.SetTolerance(e1, tol)
        shp_tol.SetTolerance(e2, tol)

        # Perform edge-edge intersection
        cci = IntTools_EdgeEdge(e1, e2)
        cci.Perform()

        # Gather results of point intersection only
        results = []
        common_parts = cci.CommonParts()
        for i in range(1, common_parts.Length() + 1):
            common_part = common_parts.Value(i)
            if not common_part.Type() == TopAbs_VERTEX:
                continue
            u1 = common_part.VertexParameter1()
            u2 = common_part.VertexParameter2()
            p1 = crv1.eval(u1)
            p2 = crv2.eval(u2)
            pi = mean([p1, p2], axis=0)
            pi = Point(*pi)
            results.append([(u1, u2), pi])

        npts = len(results)
        self._set_results(npts, results)


class IntersectCurveSurface(CurveIntersector):
    """
    Curve-surface intersection.

    :param afem.geometry.entities.Curve crv: The curve.
    :param afem.geometry.entities.Surface srf: The surface.
    """

    def __init__(self, crv, srf):
        super(IntersectCurveSurface, self).__init__(crv, srf)

        # Perform

        # OCC intersection.
        csi = GeomAPI_IntCS(crv.object, srf.object)
        results = []
        for i in range(1, csi.NbPoints() + 1):
            u, v, t = csi.Parameters(i, 0., 0., 0.)
            pc = crv.eval(t)
            ps = srf.eval(u, v)
            pi = mean([pc, ps], axis=0)
            pi = Point(*pi)
            results.append([(t, u, v), pi])

        npts = len(results)
        self._set_results(npts, results)

    @property
    def curve_parameters(self):
        return [results[0][0] for results in self._results]

    @property
    def surface_parameters(self):
        return [results[0][1:] for results in self._results]


class SurfaceIntersector(object):
    """
    Base class for handling surface intersection methods and results.
    """

    def __init__(self):
        self._crvs = []

    @property
    def ncrvs(self):
        """
        :return: Number of intersection curves.
        :rtype: int
        """
        return len(self._crvs)

    @property
    def success(self):
        """
        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        return self.ncrvs > 0

    @property
    def curves(self):
        """
        :return: The intersection curves.
        :rtype: list(afem.geometry.entities.Curve)
        """
        return self._crvs

    def curve(self, indx=1):
        """
        Generate an intersection curve.

        :param int indx: Index of intersection curve.

        :return: Intersection curve.
        :rtype: afem.geometry.entities.Curve
        """
        return self._crvs[indx - 1]

    def curve_nearest_point(self, pnt):
        """
        Get the index of the intersection curve that is nearest to the given
        reference point.

        :param point_like pnt: Reference point.

        :return: Index of curve nearest point.
        :rtype: int
        """
        return _curve_nearest_point(pnt, self._crvs)


class IntersectSurfaceSurface(SurfaceIntersector):
    """
    Surface-surface intersection.

    :param afem.geometry.entities.Surface srf1: The first surface.
    :param afem.geometry.entities.Surface srf2: The second surface.
    :param float itol: Intersection tolerance.
    :param bool approx: Approximate intersection curves.
    """

    def __init__(self, srf1, srf2, itol=1.0e-7, approx=True):
        super(IntersectSurfaceSurface, self).__init__()

        # OCC intersect
        ssi = GeomInt_IntSS(srf1.object, srf2.object, itol,
                            approx, False, False)

        # Build curves
        ncrvs = ssi.NbLines()
        crvs = []
        for i in range(1, ncrvs + 1):
            hcrv = ssi.Line(i)
            crvs.append(Curve(hcrv))

        self._crvs = crvs
        self._tol3d = ssi.TolReached3d()

    @property
    def tol3d(self):
        """
        :return: Tolerance reached for 3-D intersection curves.
        :rtype: float
        """
        return self._tol3d


def _distance_point_to_curve(point, curve):
    """
    Find the minimum distance between a point and a curve.
    """
    # OCC extrema.
    adp_crv = AdaptorCurve.to_adaptor(curve)
    ext_pc = Extrema_ExtPC(point, adp_crv.object)
    if not ext_pc.IsDone():
        return None

    # Find the minimum result.
    n_ext = ext_pc.NbExt()
    for i in range(1, n_ext + 1):
        if ext_pc.IsMin(i):
            d = ext_pc.SquareDistance(i)
            return sqrt(d)

    return None


def _curve_nearest_point(point, curves):
    """
    Find the curve nearest to the point.
    """
    ncrvs = len(curves)
    if ncrvs == 0:
        return None
    if ncrvs == 1:
        return curves[0]

    cmin = curves[0]
    dmin = _distance_point_to_curve(point, cmin)
    for c in curves[1:]:
        di = _distance_point_to_curve(point, c)
        if di < dmin:
            dmin = di
            cmin = c

    return cmin
