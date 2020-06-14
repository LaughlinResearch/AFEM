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
from math import ceil, radians

from OCCT.Approx import Approx_ChordLength, Approx_IsoParametric
from OCCT.BSplCLib import BSplCLib
from OCCT.GC import GC_MakeCircle
from OCCT.GCPnts import GCPnts_AbscissaPoint, GCPnts_UniformAbscissa
from OCCT.Geom import Geom_BSplineSurface, Geom_Circle, Geom_Line, Geom_Plane
from OCCT.Geom2dAPI import Geom2dAPI_Interpolate, Geom2dAPI_PointsToBSpline
from OCCT.GeomAPI import (GeomAPI_IntCS, GeomAPI_Interpolate,
                          GeomAPI_PointsToBSpline)
from OCCT.GeomFill import (GeomFill_AppSurf, GeomFill_Line,
                           GeomFill_SectionGenerator)
from OCCT.GeomPlate import GeomPlate_BuildAveragePlane
from OCCT.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal
from OCCT.TColgp import TColgp_Array1OfPnt
from OCCT.gce import gce_MakeCirc
from OCCT.gp import gp_Ax3, gp_Pln, gp_Quaternion, gp_Trsf
from OCCT.gp import gp_Extrinsic_XYZ
from numpy import array, cross, mean, zeros
from numpy.linalg import norm
from scipy.linalg import lu_factor, lu_solve

from afem.adaptor.entities import AdaptorCurve
from afem.config import logger
from afem.geometry import utils as geom_utils
from afem.geometry.check import CheckGeom
from afem.geometry.entities import (Direction, Vector, Point, Line, Circle,
                                    Plane, NurbsCurve, Geometry, NurbsCurve2D,
                                    Curve, TrimmedCurve, Axis3,
                                    NurbsSurface)
from afem.geometry.project import ProjectPointToCurve, ProjectPointToSurface
from afem.occ import utils as occ_utils

__all__ = ["PointByXYZ", "PointByArray",
           "PointFromParameter", "PointsAlongCurveByNumber",
           "PointsAlongCurveByDistance", "DirectionByXYZ", "DirectionByArray",
           "DirectionByPoints", "VectorByXYZ", "VectorByArray",
           "VectorByPoints", "LineByVector", "LineByPoints", "CircleByNormal",
           "CircleByPlane", "CircleBy3Points",
           "NurbsCurve2DByInterp", "NurbsCurve2DByApprox",
           "NurbsCurve2DByPoints",
           "NurbsCurveByInterp", "NurbsCurveByApprox",
           "NurbsCurveByPoints",
           "TrimmedCurveByPoints",
           "PlaneByNormal", "PlaneByAxes", "PlaneByPoints", "PlaneByApprox",
           "PlaneFromParameter", "PlaneByOrientation",
           "PlaneByCurveAndSurface",
           "PlanesAlongCurveByNumber",
           "PlanesAlongCurveByDistance", "PlanesBetweenPlanesByNumber",
           "PlanesBetweenPlanesByDistance",
           "PlanesAlongCurveAndSurfaceByDistance",
           "NurbsSurfaceByInterp", "NurbsSurfaceByApprox"]


# POINT -----------------------------------------------------------------------

class PointByXYZ(object):
    """
    Create a point by x, y, and z location.

    :param float x: x-location.
    :param float y: y-location.
    :param float z: z-location.
    """

    def __init__(self, x=0., y=0., z=0.):
        self._p = Point(x, y, z)

    @property
    def point(self):
        """
        :return: The created point.
        :rtype: afem.geometry.entities.Point
        """
        return self._p


class PointByArray(PointByXYZ):
    """
    Create a point from an array.

    :param array_like xyz: Array describing point location.

    :raise ValueError: If the length of the array is not equal to three.
    """

    def __init__(self, xyz=(0., 0., 0.)):
        if len(xyz) != 3:
            raise ValueError('Invalid array size.')
        super(PointByArray, self).__init__(*xyz)


class PointFromParameter(object):
    """
    Create a point along a curve at a specified distance from a parameter.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float u0: The initial parameter.
    :param float ds: The distance along the curve from the given parameter.
    :param float tol: Tolerance.
    """

    def __init__(self, c, u0, ds, tol=1.0e-7):
        adp_curve = AdaptorCurve.to_adaptor(c)

        tool = GCPnts_AbscissaPoint(tol, adp_curve.object, ds, u0)
        if not tool.IsDone():
            msg = 'GCPnts_AbscissaPoint failed in PointFromParameter.'
            logger.warning(msg)

        self._is_done = tool.IsDone()
        self._u, self._p = None, None
        if self._is_done:
            u = tool.Parameter()
            self._u = u
            p = adp_curve.eval(u)
            self._p = p

    @property
    def is_done(self):
        """
        :return: *True* if done, *False* otherwise.
        :rtype: bool
        """
        return self._is_done

    @property
    def point(self):
        """
        :return: The created point.
        :rtype: afem.geometry.entities.Point
        """
        return self._p

    @property
    def parameter(self):
        """
        :return: The parameter on the curve.
        :rtype: float
        """
        return self._u


class PointsAlongCurveByNumber(object):
    """
    Create a specified number of points along a curve. The points will be
    equidistant.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param int n: Number of points to create.
    :param float u1: The parameter of the first point (default=*c.u1*).
    :param float u2: The parameter of the last point (default=*c.u2*).
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param float tol: Tolerance.

    :raise RuntimeError: If ``GCPnts_UniformAbscissa`` fails.
    """

    def __init__(self, c, n, u1=None, u2=None, d1=None, d2=None, tol=1.0e-7):
        n = int(n)
        adp_crv = AdaptorCurve.to_adaptor(c)

        # Set u1 and u2
        if u1 is None:
            u1 = adp_crv.u1
        if u2 is None:
            u2 = adp_crv.u2

        # Adjust u1 and u2 if d1 or d2 != 0
        if d1 is not None:
            tool = PointFromParameter(adp_crv, u1, d1, tol)
            if tool.is_done:
                u1 = tool.parameter
        if d2 is not None:
            tool = PointFromParameter(adp_crv, u2, d2, tol)
            if tool.is_done:
                u2 = tool.parameter

        # Create uniform abscissa
        tool = GCPnts_UniformAbscissa(adp_crv.object, n, u1, u2, tol)
        if not tool.IsDone():
            msg = 'GCPnts_UniformAbscissa failed in PointsAlongCurveByNumber.'
            logger.warning(msg)

        # Gather results
        self._is_done = tool.IsDone()
        self._npts = 0
        self._prms = []
        self._pnts = []
        self._ds = None

        if self._is_done:
            self._npts = tool.NbPoints()
            for i in range(1, self._npts + 1):
                u = tool.Parameter(i)
                p = adp_crv.eval(u)
                self._pnts.append(p)
                self._prms.append(u)

        # Point spacing
        self._ds = None
        if self._npts > 1:
            ds = self._pnts[0].distance(self._pnts[1])
            self._ds = ds

    @property
    def npts(self):
        """
        :return: The number of points.
        :rtype: int
        """
        return self._npts

    @property
    def points(self):
        """
        :return: The points.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts

    @property
    def parameters(self):
        """
        :return: The parameters.
        :rtype: list(float)
        """
        return self._prms

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_points(self):
        """
        :return: The points between the first and last points.
        :rtype: list(afem.geometry.entities.Point)
        """
        if self.npts < 3:
            return []
        return self._pnts[1:-1]


class PointsAlongCurveByDistance(object):
    """
    Create points along a curve by distance between points. The points will
    be equidistant. This method calculates the number of points given the
    curve length and then uses :class:`.PointsAlongCurveByNumber`.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float maxd: The maximum allowed spacing between points. The
        actual spacing will be adjusted to not to exceed this value.
    :param float u1: The parameter of the first point (default=c.u1).
    :param float u2: The parameter of the last point (default=c.u2).
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of points to create.
    :param float tol: Tolerance.

    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, c, maxd, u1=None, u2=None, d1=None, d2=None, nmin=0,
                 tol=1.0e-7):
        maxd = float(maxd)
        nmin = int(nmin)

        adp_crv = AdaptorCurve.to_adaptor(c)

        # Set u1 and u2
        if u1 is None:
            u1 = adp_crv.u1
        if u2 is None:
            u2 = adp_crv.u2

        # Adjust u1 and u2 if d1 or d2 != 0
        if d1 is not None:
            tool = PointFromParameter(adp_crv, u1, d1, tol)
            if tool.is_done:
                u1 = tool.parameter
        if d2 is not None:
            tool = PointFromParameter(adp_crv, u2, d2, tol)
            if tool.is_done:
                u2 = tool.parameter

        # Determine number of points
        arc_length = adp_crv.arc_length(u1, u2, tol)
        n = ceil(arc_length / maxd) + 1
        if n < nmin:
            n = nmin

        # Create uniform abscissa
        ua = GCPnts_UniformAbscissa(adp_crv.object, int(n), u1, u2, tol)
        if not ua.IsDone():
            msg = "GCPnts_UniformAbscissa failed."
            raise RuntimeError(msg)

        # Gather results
        npts = ua.NbPoints()
        pnts = []
        prms = []
        for i in range(1, npts + 1):
            u = ua.Parameter(i)
            p = adp_crv.eval(u)
            pnts.append(p)
            prms.append(u)
        self._npts = npts
        self._prms = prms
        self._pnts = pnts

        # Point spacing
        self._ds = None
        if npts > 1:
            ds = pnts[0].distance(pnts[1])
            self._ds = ds

    @property
    def npts(self):
        """
        :return: The number of points.
        :rtype: int
        """
        return self._npts

    @property
    def points(self):
        """
        :return: The points.
        :rtype: list(afem.geometry.entities.Point)
        """
        return self._pnts

    @property
    def parameters(self):
        """
        :return: The parameters.
        :rtype: list(float)
        """
        return self._prms

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_points(self):
        """
        :return: The points between the first and last points.
        :rtype: list(afem.geometry.entities.Point)
        """
        if self.npts < 3:
            return []
        return self._pnts[1:-1]


# DIRECTION -------------------------------------------------------------------

class DirectionByXYZ(object):
    """
    Create a direction (i.e., unit vector) by x-, y-, and z-components.

    :param float x: x-component.
    :param float y: y-component.
    :param float z: z-component.
    """

    def __init__(self, x=1., y=0., z=0.):
        d = Direction(x, y, z)
        self._d = d

    @property
    def direction(self):
        """
        :return: The direction.
        :rtype: afem.geometry.entities.Direction
        """
        return self._d


class DirectionByArray(DirectionByXYZ):
    """
    Create a direction (i.e., unit vector) from an array-like object.

    :param array_like xyz: Array-like object defining xyz-components.

    :raise ValueError: If `len(xyz) != 3`.
    """

    def __init__(self, xyz=(1., 0., 0.)):
        if len(xyz) != 3:
            msg = 'Invalid array size.'
            raise ValueError(msg)
        super(DirectionByArray, self).__init__(*xyz)


class DirectionByPoints(DirectionByArray):
    """
    Create a direction (i.e., unit vector) between two points.

    :param point_like p1: The first point.
    :param point_like p2: The last point.
    """

    def __init__(self, p1, p2):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        super(DirectionByPoints, self).__init__(p2 - p1)


# VECTOR ----------------------------------------------------------------------

class VectorByXYZ(object):
    """
    Create a vector by x-, y-, and z-components.

    :param float x: x-component.
    :param float y: y-component.
    :param float z: z-component.
    """

    def __init__(self, x=1., y=0., z=0.):
        v = Vector(x, y, z)
        self._v = v

    @property
    def vector(self):
        """
        :return: The vector.
        :rtype: afem.geometry.entities.Vector
        """
        return self._v


class VectorByArray(VectorByXYZ):
    """
    Create a vector from an array-like object.

    :param array_like xyz: Array-like object defining xyz-components.

    :raise ValueError: If `len(xyz) != 3`.
    """

    def __init__(self, xyz=(1., 0., 0.)):
        if len(xyz) != 3:
            msg = 'Invalid array size.'
            raise ValueError(msg)
        super(VectorByArray, self).__init__(*xyz)


class VectorByPoints(object):
    """
    Create a vector between two points.

    :param point_like p1: The first point.
    :param point_like p2: The last point.

    :raise TypeError: If *p1* or *p2* cannot be converted to a :class:`.Point`.
    """

    def __init__(self, p1, p2):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        if None in [p1, p2]:
            msg = 'Invalid points provided.'
            raise TypeError(msg)

        v = Vector(p1, p2)
        self._v = v

    @property
    def vector(self):
        """
        :return: The vector.
        :rtype: afem.geometry.entities.Vector
        """
        return self._v


# LINE ------------------------------------------------------------------------

class LineByVector(object):
    """
    Create a line by an origin and a vector.

    :param point_like p: Origin of line.
    :param vector_like d: Direction of line.

    :raise TypeError: If *p* cannot be converted to a :class:`.Point`
    :raise TypeError: If *d* cannot be converted to a :class:`.Direction`
    """

    def __init__(self, p, d):
        p = CheckGeom.to_point(p)
        d = CheckGeom.to_direction(d)
        if not isinstance(p, Point):
            msg = "Invalid point."
            raise TypeError(msg)
        if not isinstance(d, Direction):
            msg = "Invalid direction."
            raise TypeError(msg)

        self._line = Line(Geom_Line(p, d))

    @property
    def line(self):
        """
        :return: The line.
        :rtype: afem.geometry.entities.Line
        """
        return self._line


class LineByPoints(object):
    """
    Create a line through two points.

    :param point_like p1: The first point.
    :param point_like p2: The last point.

    :raise TypeError: If *p1* or *p2* cannot be converted to a :class:`.Point`.
    """

    def __init__(self, p1, p2):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        if not isinstance(p1, Point):
            msg = "Invalid first point."
            raise TypeError(msg)
        if not isinstance(p2, Point):
            msg = "Invalid second point."
            raise TypeError(msg)

        d = DirectionByArray(p2 - p1).direction

        self._line = Line(Geom_Line(p1, d))

    @property
    def line(self):
        """
        :return: The line.
        :rtype: afem.geometry.entities.Line
        """
        return self._line


# CIRCLE ----------------------------------------------------------------------

class CircleByNormal(object):
    """
    Create a circle using a center, normal, and radius.

    :param point_like center: The center point.
    :param vector_like normal: The normal of the plane.
    :param float radius: The radius.
    """

    def __init__(self, center, normal, radius):
        center = CheckGeom.to_point(center)
        normal = CheckGeom.to_direction(normal)
        if not isinstance(center, Point):
            msg = "Invalid center point."
            raise TypeError(msg)
        if not isinstance(normal, Direction):
            msg = "Invalid normal direction."
            raise TypeError(msg)

        gp_circ = gce_MakeCirc(center, normal, radius).Value()

        self._circle = Circle(Geom_Circle(gp_circ))

    @property
    def circle(self):
        """
        :return: The circle.
        :rtype: afem.geometry.entities.Circle
        """
        return self._circle


class CircleByPlane(object):
    """
    Create a circle using a center, plane, and radius.

    :param point_like center: The center point.
    :param afem.geometry.entities.Plane plane: The plane.
    :param float radius: The radius.
    """

    def __init__(self, center, plane, radius):
        center = CheckGeom.to_point(center)
        if not isinstance(center, Point):
            msg = "Invalid center point."
            raise TypeError(msg)
        if not isinstance(plane, Plane):
            msg = "Invalid plane."
            raise TypeError(msg)

        gp_circ = gce_MakeCirc(center, plane.gp_pln, radius).Value()

        self._circle = Circle(Geom_Circle(gp_circ))

    @property
    def circle(self):
        """
        :return: The circle.
        :rtype: afem.geometry.entities.Circle
        """
        return self._circle


class CircleBy3Points(object):
    """
    Create a circle using three points.

    :param point_like p1: The first point.
    :param point_like p2: The second point.
    :param point_like p3: The third point.
    """

    def __init__(self, p1, p2, p3):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        p3 = CheckGeom.to_point(p3)

        geom_circ = GC_MakeCircle(p1, p2, p3).Value()
        self._circle = Circle(geom_circ)

    @property
    def circle(self):
        """
        :return: The circle.
        :rtype: afem.geometry.entities.Circle
        """
        return self._circle


# NURBSCURVE ------------------------------------------------------------------

class NurbsCurve2DByInterp(object):
    """
    Create a 2-D cubic curve by interpolating 2-D points.

    :param collections.Sequence(point2d_like) qp: Points to interpolate.
    :param bool is_periodic: Flag for curve periodicity. If *True* the curve
        will be periodic and closed.
    :param vector2d_like v1: Tangent to match at first point.
    :param vector2d_like v2: Tangent to match at last point.
    :param float tol: Tolerance used to check for coincident points and the
        magnitude of end vectors.

    :raise RuntimeError: If ``Geom2dAPI_Interpolate`` fails.
    """

    def __init__(self, qp, is_periodic=False, v1=None, v2=None, tol=1.0e-7):
        tcol_hpnts = occ_utils.to_tcolgp_harray1_pnt2d(qp)
        interp = Geom2dAPI_Interpolate(tcol_hpnts,
                                       is_periodic, tol)

        if None not in [v1, v2]:
            v1 = CheckGeom.to_vector2d(v1)
            v2 = CheckGeom.to_vector2d(v2)
            if v1 and v2:
                interp.Load(v1, v2)

        interp.Perform()
        if not interp.IsDone():
            msg = "Geom2dAPI_Interpolate failed."
            raise RuntimeError(msg)

        self._c = NurbsCurve2D(interp.Curve())

    @property
    def curve(self):
        """
        :return: The 2-D NURBS curve.
        :rtype: afem.geometry.entities.NurbsCurve2D
        """
        return self._c


class NurbsCurve2DByApprox(object):
    """
    Create a 2-D NURBS curve by approximating 2-D points.

    :param collections.Sequence(point2d_like) qp: Points to approximate.
    :param int dmin: Minimum degree.
    :param int dmax: Maximum degree.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity: Desired continuity of curve.
    :param OCCT.Approx.Approx_ParametrizationType parm_type: Parametrization
        type.
    :param float tol: The tolerance used for approximation. The distance
        from the points to the resulting curve should be lower than *tol*.

    :raise RuntimeError: If OCC method fails to interpolate the points with
        a curve.
    """

    def __init__(self, qp, dmin=3, dmax=8, continuity=Geometry.C2,
                 parm_type=Approx_ChordLength, tol=1.0e-6):
        dmin = int(dmin)
        dmax = int(dmax)
        tcol_pnts = occ_utils.to_tcolgp_array1_pnt2d(qp)

        fit = Geom2dAPI_PointsToBSpline(tcol_pnts, parm_type, dmin,
                                        dmax, continuity, tol)
        if not fit.IsDone():
            msg = "Geom2dAPI_PointsToBSpline failed."
            raise RuntimeError(msg)

        self._c = NurbsCurve2D(fit.Curve())

    @property
    def curve(self):
        """
        :return: The 2-D NURBS curve.
        :rtype: afem.geometry.entities.NurbsCurve2D
        """
        return self._c


class NurbsCurve2DByPoints(NurbsCurve2DByApprox):
    """
    Create a 2-D linear curve (i.e., a polyline) between points. This method
    uses :class:`.NurbsCurve2DByApprox` to fit a linear curve.

    :param collections.Sequence(point2d_like) qp: Points.
    """

    def __init__(self, qp):
        super(NurbsCurve2DByPoints, self).__init__(qp, 1, 1, Geometry.C0)


class NurbsCurveByInterp(object):
    """
    Create a cubic curve by interpolating points.

    :param collections.Sequence(point_like) qp: Points to interpolate.
    :param bool is_periodic: Flag for curve periodicity. If *True* the curve
        will be periodic and closed.
    :param vector_like v1: Tangent to match at first point.
    :param vector_like v2: Tangent to match at last point.
    :param float tol: Tolerance used to check for coincident points and the
        magnitude of end vectors.

    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, qp, is_periodic=False, v1=None, v2=None, tol=1.0e-7):
        tcol_hpnts = occ_utils.to_tcolgp_harray1_pnt(qp)
        interp = GeomAPI_Interpolate(tcol_hpnts,
                                     is_periodic, tol)

        if None not in [v1, v2]:
            v1 = CheckGeom.to_vector(v1)
            v2 = CheckGeom.to_vector(v2)
            if v1 and v2:
                interp.Load(v1, v2)

        interp.Perform()
        if not interp.IsDone():
            msg = "GeomAPI_Interpolate failed."
            raise RuntimeError(msg)

        self._c = NurbsCurve(interp.Curve())

    @property
    def curve(self):
        """
        :return: The NURBS curve.
        :rtype: afem.geometry.entities.NurbsCurve
        """
        return self._c


class NurbsCurveByApprox(object):
    """
    Create a NURBS curve by approximating points.

    :param collections.Sequence(point_like) qp: Points to approximate.
    :param int dmin: Minimum degree.
    :param int dmax: Maximum degree.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity: Desired continuity of curve.
    :param OCCT.Approx.Approx_ParametrizationType parm_type: Parametrization
        type.
    :param float tol: The tolerance used for approximation. The distance
        from the points to the resulting curve should be lower than *tol*.

    :raise RuntimeError: If OCC method fails to interpolate the points with
        a curve.
    """

    def __init__(self, qp, dmin=3, dmax=8, continuity=Geometry.C2,
                 parm_type=Approx_ChordLength, tol=1.0e-3):
        dmin = int(dmin)
        dmax = int(dmax)
        tcol_pnts = occ_utils.to_tcolgp_array1_pnt(qp)

        fit = GeomAPI_PointsToBSpline(tcol_pnts, parm_type, dmin,
                                      dmax, continuity, tol)
        if not fit.IsDone():
            msg = "GeomAPI_PointsToBSpline failed."
            raise RuntimeError(msg)

        self._c = NurbsCurve(fit.Curve())

    @property
    def curve(self):
        """
        :return: The NURBS curve.
        :rtype: afem.geometry.entities.NurbsCurve
        """
        return self._c


class NurbsCurveByPoints(NurbsCurveByApprox):
    """
    Create a linear curve (i.e., a polyline) between points. This method uses
    :class:`.NurbsCurveByApprox` to fit a linear curve.

    :param collections.Sequence(point_like) qp: Points.
    """

    def __init__(self, qp):
        super(NurbsCurveByPoints, self).__init__(qp, 1, 1, Geometry.C0)


# TRIMMED CURVE ---------------------------------------------------------------

class TrimmedCurveByPoints(object):
    """
    Create a trimmed curve using a basis curve and limiting points. The
    points are projected to the basis curve to find the limiting parameters.

    :param afem.geometry.entities.Curve basis_curve: The basis curve.
    :param point_like p1: The first point.
    :param point_like p2: The last point.
    :param bool sense: If the basis curve is periodic, the trimmed curve
        will have the same orientation as the basis curve if ``True`` or
        opposite if ``False``.
    :param bool adjust_periodic: If the basis curve is periodic, the bounds
        of the trimmed curve may be different from *u1* and *u2* if
        ``True``.

    :raise TypeError: If the basis curve is not a valid curve type.
    :raise ValueError: If *u1* >= *u2*.
    """

    def __init__(self, basis_curve, p1, p2, sense=True,
                 adjust_periodic=True):
        if not isinstance(basis_curve, Curve):
            raise TypeError('Invalid type of basis curve.')

        u1 = ProjectPointToCurve(p1, basis_curve).nearest_param
        u2 = ProjectPointToCurve(p2, basis_curve).nearest_param

        self._c = TrimmedCurve.by_parameters(basis_curve, u1, u2, sense,
                                             adjust_periodic)

    @property
    def curve(self):
        """
        :return: The trimmed curve.
        :rtype: afem.geometry.entities.TrimmedCurve
        """
        return self._c


# PLANE -----------------------------------------------------------------------

class PlaneByNormal(object):
    """
    Create a plane by an origin and a normal vector.

    :param point_like origin: The origin.
    :param vector_like vnorm: The normal vector.

    :raise TypeError: If *origin* cannot be converted to :class:`.Point`.
    :raise TypeError: If *vnorm* cannot be converted to :class:`.Direction`.
    """

    def __init__(self, origin=(0., 0., 0.), vnorm=(0., 0., 1.)):
        p0 = CheckGeom.to_point(origin)
        vn = CheckGeom.to_direction(vnorm)
        if not isinstance(p0, Point):
            msg = 'Invalid point.'
            raise TypeError(msg)
        if not isinstance(vn, Direction):
            msg = 'Invalid normal vector.'
            raise TypeError(msg)

        self._pln = Plane(Geom_Plane(p0, vn))

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneByAxes(object):
    """
    Create a plane by an origin and basic axes.

    :param point_like origin: The origin.
    :param str axes: The axes ('xy', 'xz', 'yz').

    :raise TypeError: If *origin* cannot be converted to :class:`.Point`.
    :raise ValueError: If *axes* is not a supported option.
    """

    def __init__(self, origin=(0., 0., 0.), axes='xz'):
        origin = CheckGeom.to_point(origin)
        axes = axes.lower()
        if not isinstance(origin, Point):
            msg = 'Invalid point.'
            raise TypeError(msg)
        if axes not in ['xy', 'yx', 'xz', 'zx', 'yz', 'zy']:
            msg = 'Invalid axes.'
            raise ValueError(msg)

        if axes in ['xy', 'yx']:
            vx = Direction(1., 0., 0.)
            n = Direction(0., 0., 1.)
        elif axes in ['yz', 'zy']:
            vx = Direction(0., 1., 0.)
            n = Direction(1., 0., 0.)
        elif axes in ['xz', 'zx']:
            vx = Direction(1., 0., 0.)
            n = Direction(0., -1., 0.)
        else:
            msg = 'Unknown axes.'
            raise ValueError(msg)

        ax3 = gp_Ax3(origin, n, vx)
        pln = Geom_Plane(ax3)

        self._pln = Plane(pln)

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneByPoints(object):
    """
    Create a plane by three points. The points must not be collinear. The
    plane will be defined by (*p2* - *p1*) x (*p3* - *p1*).

    :param point_like p1: First point. This point will be used as the origin.
    :param point_like p2: Second point.
    :param point_like p3: Third point.

    :raise TypeError: if *p1*, *p2*, or *p3* cannot be converted to a
        :class:`.Point`.
    :raise ValueError: If the points are collinear.
    """

    def __init__(self, p1, p2, p3):
        p1 = CheckGeom.to_point(p1)
        if not isinstance(p1, Point):
            msg = 'Invalid point 1.'
            raise TypeError(msg)
        p2 = CheckGeom.to_point(p2)
        if not isinstance(p2, Point):
            msg = 'Invalid point 2.'
            raise TypeError(msg)
        p3 = CheckGeom.to_point(p3)
        if not isinstance(p3, Point):
            msg = 'Invalid point 3.'
            raise TypeError(msg)

        vx = p2 - p1
        v31 = p3 - p1
        vn = cross(vx, v31)
        if norm(vn) <= 1.0e-12:
            msg = 'Points are collinear.'
            raise ValueError(msg)

        n = Direction(*vn)
        vx = Direction(*vx)
        ax = Axis3(p1, n, vx)

        self._pln = Plane(Geom_Plane(gp_Pln(ax)))

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneByApprox(object):
    """
    Create a plane by fitting points. The points must not be collinear.

    :param list(point_like) pnts: Points to fit plane. Should not be collinear.
    :param float tol: Tolerance used to check for collinear points.

    :raise ValueError: If the number of points is less than three.
    :raise RuntimeError: If a plane cannot be fit to the points.
    """

    def __init__(self, pnts, tol=1.0e-7):
        _pnts = [CheckGeom.to_point(p) for p in pnts if
                 CheckGeom.is_point_like(p)]
        if len(_pnts) < 3:
            msg = "Need at least three points to fit a plane."
            raise ValueError(msg)

        tcol_pnts = occ_utils.to_tcolgp_harray1_pnt(pnts)
        avg_pln = GeomPlate_BuildAveragePlane(tcol_pnts,
                                              tcol_pnts.Length(),
                                              tol, 1, 1)
        if not avg_pln.IsPlane():
            msg = ('Failed to create a plane by fitting points. Check for '
                   'collinear points.')
            raise RuntimeError(msg)

        # Move to centroid.
        gp_pln = avg_pln.Plane().Pln()
        pcg = Point(*mean(pnts, axis=0))
        gp_pln.SetLocation(pcg)

        self._pln = Plane(Geom_Plane(gp_pln))

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneFromParameter(object):
    """
    Create a plane along a curve at a specified distance from a parameter.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float u0: The initial parameter.
    :param float ds: The distance along the curve from the given parameter.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of the new plane. If no plane is
        provided, then the first derivative of the curve will define the
        plane normal.
    :param float tol: Tolerance.

    :return: The plane.
    :rtype: afem.geometry.entities.Plane
    """

    def __init__(self, c, u0, ds, ref_pln=None, tol=1.0e-7):
        adp_curve = AdaptorCurve.to_adaptor(c)

        tool = PointFromParameter(adp_curve, u0, ds, tol)

        u = tool.parameter
        self._u = u
        p = adp_curve.eval(u)
        if isinstance(ref_pln, Plane):
            gp_pln = ref_pln.gp_pln
            ax1 = gp_pln.Axis()
            v = ax1.Direction()
        else:
            v = adp_curve.deriv(u, 1)
        self._pln = PlaneByNormal(p, v).plane

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln

    @property
    def parameter(self):
        """
        :return: The parameter on the curve.
        :rtype: float
        """
        return self._u


class PlaneByOrientation(object):
    """
    Create a plane by rotation angles.

    :param point_like origin: The origin.
    :param str axes: The reference axes ('xy', 'xz', 'yz').
    :param float alpha: Rotation in degrees about global x-axis.
    :param float beta: Rotation in degrees about global y-axis.
    :param float gamma: Rotation in degrees about global z-axis.
    """

    def __init__(self, origin=(0., 0., 0.), axes='xz', alpha=0., beta=0.,
                 gamma=0.):
        pln = PlaneByAxes(axes=axes).plane

        # Build a quaternion for rotation angles
        r = gp_Quaternion()
        r.SetEulerAngles(gp_Extrinsic_XYZ, radians(alpha), radians(beta),
                         radians(gamma))

        # Build transformation matrix and rotate about global origin and
        # translate to desired plane origin.
        tf = gp_Trsf()
        v = CheckGeom.to_vector(origin)

        tf.SetTransformation(r, v)
        pln.object.Transform(tf)

        self._pln = pln

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneByCurveAndSurface(object):
    """
    Create a plane using a curve that lies on a surface. The x-axis of the
    plane is found by the cross product of the surface normal and the first
    derivative of the curve at the parameter. The normal of the plane is
    found by the cross product of the x-axis and the surface normal. The
    origin will be located at a point along the curve at the given parameter.

    :param afem.geometry.entities.Curve crv: The curve.
    :param afem.geometry.entities.Surface srf: The surface.
    :param float u: The curve parameter.
    """

    def __init__(self, crv, srf, u):
        origin = crv.eval(u)
        u, v = ProjectPointToSurface(origin, srf).nearest_param
        srf_nrm = srf.norm(u, v)
        crv_deriv = crv.deriv(u, 1)
        vx = Direction(srf_nrm.Crossed(crv_deriv))
        n = Direction(crv_deriv)
        ax3 = gp_Ax3(origin, n, vx)
        self._pln = Plane(Geom_Plane(ax3))

    @property
    def plane(self):
        """
        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlanesAlongCurveByNumber(object):
    """
    Create planes along a curve using a specified number. The origin of the
    planes will be equidistant along the curve.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param int n: Number of planes to create (*n* > 0).
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=c.u1).
    :param float u2: The parameter of the last plane (default=c.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param float tol: Tolerance.

    :raise RuntimeError: If :class:`.PointsAlongCurveByNumber` fails to
        generate points along the curve.
    """

    def __init__(self, c, n, ref_pln=None, u1=None, u2=None, d1=None,
                 d2=None, tol=1.0e-7):
        adp_crv = AdaptorCurve.to_adaptor(c)
        pnt_builder = PointsAlongCurveByNumber(adp_crv, n, u1, u2, d1, d2, tol)
        if pnt_builder.npts == 0:
            msg = ('Failed to generate points along the curve for creating '
                   'planes along a curve by number.')
            raise RuntimeError(msg)
        npts = pnt_builder.npts
        pnts = pnt_builder.points
        prms = pnt_builder.parameters
        spacing = pnt_builder.spacing

        plns = []
        if isinstance(ref_pln, Plane):
            gp_pln = ref_pln.gp_pln
            ax1 = gp_pln.Axis()
            dn = ax1.Direction()
            for p in pnts:
                pln = Plane(Geom_Plane(p, dn))
                plns.append(pln)
        else:
            for i in range(npts):
                p = pnts[i]
                u = prms[i]
                vn = adp_crv.deriv(u, 1)
                dn = Direction(vn)
                pln = Plane(Geom_Plane(p, dn))
                plns.append(pln)

        self._plns = plns
        self._nplns = npts
        self._prms = prms
        self._ds = spacing

    @property
    def nplanes(self):
        """
        :return: The number of planes.
        :rtype: int
        """
        return self._nplns

    @property
    def planes(self):
        """
        :return: The planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        return self._plns

    @property
    def parameters(self):
        """
        :return: The parameters.
        :rtype: list(float)
        """
        return self._prms

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_planes(self):
        """
        :return: The planes between the first and last planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        if self.nplanes < 3:
            return []
        return self._plns[1:-1]


class PlanesAlongCurveByDistance(object):
    """
    Create planes along a curve by distance between points. The origin of the
    planes will be equidistant along the curve. This method calculates the
    number of points given the curve length and then uses
    :class:`.PlanesAlongCurveByNumber`.

    :param c: The curve.
    :type c: afem.adaptor.entities.AdaptorCurve or afem.geometry.entities.Curve
        or afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=c.u1).
    :param float u2: The parameter of the last plane (default=c.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.
    :param float tol: Tolerance.

    :raise RuntimeError: If :class:`.PointsAlongCurveByDistance` fails to
        generate points along the curve.
    """

    def __init__(self, c, maxd, ref_pln=None, u1=None, u2=None, d1=None,
                 d2=None, nmin=0, tol=1.0e-7):
        adp_crv = AdaptorCurve.to_adaptor(c)
        pnt_builder = PointsAlongCurveByDistance(adp_crv, maxd, u1, u2, d1, d2,
                                                 nmin, tol)
        if pnt_builder.npts == 0:
            msg = ('Failed to generate points along the curve for creating '
                   'planes along a curve by distance.')
            raise RuntimeError(msg)
        npts = pnt_builder.npts
        pnts = pnt_builder.points
        prms = pnt_builder.parameters
        spacing = pnt_builder.spacing

        plns = []
        if isinstance(ref_pln, Plane):
            gp_pln = ref_pln.gp_pln
            ax1 = gp_pln.Axis()
            dn = ax1.Direction()
            for p in pnts:
                pln = Plane(Geom_Plane(p, dn))
                plns.append(pln)
        else:
            for i in range(npts):
                p = pnts[i]
                u = prms[i]
                vn = adp_crv.deriv(u, 1)
                dn = Direction(vn)
                pln = Plane(Geom_Plane(p, dn))
                plns.append(pln)

        self._plns = plns
        self._nplns = npts
        self._prms = prms
        self._ds = spacing

    @property
    def nplanes(self):
        """
        :return: The number of planes.
        :rtype: int
        """
        return self._nplns

    @property
    def planes(self):
        """
        :return: The planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        return self._plns

    @property
    def parameters(self):
        """
        :return: The parameters.
        :rtype: list(float)
        """
        return self._prms

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_planes(self):
        """
        :return: The planes between the first and last planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        if self.nplanes < 3:
            return []
        return self._plns[1:-1]


class PlanesBetweenPlanesByNumber(object):
    """
    Create planes between two other planes. This method will create a line
    normal to the first plane at its origin and then intersect
    that line with the second plane. Planes are then created along this line
    using :class:`.PlanesAlongCurveByNumber`. Planes are not generated at
    the same location as the boundary planes.

    :param afem.geometry.entities.Plane pln1: The first plane. This will be
        the reference plane to define the orientation of the new planes.
    :param afem.geometry.entities.Plane pln2: The last plane.
    :param int n: The number of planes.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.

    :raise RuntimeError: If the line extending from the normal of the first
        plane cannot be intersected with the second plane.
    """

    def __init__(self, pln1, pln2, n, d1=None, d2=None):
        p1 = pln1.eval(0., 0.)
        vn = pln1.norm(0., 0.)
        line = LineByVector(p1, vn).line
        csi = GeomAPI_IntCS(line.object, pln2.object)
        if csi.NbPoints() == 0:
            msg = ('Failed to intersect the second plane to create planes '
                   'between them.')
            raise RuntimeError(msg)
        p2 = csi.Point(1)

        n = int(n)
        if d1 is None:
            n += 1
        if d2 is None:
            n += 1

        c = NurbsCurveByPoints([p1, p2]).curve
        builder = PlanesAlongCurveByNumber(c, n, pln1, d1=d1, d2=d2)

        plns = builder.planes
        nplns = builder.nplanes
        spacing = None

        if plns and plns[0].distance(c.p1) <= 1.0e-7:
            plns.pop(0)
            nplns -= 1
        if plns and plns[-1].distance(c.p2) <= 1.0e-7:
            plns.pop(-1)
            nplns -= 1

        if nplns > 1:
            p1 = plns[0].eval(0., 0.)
            p2 = plns[1].eval(0., 0.)
            spacing = p1.distance(p2)

        self._plns = plns
        self._nplns = nplns
        self._ds = spacing

    @property
    def nplanes(self):
        """
        :return: The number of planes.
        :rtype: int
        """
        return self._nplns

    @property
    def planes(self):
        """
        :return: The planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        return self._plns

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_planes(self):
        """
        :return: The planes between the first and last planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        if self.nplanes < 3:
            return []
        return self._plns[1:-1]


class PlanesBetweenPlanesByDistance(object):
    """
    Create planes between two other planes by distance. This method will
    create a line normal to the first plane at its origin and then intersect
    that line with the second plane. Planes are then created along this line
    using :class:`.PlanesAlongCurveByNumber`. Planes are not generated at
    the same location as the boundary planes.

    :param afem.geometry.entities.Plane pln1: The first plane. This will be
        the reference plane to define the orientation of the new planes.
    :param afem.geometry.entities.Plane pln2: The last plane.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.

    :raise RuntimeError: If the line extending from the normal of the first
        plane cannot be intersected with the second plane.
    """

    def __init__(self, pln1, pln2, maxd, d1=None, d2=None, nmin=0):
        p1 = pln1.eval(0., 0.)
        vn = pln1.norm(0., 0.)
        line = LineByVector(p1, vn).line
        csi = GeomAPI_IntCS(line.object, pln2.object)
        if csi.NbPoints() == 0:
            msg = ('Failed to intersect the second plane to create planes '
                   'between them.')
            raise RuntimeError(msg)
        p2 = csi.Point(1)

        c = NurbsCurveByPoints([p1, p2]).curve
        builder = PlanesAlongCurveByDistance(c, maxd, pln1, d1=d1, d2=d2,
                                             nmin=nmin)

        plns = builder.planes
        nplns = builder.nplanes
        spacing = None

        if plns and plns[0].distance(c.p1) <= 1.0e-7:
            plns.pop(0)
            nplns -= 1
        if plns and plns[-1].distance(c.p2) <= 1.0e-7:
            plns.pop(-1)
            nplns -= 1

        if nplns > 1:
            p1 = plns[0].eval(0., 0.)
            p2 = plns[1].eval(0., 0.)
            spacing = p1.distance(p2)

        self._plns = plns
        self._nplns = nplns
        self._ds = spacing

    @property
    def nplanes(self):
        """
        :return: The number of planes.
        :rtype: int
        """
        return self._nplns

    @property
    def planes(self):
        """
        :return: The planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        return self._plns

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_planes(self):
        """
        :return: The planes between the first and last planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        if self.nplanes < 3:
            return []
        return self._plns[1:-1]


class PlanesAlongCurveAndSurfaceByDistance(object):
    """
    Create planes along a curve and surface by distance between points. The
    origin of the planes will be equidistant along the curve. This method
    calculates the number of points given the curve length and then uses
    :class:`.PlaneByCurveAndSurface` at each point.

    :param afem.geometry.entities.Curve c: The curve.
    :param afem.geometry.entities.Surface s: The surface.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param float u1: The parameter of the first plane (default=c.u1).
    :param float u2: The parameter of the last plane (default=c.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.
    :param float tol: Tolerance.

    :raise RuntimeError: If :class:`.PointsAlongCurveByDistance` fails to
        generate points along the curve.
    """

    def __init__(self, c, s, maxd, u1=None, u2=None, d1=None,
                 d2=None, nmin=0, tol=1.0e-7):
        pnt_builder = PointsAlongCurveByDistance(c, maxd, u1, u2, d1, d2,
                                                 nmin, tol)
        if pnt_builder.npts == 0:
            msg = ('Failed to generate points along the curve for creating '
                   'planes along a curve by distance.')
            raise RuntimeError(msg)
        npts = pnt_builder.npts
        prms = pnt_builder.parameters
        spacing = pnt_builder.spacing

        plns = []
        for i in range(npts):
            u = prms[i]
            pln = PlaneByCurveAndSurface(c, s, u).plane
            plns.append(pln)

        self._plns = plns
        self._nplns = npts
        self._prms = prms
        self._ds = spacing

    @property
    def nplanes(self):
        """
        :return: The number of planes.
        :rtype: int
        """
        return self._nplns

    @property
    def planes(self):
        """
        :return: The planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        return self._plns

    @property
    def parameters(self):
        """
        :return: The parameters.
        :rtype: list(float)
        """
        return self._prms

    @property
    def spacing(self):
        """
        :return: The spacing between the first and second points if there
            are more than one point. Otherwise *None*.
        :rtype: float or None
        """
        return self._ds

    @property
    def interior_planes(self):
        """
        :return: The planes between the first and last planes.
        :rtype: list(afem.geometry.entities.Plane)
        """
        if self.nplanes < 3:
            return []
        return self._plns[1:-1]

    def rotate_x(self, angle):
        """
        Rotate each of the planes around their local x-axis.

        :param float angle: The rotation angle in degrees.

        :return: None.
        """
        for pln in self.planes:
            pln.rotate_x(angle)

    def rotate_y(self, angle):
        """
        Rotate each of the planes around their local y-axis.

        :param float angle: The rotation angle in degrees.

        :return: None.
        """
        for pln in self.planes:
            pln.rotate_y(angle)


# NURBSSURFACE ----------------------------------------------------------------

class NurbsSurfaceByInterp(object):
    """
    Create a surface by interpolating curves. This method was developed from
    scratch using "The NURBS Book" since OpenCASCADE did not support
    interpolating surfaces with *q* = 1.

    :param list(curve_like) crvs: List of curves to interpolate.
    :param int q: Degree. The parameter will be adjusted if the number of
        curves provided does not support the desired degree.
    :param OCCT.Approx.Approx_ParametrizationType parm_type: Parametrization
        type.
    :param float tol2d: 2-D tolerance.
    """

    def __init__(self, crvs, q=3, parm_type=Approx_ChordLength, tol2d=1.0e-9):
        q = int(q)

        ncrvs = len(crvs)
        if ncrvs - 1 < q:
            q = ncrvs - 1

        # Make curves compatible using section generator.
        sec_gen = GeomFill_SectionGenerator()
        for c in crvs:
            sec_gen.AddCurve(c.object)
        sec_gen.Perform(tol2d)

        # Use old method since OCC struggles to interpolate curves of low
        # order. Gather all data for old method.
        p = sec_gen.Degree()
        is_u_periodic = sec_gen.IsPeriodic()
        tcol_uknots = TColStd_Array1OfReal(1, sec_gen.NbKnots())
        tcol_umult = TColStd_Array1OfInteger(1, sec_gen.NbKnots())
        sec_gen.KnotsAndMults(tcol_uknots, tcol_umult)
        temp = []
        for i in range(1, ncrvs + 1):
            tcol_poles = TColgp_Array1OfPnt(1, sec_gen.NbPoles())
            sec_gen.Poles(i, tcol_poles)
            tcol_weights = TColStd_Array1OfReal(1, sec_gen.NbPoles())
            sec_gen.Weights(i, tcol_weights)
            cp = occ_utils.to_np_from_tcolgp_array1_pnt(tcol_poles)
            w = occ_utils.to_np_from_tcolstd_array1_real(tcol_weights)
            cpw = geom_utils.homogenize_array1d(cp, w)
            temp.append(cpw)

        # Compute v-direction parameters between [0, 1].
        # Find parameters between each curve by averaging each segment.
        temp = array(temp, dtype=float)
        pnts_matrix = temp.transpose((1, 0, 2))
        n = sec_gen.NbPoles() - 1
        m = ncrvs - 1
        v_matrix = zeros((n + 1, m + 1), dtype=float)
        for i in range(0, n + 1):
            if parm_type == Approx_IsoParametric:
                vknots = geom_utils.uniform_parameters(pnts_matrix[i, :], 0.,
                                                       1.)
            elif parm_type == Approx_ChordLength:
                vknots = geom_utils.chord_parameters(pnts_matrix[i, :], 0., 1.)
            else:
                vknots = geom_utils.centripetal_parameters(pnts_matrix[i, :],
                                                           0., 1.)
            v_matrix[i] = vknots
        # Average each column.
        vknots = mean(v_matrix, axis=0, dtype=float)
        vknots[0] = 0.0
        vknots[-1] = 1.0

        s = m + q + 1
        vk = zeros(s + 1, dtype=float)
        vk[s - q:] = 1.0
        for j in range(1, m - q + 1):
            temp = 0.
            for i in range(j, j + q):
                temp += vknots[i]
            vk[j + q] = 1.0 / q * temp
        # Compute OCC vknots and vmult.
        tcol_vknot_seq = occ_utils.to_tcolstd_array1_real(vk)

        nv = BSplCLib.KnotsLength_(tcol_vknot_seq, False)
        tcol_vknots = TColStd_Array1OfReal(1, nv)
        tcol_vmult = TColStd_Array1OfInteger(1, nv)
        BSplCLib.Knots_(tcol_vknot_seq, tcol_vknots, tcol_vmult, False)

        # Perform n + 1 interpolations in v-direction to generate surface
        # control points.
        cpw = zeros((n + 1, m + 1, 4), dtype=float)
        for i in range(0, n + 1):
            qp = pnts_matrix[i]
            # Set up system of linear equations.
            a = zeros((m + 1, m + 1), dtype=float)
            for j in range(0, m + 1):
                # span1, _ = CLib.bsplclib_LocateParameter(pnt, tcol_vknots,
                #                                          tcol_vmult,
                #                                         vknots[j], False)
                # Evaluate basis function. Figure out how to use math_Matrix
                # and use OCC in future.
                # mat = occ_math.math_Matrix(1, q, 1, q)
                # CLib.bsplclib_EvalBsplineBasis(span, q, q, tcol_vknot_seq,
                #                                vknots[j], mat)
                span = geom_utils.find_span(m, q, vknots[j], vk)
                a[j, span - q: span + 1] = geom_utils.basis_funs(span,
                                                                 vknots[j], q,
                                                                 vk)
            # Solve for [a][cp] = [qp] using LU decomposition.
            lu, piv = lu_factor(a, overwrite_a=True, check_finite=False)
            cpw[i, :] = lu_solve((lu, piv), qp, trans=0, overwrite_b=True,
                                 check_finite=True)

        # Create surface.
        cp, w = geom_utils.dehomogenize_array2d(cpw)
        tcol_poles = occ_utils.to_tcolgp_array2_pnt(cp)
        tcol_weights = occ_utils.to_tcolstd_array2_real(w)
        s = Geom_BSplineSurface(tcol_poles, tcol_weights, tcol_uknots,
                                tcol_vknots, tcol_umult, tcol_vmult, p, q,
                                is_u_periodic, False)

        self._s = NurbsSurface(s)

    @property
    def surface(self):
        """
        :return: The NURBS surface.
        :rtype: afem.geometry.entities.NurbsSurface
        """
        return self._s


class NurbsSurfaceByApprox(object):
    """
    Create a NURBS surface by approximating curves.

    :param list(curve_like) crvs: List of curves.
    :param int dmin: Minimum degree.
    :param int dmax: Maximum degree.
    :param float tol3d: 3-D tolerance.
    :param float tol2d: 2-D tolerance.
    :param int niter: Number of iterations.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity: Desired continuity of curve.
    :param OCCT.Approx.Approx_ParametrizationType parm_type: Parametrization
        type.

    :raise RuntimeError: If OCC method fails to approximate the curves with a
        surface.
    """

    def __init__(self, crvs, dmin=3, dmax=8, tol3d=1.0e-3, tol2d=1.0e-6,
                 niter=5, continuity=Geometry.C2,
                 parm_type=Approx_ChordLength):
        dmin = int(dmin)
        dmax = int(dmax)

        # Initialize approximation tool
        app_tool = GeomFill_AppSurf(dmin, dmax, tol3d, tol2d, niter)

        # Set parametrization type
        app_tool.SetParType(parm_type)

        # Set continuity
        app_tool.SetContinuity(continuity)

        # Use section generator to make all curves compatible
        sec_gen = GeomFill_SectionGenerator()
        for c in crvs:
            sec_gen.AddCurve(c.object)
        sec_gen.Perform(tol3d)

        # Create line tool
        line_tool = GeomFill_Line(len(crvs))

        # Perform the approximation
        app_tool.Perform(line_tool, sec_gen, False)
        if not app_tool.IsDone():
            msg = 'OCC method failed.'
            raise RuntimeError(msg)

        # Create a surface
        tcol_poles = app_tool.SurfPoles()
        tcol_weights = app_tool.SurfWeights()
        tcol_uknots = app_tool.SurfUKnots()
        tcol_vknots = app_tool.SurfVKnots()
        tcol_umult = app_tool.SurfUMults()
        tcol_vmult = app_tool.SurfVMults()
        p = app_tool.UDegree()
        q = app_tool.VDegree()
        is_u_periodic = sec_gen.IsPeriodic()
        is_v_periodic = False
        s = Geom_BSplineSurface(tcol_poles, tcol_weights, tcol_uknots,
                                tcol_vknots, tcol_umult, tcol_vmult, p, q,
                                is_u_periodic, is_v_periodic)

        tol3d_reached, tol2d_reached = app_tool.TolReached(0., 0.)
        self._s = NurbsSurface(s)
        self._tol3d_reached = tol3d_reached
        self._tol2d_reached = tol2d_reached

    @property
    def surface(self):
        """
        :return: The NURBS surface.
        :rtype: afem.geometry.entities.NurbsSurface
        """
        return self._s

    @property
    def told3d_reached(self):
        """
        :return: 3-D tolerance achieved.
        :rtype: float
        """
        return self._tol3d_reached

    @property
    def told2d_reached(self):
        """
        :return: 2-D tolerance achieved.
        :rtype: float
        """
        return self._tol2d_reached
