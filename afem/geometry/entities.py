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
from math import radians

from OCCT.BRepBuilderAPI import (BRepBuilderAPI_MakeFace,
                                 BRepBuilderAPI_MakeEdge,
                                 BRepBuilderAPI_MakeVertex)
from OCCT.BRepGProp import BRepGProp
from OCCT.GCPnts import GCPnts_AbscissaPoint
from OCCT.GProp import GProp_GProps
from OCCT.Geom import (Geom_Line, Geom_Circle, Geom_Ellipse, Geom_BSplineCurve,
                       Geom_TrimmedCurve, Geom_Plane, Geom_BSplineSurface,
                       Geom_Curve, Geom_Surface, Geom_Geometry)
from OCCT.Geom2d import Geom2d_BSplineCurve, Geom2d_Curve
from OCCT.Geom2dAdaptor import Geom2dAdaptor_Curve
from OCCT.GeomAPI import (GeomAPI, GeomAPI_ProjectPointOnCurve,
                          GeomAPI_ProjectPointOnSurf)
from OCCT.GeomAbs import (GeomAbs_Shape, GeomAbs_JoinType, GeomAbs_CurveType,
                          GeomAbs_SurfaceType)
from OCCT.GeomAdaptor import GeomAdaptor_Curve, GeomAdaptor_Surface
from OCCT.GeomLib import GeomLib_IsPlanarSurface
from OCCT.TColStd import (TColStd_Array1OfInteger, TColStd_Array1OfReal,
                          TColStd_Array2OfReal)
from OCCT.TColgp import TColgp_Array1OfPnt, TColgp_Array2OfPnt
from OCCT.gp import (gp_Ax1, gp_Ax2, gp_Ax3, gp_Dir, gp_Pnt, gp_Pnt2d,
                     gp_Vec2d, gp_Dir2d, gp_Vec)
from numpy import add, array, float64, subtract, ones

from afem.base.entities import ViewableItem
from afem.geometry import utils as geom_utils
from afem.misc import utils as misc_utils
from afem.occ import utils as occ_utils

__all__ = ["Geometry2D", "Point2D", "Vector2D", "Direction2D",
           "Curve2D", "NurbsCurve2D",
           "Geometry", "Point", "Direction", "Vector", "Axis1", "Axis3",
           "Curve", "Line", "Circle", "Ellipse", "NurbsCurve", "TrimmedCurve",
           "Surface", "Plane", "NurbsSurface"]


# 2-D -------------------------------------------------------------------------
# Types derived from OpenCASCADE geometric processor (gp) package.
class Point2D(gp_Pnt2d):
    """
    A 2-D Cartesian point derived from ``gp_Pnt2d``.
    """

    def __init__(self, *args):
        super(Point2D, self).__init__(*args)

    def __str__(self):
        return 'Point2D({0}, {1})'.format(*self.xy)

    def __repr__(self):
        return 'Point2D({0}, {1})'.format(*self.xy)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.xy, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.xy:
            yield elm

    def __len__(self):
        return 2

    def __getitem__(self, item):
        return self.xy[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def xy(self):
        """
        :return: The point xy-location.
        :rtype: numpy.ndarray
        """
        return array([self.X(), self.Y()], dtype=float64)

    @property
    def x(self):
        return self.X()

    @x.setter
    def x(self, x):
        """
        The point x-location.

        :Getter: Returns the x-location.
        :Setter: Sets the x-location.
        :type: float
        """
        self.SetX(x)

    @property
    def y(self):
        """
        The point y-location.

        :Getter: Returns the y-location.
        :Setter: Sets the y-location.
        :type: float
        """
        return self.Y()

    @y.setter
    def y(self, y):
        self.SetY(y)

    def set_xy(self, xy):
        """
        Set point coordinates.

        :param point2d_like xy: Point coordinates.

        :return: *True* if set, *False* if not.
        :rtype: bool
        """
        xy = Point2D.to_point2d(xy)
        self.SetXY(xy.XY())
        return True

    def distance(self, other):
        """
        Compute the distance between two points.

        :param point2d_like other: The other point.

        :return: Distance to the other point.
        :rtype: float
        """
        other = Point2D.to_point2d(other)
        return self.Distance(other)

    def is_equal(self, other, tol=1.0e-7):
        """
        Check for coincident points.

        :param point_like other: The other point.
        :param float tol: Tolerance for coincidence.

        :return: *True* if coincident, *False* if not.
        :rtype: bool
        """
        other = Point2D.to_point2d(other)
        return self.IsEqual(other, tol)

    def scale(self, pnt, scale):
        """
        Scale the point.

        :param point2d_like pnt: The reference point.
        :param float scale: The scaling value.

        :return: *True* if scaled.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        self.Scale(pnt, scale)
        return True

    def rotate(self, pnt, angle):
        """
        Rotate the point about a point.

        :param point2d_like pnt: The reference point.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        angle = radians(angle)
        self.Rotate(pnt, angle)
        return True

    def copy(self):
        """
        Return a new copy of the point.

        :return: New point.
        :rtype: afem.geometry.entities.Point2D
        """
        return Point2D(*self.xy)

    @classmethod
    def is_point2d_like(cls, entity):
        """
        Check if the entity is point2d_like.

        :param entity: An entity.

        :return: *True* if the entity is point2d_like, *False* if not.
        :rtype: bool
        """
        if isinstance(entity, gp_Pnt2d):
            return True
        if misc_utils.is_array_like(entity):
            return len(entity) == 2
        return False

    @classmethod
    def to_point2d(cls, entity):
        """
        Convert entity to a point if possible.

        :param entity: An entity.

        :return: The entity if already a point, or a new point if it is
            "point like".
        :rtype: afem.geometry.entities.Point2D

        :raise TypeError: If entity cannot be converted to a Point.
        """
        if entity is None:
            return None

        if isinstance(entity, cls):
            return entity
        elif isinstance(entity, gp_Pnt2d):
            return cls(entity.XY())
        elif cls.is_point2d_like(entity):
            return cls(entity[0], entity[1])
        else:
            raise TypeError('Cannot convert to Point2D.')

    @classmethod
    def by_xy(cls, x, y):
        """
        Create a 2-D point by *x* and *y* locations.

        :param float x: The x-location.
        :param float y: The y-location.

        :return: The new 2-D point.
        :rtype: afem.geometry.entities.Point2D
        """
        return cls(x, y)


class Direction2D(gp_Dir2d):
    """
    Unit vector in 2-D space derived from ``gp_Dir2d``.
    """

    def __init__(self, *args):
        super(Direction2D, self).__init__(*args)

    def __str__(self):
        return 'Direction2D({0}, {1})'.format(*self.xy)

    def __repr__(self):
        return 'Direction2D({0}, {1})'.format(*self.xy)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.ij, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.ijk:
            yield elm

    def __len__(self):
        return 2

    def __getitem__(self, item):
        return self.ij[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def i(self):
        """
        The direction i-component.

        :Getter: Returns the i-component.
        :Setter: Sets the i-component.
        :type: float
        """
        return self.X()

    @i.setter
    def i(self, i):
        self.SetX(i)

    @property
    def j(self):
        """
        The direction j-component.

        :Getter: Returns the j-component.
        :Setter: Sets the j-component.
        :type: float
        """
        return self.Y()

    @j.setter
    def j(self, j):
        self.SetY(j)

    @property
    def ij(self):
        """
        :return: The direction ij-components.
        :rtype: numpy.ndarray
        """
        return array([self.i, self.j], dtype=float64)

    @property
    def xy(self):
        """
        :return: The direction ij-components (Same as ij property,
            for compatibility only).
        :rtype: numpy.ndarray
        """
        return self.ij

    @property
    def mag(self):
        """
        :return: Direction magnitude is always 1.
        :rtype: float
        """
        return 1.

    def rotate(self, pnt, angle):
        """
        Rotate the direction about a point.

        :param point2d_like pnt: The reference point.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        angle = radians(angle)
        self.Rotate(pnt, angle)
        return True

    @classmethod
    def by_xy(cls, x, y):
        """
        Create a 2-D direction by *x* and *y* components.

        :param float x: The x-component.
        :param float y: The y-component.

        :return: The new 2-D direction.
        :rtype: afem.geometry.entities.Direction2D
        """
        return cls(x, y)

    @classmethod
    def by_vector(cls, v):
        """
        Create a 2-D direction from a 2-D vector.

        :param afem.geometry.entities.Vector2D v: The 2-D vector.

        :return: The new 2-D direction.
        :rtype: afem.geometry.entities.Direction2D
        """
        return cls(v)


class Vector2D(gp_Vec2d):
    """
    Vector in 2-D space derived from ``gp_Vec2d``.
    """

    def __init__(self, *args):
        super(Vector2D, self).__init__(*args)

    def __str__(self):
        return 'Vector2D({0}, {1})'.format(*self.xy)

    def __repr__(self):
        return 'Vector2D({0}, {1})'.format(*self.xy)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.xy, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.xy:
            yield elm

    def __len__(self):
        return 2

    def __getitem__(self, item):
        return self.xy[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def x(self):
        """
        The vector x-component.

        :Getter: Returns the x-component.
        :Setter: Sets the x-component.
        :type: float
        """
        return self.X()

    @x.setter
    def x(self, x):
        self.SetX(x)

    @property
    def y(self):
        """
        The vector y-component.

        :Getter: Returns the y-component.
        :Setter: Sets the y-component.
        :type: float
        """
        return self.Y()

    @y.setter
    def y(self, y):
        self.SetY(y)

    @property
    def xy(self):
        """
        :return: The vector xy-components.
        :rtype: numpy.ndarray
        """
        return array([self.x, self.y], dtype=float64)

    @property
    def mag(self):
        """
        :return: Vector magnitude.
        :rtype: float
        """
        return self.Magnitude()

    @property
    def ij(self):
        """
        :return: Normalized vector xy-components.
        :rtype: numpy.ndarray
        """
        return self.xy / self.mag

    def reverse(self):
        """
        Reverse the direction of the vector.

        :return: None.
        """
        self.Reverse()

    def normalize(self):
        """
        Normalize the vector.

        :return: None.
        """
        self.Normalize()

    def scale(self, scale):
        """
        Scale the vector.

        :param float scale: Scaling value.

        :return: None.
        """
        self.Scale(scale)

    def rotate(self, pnt, angle):
        """
        Rotate the vector about a point.

        :param point2d_like pnt: The reference point.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        angle = radians(angle)
        self.Rotate(pnt, angle)
        return True

    @classmethod
    def to_vector2d(cls, entity):
        """
        Convert entity to a 2-D vector if possible.

        :param vector2d_like entity: An entity.

        :return: The entity if already a Vector2D, or a new Vector2D if it is
            vector2d_like.
        :rtype: afem.geometry.entities.Vector2D

        :raise TypeError: If entity cannot be converted to a Vector2D.
        """
        if entity is None:
            return None

        if isinstance(entity, cls):
            return entity
        elif misc_utils.is_array_like(entity):
            return cls(entity[0], entity[1])
        elif isinstance(entity, Direction2D):
            return cls(entity)
        elif isinstance(entity, gp_Vec2d):
            return cls(entity.XY())
        else:
            raise TypeError('Cannot convert to Vector2D.')

    @classmethod
    def by_xy(cls, x, y):
        """
        Create a 2-D vector by *x* and *y* components.

        :param float x: The x-component.
        :param float y: The y-component.

        :return: The new 2-D vector.
        :rtype: afem.geometry.entities.Vector2D
        """
        return cls(x, y)

    @classmethod
    def by_direction(cls, d):
        """
        Create a 2-D vector from a 2-D direction.

        :param afem.geometry.entities.Direction2D d: The 2-D direction.

        :return: The new 2-D vector.
        :rtype: afem.geometry.entities.Vector2D
        """
        return cls(d)

    @classmethod
    def by_points(cls, p1, p2):
        """
        Create a 2-D vector by two 2-D points.

        :param afem.geometry.entities.Point2D p1: The first point.
        :param afem.geometry.entities.Point2D p2: The second point.

        :return: The new 2-D vector.
        :rtype: afem.geometry.entities.Vector2D
        """
        return cls(p1, p2)


# Transient types that are wrapped.
class Geometry2D(object):
    """
    Base class for 2-D geometry.

    :param OCCT.Geom2d.Geom2d_Geometry obj: The geometry object.

    :raise TypeError: If the wrapped type of ``obj`` does not match the
        expected type.
    """

    def __init__(self, obj):
        if not isinstance(obj, self._OCC_TYPE):
            n1 = self._OCC_TYPE.__name__
            n2 = obj.__class__.__name__
            msg = ('Error wrapping an OpenCASCADE 2-D geometry type. '
                   'Expected a {} but got a {}.'.format(n1, n2))
            raise TypeError(msg)
        super(Geometry2D, self).__init__()
        self._object = obj

    @property
    def object(self):
        """
        :return: The underlying OpenCASCADE object.
        :rtype: OCCT.Geom2d.Geom2d_Geometry
        """
        return self._object

    def scale(self, pnt, scale):
        """
        Scale the geometry.

        :param point2d_like pnt: The reference point.
        :param float scale: The scaling value.

        :return: *True* if scaled.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        self.object.Scale(pnt, scale)
        return True

    def rotate(self, pnt, angle):
        """
        Rotate the geometry about a point.

        :param point2d_like pnt: The reference point.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        pnt = Point2D.to_point2d(pnt)
        angle = radians(angle)
        self.object.Rotate(pnt, angle)
        return True


class Curve2D(Geometry2D):
    """
    Base class for 2-D curves around ``Geom2d_Curve``.
    """
    _OCC_TYPE = Geom2d_Curve

    @property
    def u1(self):
        """
        :return: The first parameter.
        :rtype: float
        """
        return self.object.FirstParameter()

    @property
    def u2(self):
        """
        :return: The last parameter.
        :rtype: float
        """
        return self.object.LastParameter()

    @property
    def is_closed(self):
        """
        :return: *True* if curve is closed, *False* if not.
        :rtype: bool
        """
        return self.object.IsClosed()

    @property
    def is_periodic(self):
        """
        :return: *True* if curve is periodic, *False* if not.
        :rtype: bool
        """
        return self.object.IsPeriodic()

    @property
    def p1(self):
        """
        :return: The first point.
        :rtype: afem.geometry.entities.Point2D
        """
        return self.eval(self.u1)

    @property
    def p2(self):
        """
        :return: The last point.
        :rtype: afem.geometry.entities.Point2D
        """
        return self.eval(self.u2)

    @property
    def length(self):
        """
        :return: Curve length.
        :rtype: float
        """
        return self.arc_length(self.u1, self.u2)

    def copy(self):
        """
        Return a new copy of the curve.

        :return: New curve.
        :rtype: afem.geometry.entities.Curve
        """
        h_crv = self.object.Copy()
        return Curve2D.wrap(h_crv)

    def local_to_global_u(self, *args):
        """
        Convert parameter(s) from local domain 0. <= u <= 1. to global domain
        a <= u <= b.

        :param float args: Local parameter(s).

        :return: Global parameter(s).
        :rtype: float or list(float)
        """
        return geom_utils.local_to_global_param(self.u1, self.u2, *args)

    def global_to_local_u(self, *args):
        """
        Convert parameter(s) from global domain a <= u <= b to local domain
        0. <= u <= 1.

        :param float args: Global parameter(s).

        :return: Local parameter(s).
        :rtype: float or list(float)
        """
        return geom_utils.global_to_local_param(self.u1, self.u2, *args)

    def eval(self, u):
        """
        Evaluate a point on the curve.

        :param float u: Curve parameter.

        :return: Curve point.
        :rtype: afem.geometry.entities.Point2D
        """
        p = Point2D()
        self.object.D0(u, p)
        return p

    def deriv(self, u, d=1):
        """
        Evaluate a derivative on the curve.

        :param float u: Curve parameter.
        :param int d: Derivative to evaluate.

        :return: Curve derivative.
        :rtype: afem.geometry.entities.Vector2D
        """
        return Vector2D(self.object.DN(u, d).XY())

    def reverse(self):
        """
        Reverse curve direction.

        :return: None.
        """
        self.object.Reverse()

    def reversed_u(self, u):
        """
        Calculate the parameter on the reversed curve.

        :param float u: Curve parameter.

        :return: Reversed parameter.
        :rtype: float
        """
        return self.object.ReversedParameter(u)

    def arc_length(self, u1, u2, tol=1.0e-7):
        """
        Calculate the curve length between the parameters.

        :param float u1: First parameter.
        :param float u2: Last parameter.
        :param float tol: The tolerance.

        :return: Curve length.
        :rtype: float
        """
        if u1 > u2:
            u1, u2 = u2, u1
        adp_crv = Geom2dAdaptor_Curve(self.object)
        return GCPnts_AbscissaPoint.Length_(adp_crv, u1, u2, tol)

    def to_3d(self, pln):
        """
        Convert this 2-D curve in the plane.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: The 3-D curve.
        :rtype: afem.geometry.entities.Curve2D
        """
        geom_crv = GeomAPI.To3d_(self.object, pln.gp_pln)
        return Curve.wrap(geom_crv)

    @staticmethod
    def wrap(curve):
        """
        Wrap the OpenCASCADE curve based on its type.

        :param OCCT.Geom2d.Geom2d_Curve curve: The curve.

        :return: The wrapped curve.
        :rtype: afem.geometry.entities.Curve2D
        """
        if isinstance(curve, Geom2d_BSplineCurve):
            return NurbsCurve2D(curve)

        # Catch for unsupported type
        if isinstance(curve, Geom_Curve):
            return Curve2D(curve)

        raise TypeError('Curve2D type not supported.')


class NurbsCurve2D(Curve2D):
    """
    NURBS curve in 2-D space around ``Geom2d_BSplineCurve``.
    """
    _OCC_TYPE = Geom2d_BSplineCurve

    @property
    def p(self):
        """
        :return: Degree of curve.
        :rtype: int
        """
        return self.object.Degree()

    @property
    def n(self):
        """
        :return: Number of control points - 1.
        :rtype: int
        """
        return self.object.NbPoles() - 1

    @property
    def knots(self):
        """
        :return: Knot vector.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfReal(1, self.object.NbKnots())
        self.object.Knots(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_array)

    @property
    def mult(self):
        """
        :return: Multiplicity of knot vector.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfInteger(1, self.object.NbKnots())
        self.object.Multiplicities(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_integer(tcol_array)

    @property
    def uk(self):
        """
        :return: Knot sequence.
        :rtype: numpy.ndarray
        """
        tcol_knot_seq = TColStd_Array1OfReal(1, self.object.NbPoles() +
                                             self.object.Degree() + 1)
        self.object.KnotSequence(tcol_knot_seq)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_knot_seq)

    def set_domain(self, u1=0., u2=1.):
        """
        Reparameterize the knot vector between *u1* and *u2*.

        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        if u1 > u2:
            return False
        tcol_knots = TColStd_Array1OfReal(1, self.object.NbKnots())
        self.object.Knots(tcol_knots)
        geom_utils.reparameterize_knots(u1, u2, tcol_knots)
        self.object.SetKnots(tcol_knots)
        return True

    def segment(self, u1, u2):
        """
        Segment the curve between parameters.

        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: *True* if segmented, *False* if not.
        :rtype: bool
        """
        if u1 > u2:
            return False
        self.object.Segment(u1, u2)
        return True


# 3-D -------------------------------------------------------------------------
# Types derived from OpenCASCADE geometric processor (gp) package.
class Point(gp_Pnt, ViewableItem):
    """
    A 3-D Cartesian point derived from ``gp_Pnt``.
    """

    def __init__(self, *args):
        super(Point, self).__init__(*args)
        ViewableItem.__init__(self)
        self.set_color(1, 1, 0)

    def __str__(self):
        return 'Point({0:.3f}, {1:.3f}, {2:.3f})'.format(*self.xyz)

    def __repr__(self):
        return 'Point({0:.3f}, {1:.3f}, {2:.3f})'.format(*self.xyz)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.xyz, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.xyz:
            yield elm

    def __len__(self):
        return 3

    def __getitem__(self, item):
        return self.xyz[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Vertex
        """
        return BRepBuilderAPI_MakeVertex(self).Vertex()

    @property
    def x(self):
        """
        The point x-location.

        :Getter: Returns the x-location.
        :Setter: Sets the x-location.
        :type: float
        """
        return self.X()

    @x.setter
    def x(self, x):
        self.SetX(x)

    @property
    def y(self):
        """
        The point y-location.

        :Getter: Returns the y-location.
        :Setter: Sets the y-location.
        :type: float
        """
        return self.Y()

    @y.setter
    def y(self, y):
        self.SetY(y)

    @property
    def z(self):
        """
        The point z-location.

        :Getter: Returns the z-location.
        :Setter: Sets the z-location.
        :type: float
        """
        return self.Z()

    @z.setter
    def z(self, z):
        self.SetZ(z)

    @property
    def xyz(self):
        """
        :return: The point xyz-location.
        :rtype: numpy.ndarray
        """
        return array([self.X(), self.Y(), self.Z()], dtype=float64)

    def set_xyz(self, xyz):
        """
        Set point coordinates.

        :param point_like xyz: Point coordinates.

        :return: *True* if set, *False* if not.
        :rtype: bool
        """
        xyz = Point.to_point(xyz)
        self.SetXYZ(xyz.XYZ())
        return True

    def distance(self, other):
        """
        Compute the distance between two points.

        :param point_like other: The other point.

        :return: Distance to the other point.
        :rtype: float
        """
        other = Point.to_point(other)
        return self.Distance(other)

    def is_equal(self, other, tol=1.0e-7):
        """
        Check for coincident points.

        :param point_like other: The other point.
        :param float tol: Tolerance for coincidence.

        :return: *True* if coincident, *False* if not.
        :rtype: bool
        """
        other = Point.to_point(other)
        return self.IsEqual(other, tol)

    def translate(self, v):
        """
        Translate the geometry along the vector.

        :param vector_like v: The translation vector.

        :return: *True* if translated, *False* if not.
        :rtype: bool

        :raise TypeError: If *v* cannot be converted to a vector.
        """
        v = Vector.to_vector(v)
        self.Translate(v)
        return True

    def mirror(self, pln):
        """
        Mirror the geometry using a plane.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: *True* if mirrored.
        :rtype: bool
        """
        gp_pln = pln.gp_pln
        gp_ax2 = gp_Ax2()
        gp_ax2.SetAxis(gp_pln.Axis())
        self.Mirror(gp_ax2)
        return True

    def scale(self, pnt, s):
        """
        Scale the geometry.

        :param point_like pnt: The reference point.
        :param float s: The scaling value.

        :return: *True* if scaled.
        :rtype: bool
        """
        pnt = Point.to_point(pnt)
        self.Scale(pnt, s)
        return True

    def rotate(self, ax1, angle):
        """
        Rotate the geometry about an axis.

        :param afem.geometry.entities.Axis1 ax1: The axis of rotation.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        angle = radians(angle)
        self.Rotate(ax1, angle)
        return True

    def rotate_xyz(self, origin, x, y, z):
        """
        Rotate the point about the global x-, y-, and z-axes using
        *origin* as the point of rotation if *origin* is a point. Otherwise, if
        *origin* is an :class:`.Axis3`, rotate the point about the axes of the
        coordinate system. Rotations follow the right-hand rule for each axis.

        :param origin: The origin of rotation.
        :type origin: point_like or afem.geometry.entities.Axis3
        :param float x: Rotation about x-axis in degrees.
        :param float y: Rotation about y-axis in degrees.
        :param float z: Rotation about z-axis in degrees.

        :return: None.
        """
        is_local = False
        if isinstance(origin, Axis3):
            is_local = True
        else:
            origin = Point.to_point(origin)

        # x-axis
        if is_local:
            ax = origin.x_axis
        else:
            ax = gp_Ax1(origin, gp_Dir(1., 0., 0.))
        self.Rotate(ax, radians(x))

        # y-axis
        if is_local:
            ax = origin.y_axis
        else:
            ax = gp_Ax1(origin, gp_Dir(0., 1., 0.))
        self.Rotate(ax, radians(y))

        # z-axis
        if is_local:
            ax = origin.z_axis
        else:
            ax = gp_Ax1(origin, gp_Dir(0., 0., 1.))
        self.Rotate(ax, radians(z))

    def copy(self):
        """
        Return a new copy of the point.

        :return: New point.
        :rtype: afem.geometry.entities.Point
        """
        return Point(*self.xyz)

    @classmethod
    def by_xyz(cls, x, y, z):
        """
        Create a point by *x*, *y*, and *z* locations.

        :param float x: The x-location.
        :param float y: The y-location.
        :param float z: The z-location.

        :return: The new point.
        :rtype: afem.geometry.entities.Point
        """
        return cls(x, y, z)

    @classmethod
    def is_point_like(cls, entity):
        """
        Check if the entity is point_like.

        :param entity: An entity.

        :return: *True* if the entity is point_like, *False* if not.
        :rtype: bool
        """
        if isinstance(entity, gp_Pnt):
            return True
        if misc_utils.is_array_like(entity):
            return len(entity) == 3
        return False

    @classmethod
    def to_point(cls, entity):
        """
        Convert entity to a point if possible.

        :param entity: An entity.

        :return: The entity if already a point, or a new point if it is
            "point like".
        :rtype: afem.geometry.entities.Point

        :raise TypeError: If entity cannot be converted to a Point.
        """
        if entity is None:
            return None

        if isinstance(entity, cls):
            return entity
        elif isinstance(entity, gp_Pnt):
            return cls(entity.XYZ())
        elif cls.is_point_like(entity):
            return cls(entity[0], entity[1], entity[2])
        else:
            raise TypeError('Cannot convert to Point.')


class Direction(gp_Dir):
    """
    Unit vector in 3-D space derived from ``gp_Dir``.
    """

    def __init__(self, *args):
        super(Direction, self).__init__(*args)

    def __str__(self):
        return 'Direction({0}, {1}, {2})'.format(*self.xyz)

    def __repr__(self):
        return 'Direction({0}, {1}, {2})'.format(*self.xyz)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.ijk, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.ijk:
            yield elm

    def __len__(self):
        return 3

    def __getitem__(self, item):
        return self.ijk[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def i(self):
        """
        The direction i-component.

        :Getter: Returns the i-component.
        :Setter: Sets the i-component.
        :type: float
        """
        return self.X()

    @i.setter
    def i(self, i):
        self.SetX(i)

    @property
    def j(self):
        """
        The direction j-component.

        :Getter: Returns the j-component.
        :Setter: Sets the j-component.
        :type: float
        """
        return self.Y()

    @j.setter
    def j(self, j):
        self.SetY(j)

    @property
    def k(self):
        """
        The direction k-component.

        :Getter: Returns the k-component.
        :Setter: Sets the k-component.
        :type: float
        """
        return self.Z()

    @k.setter
    def k(self, k):
        self.SetZ(k)

    @property
    def ijk(self):
        """
        :return: The direction ijk-components.
        :rtype: numpy.ndarray
        """
        return array([self.i, self.j, self.k], dtype=float64)

    @property
    def xyz(self):
        """
        :return: The direction ijk-components (Same as ijk property,
            for compatibility only).
        :rtype: numpy.ndarray
        """
        return self.ijk

    @property
    def mag(self):
        """
        :return: Direction magnitude is always 1.
        :rtype: float
        """
        return 1.

    def mirror(self, pln):
        """
        Mirror the geometry using a plane.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: *True* if mirrored.
        :rtype: bool
        """
        gp_pln = pln.gp_pln
        gp_ax2 = gp_Ax2()
        gp_ax2.SetAxis(gp_pln.Axis())
        self.Mirror(gp_ax2)
        return True

    def rotate(self, ax1, angle):
        """
        Rotate the geometry about an axis.

        :param afem.geometry.entities.Axis1 ax1: The axis of rotation.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        angle = radians(angle)
        self.Rotate(ax1, angle)
        return True

    @classmethod
    def to_direction(cls, entity):
        """
        Convert entity to a Direction if possible.

        :param vector_like entity: An entity.

        :return: The entity if already a Direction, or a new Direction if it is
            vector_like.
        :rtype: afem.geometry.entities.Direction

        :raise TypeError: If entity cannot be converted to a Direction.
        """
        if entity is None:
            return None

        if isinstance(entity, cls):
            return entity
        elif misc_utils.is_array_like(entity):
            return cls(entity[0], entity[1], entity[2])
        elif isinstance(entity, Vector):
            return cls(entity)
        else:
            raise TypeError('Cannot convert to Direction.')

    @classmethod
    def by_xyz(cls, x, y, z):
        """
        Create a direction by *x*, *y*, and *z* components.

        :param float x: The x-component.
        :param float y: The y-component.
        :param float z: The z-component.

        :return: The new direction.
        :rtype: afem.geometry.entities.Direction
        """
        return cls(x, y, z)

    @classmethod
    def by_vector(cls, v):
        """
        Create a direction from a vector.

        :param afem.geometry.entities.Vector v: The vector.

        :return: The new direction.
        :rtype: afem.geometry.entities.Direction
        """
        return cls(v)


class Vector(gp_Vec):
    """
    Vector in 3-D space derived from ``gp_Vec``.
    """

    def __init__(self, *args):
        super(Vector, self).__init__(*args)

    def __str__(self):
        return 'Vector({0}, {1}, {2})'.format(*self.xyz)

    def __repr__(self):
        return 'Vector({0}, {1}, {2})'.format(*self.xyz)

    def __array__(self, dtype=float64, copy=True, order=None, subok=False,
                  ndmin=0):
        return array(self.xyz, dtype=dtype, copy=copy, order=order,
                     subok=subok, ndmin=ndmin)

    def __iter__(self):
        for elm in self.xyz:
            yield elm

    def __len__(self):
        return 3

    def __getitem__(self, item):
        return self.xyz[item]

    def __add__(self, other):
        return add(self, other)

    def __sub__(self, other):
        return subtract(self, other)

    @property
    def x(self):
        """
        The vector x-component.

        :Getter: Returns the x-component.
        :Setter: Sets the x-component.
        :type: float
        """
        return self.X()

    @x.setter
    def x(self, x):
        self.SetX(x)

    @property
    def y(self):
        """
        The vector y-component.

        :Getter: Returns the y-component.
        :Setter: Sets the y-component.
        :type: float
        """
        return self.Y()

    @y.setter
    def y(self, y):
        self.SetY(y)

    @property
    def z(self):
        """
        The vector z-component.

        :Getter: Returns the z-component.
        :Setter: Sets the z-component.
        :type: float
        """
        return self.Z()

    @z.setter
    def z(self, z):
        self.SetZ(z)

    @property
    def xyz(self):
        """
        :return: The vector xyz-components.
        :rtype: numpy.ndarray
        """
        return array([self.x, self.y, self.z], dtype=float64)

    @property
    def mag(self):
        """
        :return: Vector magnitude.
        :rtype: float
        """
        return self.Magnitude()

    @property
    def ijk(self):
        """
        :return: Normalized vector xyz-components.
        :rtype: numpy.ndarray
        """
        return self.xyz / self.mag

    def reverse(self):
        """
        Reverse the direction of the vector.

        :return: None.
        """
        self.Reverse()

    def normalize(self):
        """
        Normalize the vector.

        :return: None.
        """
        self.Normalize()

    def scale(self, scale):
        """
        Scale the vector.

        :param float scale: Scaling value.

        :return: None.
        """
        self.Scale(scale)

    def mirror(self, pln):
        """
        Mirror the geometry using a plane.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: *True* if mirrored.
        :rtype: bool
        """
        gp_pln = pln.gp_pln
        gp_ax2 = gp_Ax2()
        gp_ax2.SetAxis(gp_pln.Axis())
        self.Mirror(gp_ax2)
        return True

    def rotate(self, ax1, angle):
        """
        Rotate the geometry about an axis.

        :param afem.geometry.entities.Axis1 ax1: The axis of rotation.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        angle = radians(angle)
        self.Rotate(ax1, angle)
        return True

    @classmethod
    def to_vector(cls, entity):
        """
        Convert entity to a vector if possible.

        :param vector_like entity: An entity.

        :return: The entity if already a vector, or a new vector if it is
            vector_like.
        :rtype: afem.geometry.entities.Vector

        :raise TypeError: If entity cannot be converted to a Vector.
        """
        if entity is None:
            return None

        if isinstance(entity, cls):
            return entity
        elif misc_utils.is_array_like(entity):
            return cls(entity[0], entity[1], entity[2])
        elif isinstance(entity, Direction):
            return cls(entity)
        elif isinstance(entity, gp_Vec):
            return cls(entity.XYZ())
        else:
            raise TypeError('Cannot convert to Vector.')

    @classmethod
    def by_xyz(cls, x, y, z):
        """
        Create a vector by *x*, *y*, and *z* components.

        :param float x: The x-component.
        :param float y: The y-component.
        :param float z: The yzcomponent.

        :return: The new vector.
        :rtype: afem.geometry.entities.Vector
        """
        return cls(x, y, z)

    @classmethod
    def by_direction(cls, d):
        """
        Create a vector from a direction.

        :param afem.geometry.entities.Direction d: The direction.

        :return: The new vector.
        :rtype: afem.geometry.entities.Vector
        """
        return cls(d)

    @classmethod
    def by_points(cls, p1, p2):
        """
        Create a vector by two points.

        :param afem.geometry.entities.Point p1: The first point.
        :param afem.geometry.entities.Point p2: The second point.

        :return: The new vector.
        :rtype: afem.geometry.entities.Vector
        """
        return cls(p1, p2)


class Axis1(gp_Ax1):
    """
    Axis in 3-D space derived from ``gp_Ax1``.
    """

    def __init__(self, *args):
        super(Axis1, self).__init__(*args)

    @property
    def origin(self):
        """
        :return: The origin of the axis.
        :rtype: afem.geometry.entities.Point
        """
        p = self.Location()
        return Point(p.X(), p.Y(), p.Z())

    @classmethod
    def by_direction(cls, p, d):
        """
        Create an axis by an origin point and direction.

        :param afem.geometry.entities.Point p: The origin.
        :param afem.geometry.entities.Direction d: The direction.

        :return: The new axis.
        :rtype: afem.geometry.entities.Axis
        """
        return cls(p, d)


class Axis3(gp_Ax3):
    """
    Coordinate system in 3-D space derived from ``gp_Ax3``.
    """

    def __init__(self, *args):
        super(Axis3, self).__init__(*args)

    @property
    def origin(self):
        """
        :return: The origin of the coordinate system.
        :rtype: afem.geometry.entities.Point
        """
        p = self.Location()
        return Point(p.X(), p.Y(), p.Z())

    @property
    def x_dir(self):
        """
        :return: The x-direction.
        :rtype: afem.geometry.entities.Direction
        """
        return Direction(self.XDirection().XYZ())

    @property
    def y_dir(self):
        """
        :return: The y-direction.
        :rtype: afem.geometry.entities.Direction
        """
        return Direction(self.YDirection().XYZ())

    @property
    def z_dir(self):
        """
        :return: The z-direction.
        :rtype: afem.geometry.entities.Direction
        """
        return Direction(self.Direction().XYZ())

    @property
    def x_axis(self):
        """
        :return: The x-axis.
        :rtype: afem.geometry.entities.Axis1
        """
        return Axis1(self.origin, self.x_dir)

    @property
    def y_axis(self):
        """
        :return: The y-axis.
        :rtype: afem.geometry.entities.Axis1
        """
        return Axis1(self.origin, self.y_dir)

    @property
    def z_axis(self):
        """
        :return: The z-axis.
        :rtype: afem.geometry.entities.Axis1
        """
        return Axis1(self.origin, self.z_dir)

    @classmethod
    def by_normal(cls, p, n):
        """
        Create a coordinate system by an origin point and normal direction.

        :param afem.geometry.entities.Point p: The origin.
        :param afem.geometry.entities.Direction n: The normal direction.

        :return: The new coordinate system.
        :rtype: afem.geometry.entities.Axis3
        """
        return cls(p, n)

    @classmethod
    def by_xdirection(cls, p, n, x):
        """
        Create a coordinate system by an origin, a normal direction, and its
        x-direction.

        :param afem.geometry.entities.Point p: The origin.
        :param afem.geometry.entities.Direction n: The normal direction.
        :param afem.geometry.entities.Direction x: The direction of the x-axis.

        :return: The new coordinate system.
        :rtype: afem.geometry.entities.Axis3
        """
        return cls(p, n, x)


# Transient types that are wrapped.
class Geometry(ViewableItem):
    """
    Base class for geometry.

    :param obj: The geometry object.

    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_C0 C0: Only geometric continuity.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_C1 C1: Continuity of the first
        derivative.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_C2 C2: Continuity of the second
        derivative.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_C3 C3: Continuity of the third
        derivative.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_CN CN: Continuity of the n-th
        derivative.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_G1 G1: Tangent vectors on either
        side of a point on a curve are collinear with the same orientation.
    :cvar OCCT.GeomAbs.GeomAbs_Shape.GeomAbs_G2 G2: Normalized vectors on
        either side of a point on a curve are equal.

    :cvar OCCT.GeomAbs.GeomAbs_JoinType.GeomAbs_Arc ARC: Arc join type.
    :cvar OCCT.GeomAbs.GeomAbs_JoinType.GeomAbs_Tangent TANGENT: Tangent join
        type.
    :cvar OCCT.GeomAbs.GeomAbs_JoinType.GeomAbs_Intersection INTERSECT:
        Intersection join type.

    :raise TypeError: If the wrapped type of ``obj`` does not match the
        expected type.
    """
    # Expected type
    _OCC_TYPE = Geom_Geometry

    # Continuities
    C0 = GeomAbs_Shape.GeomAbs_C0
    C1 = GeomAbs_Shape.GeomAbs_C1
    C2 = GeomAbs_Shape.GeomAbs_C2
    C3 = GeomAbs_Shape.GeomAbs_C3
    CN = GeomAbs_Shape.GeomAbs_CN
    G1 = GeomAbs_Shape.GeomAbs_G1
    G2 = GeomAbs_Shape.GeomAbs_G2

    # Join types
    ARC = GeomAbs_JoinType.GeomAbs_Arc
    TANGENT = GeomAbs_JoinType.GeomAbs_Tangent
    INTERSECT = GeomAbs_JoinType.GeomAbs_Intersection

    def __init__(self, obj):
        if not isinstance(obj, self._OCC_TYPE):
            n1 = self._OCC_TYPE.__name__
            n2 = obj.__class__.__name__
            msg = ('Error wrapping an OpenCASCADE geometry type. '
                   'Expected a {} but got a {}.'.format(n1, n2))
            raise TypeError(msg)
        super(Geometry, self).__init__()
        self._object = obj

        # Set default color
        if isinstance(self, Curve):
            self.set_color(1., 0., 0.)
        elif isinstance(self, Surface):
            self.set_color(0.5, 0.5, 0.5)

    @property
    def object(self):
        """
        :return: The underlying OpenCASCADE object.
        :rtype: OCCT.Geom.Geom_Geometry
        """
        return self._object

    def translate(self, v):
        """
        Translate the geometry along the vector.

        :param vector_like v: The translation vector.

        :return: *True* if translated, *False* if not.
        :rtype: bool

        :raise TypeError: If *v* cannot be converted to a vector.
        """
        v = Vector.to_vector(v)
        self.object.Translate(v)
        return True

    def mirror(self, pln):
        """
        Mirror the geometry using a plane.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: *True* if mirrored.
        :rtype: bool
        """
        gp_pln = pln.object.Pln()
        gp_ax2 = gp_Ax2()
        gp_ax2.SetAxis(gp_pln.Axis())
        self.object.Mirror(gp_ax2)
        return True

    def scale(self, pnt, s):
        """
        Scale the geometry.

        :param point_like pnt: The reference point.
        :param float s: The scaling value.

        :return: *True* if scaled.
        :rtype: bool
        """
        pnt = Point.to_point(pnt)
        self.object.Scale(pnt, s)
        return True

    def rotate(self, ax1, angle):
        """
        Rotate the geometry about an axis.

        :param afem.geometry.entities.Axis1 ax1: The axis of rotation.
        :param float angle: The angle in degrees.

        :return: *True* if rotated.
        :rtype: bool
        """
        angle = radians(angle)
        self.object.Rotate(ax1, angle)
        return True


class Curve(Geometry):
    """
    Base class for curves around ``Geom_Curve``.

    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_Line LINE: Line type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_Circle CIRCLE: Circle type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_Ellipse ELLIPSE: Ellipse type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_Hyperbola HYPERBOLA: Hyperbola
        type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_Parabola PARABOLA: Parabola
        type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_BezierCurve BEZIER: Bezier
        curve type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_BSplineCurve BSPLINE: BSpline
        curve type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_OffsetCurve OFFSET: Offset
        curve type.
    :cvar OCCT.GeomAbs.GeomAbs_CurveType.GeomAbs_OtherCurve OTHER: Other curve
        type.
    """
    _OCC_TYPE = Geom_Curve

    # Curve types
    LINE = GeomAbs_CurveType.GeomAbs_Line
    CIRCLE = GeomAbs_CurveType.GeomAbs_Circle
    ELLIPSE = GeomAbs_CurveType.GeomAbs_Ellipse
    HYPERBOLA = GeomAbs_CurveType.GeomAbs_Hyperbola
    PARABOLA = GeomAbs_CurveType.GeomAbs_Parabola
    BEZIER = GeomAbs_CurveType.GeomAbs_BezierCurve
    BSPLINE = GeomAbs_CurveType.GeomAbs_BSplineCurve
    OFFSET = GeomAbs_CurveType.GeomAbs_OffsetCurve
    OTHER = GeomAbs_CurveType.GeomAbs_OtherCurve

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Edge
        """
        return BRepBuilderAPI_MakeEdge(self.object).Edge()

    @property
    def u1(self):
        """
        :return: The first parameter.
        :rtype: float
        """
        return self.object.FirstParameter()

    @property
    def u2(self):
        """
        :return: The last parameter.
        :rtype: float
        """
        return self.object.LastParameter()

    @property
    def is_closed(self):
        """
        :return: *True* if curve is closed, *False* if not.
        :rtype: bool
        """
        return self.object.IsClosed()

    @property
    def is_periodic(self):
        """
        :return: *True* if curve is periodic, *False* if not.
        :rtype: bool
        """
        return self.object.IsPeriodic()

    @property
    def p1(self):
        """
        :return: The first point.
        :rtype: afem.geometry.entities.Point
        """
        return self.eval(self.u1)

    @property
    def p2(self):
        """
        :return: The last point.
        :rtype: afem.geometry.entities.Point
        """
        return self.eval(self.u2)

    @property
    def length(self):
        """
        :return: Curve length.
        :rtype: float
        """
        return self.arc_length(self.u1, self.u2)

    def copy(self):
        """
        Return a new copy of the curve.

        :return: New curve.
        :rtype: afem.geometry.entities.Curve
        """
        return Curve.wrap(self.object.Copy())

    def local_to_global_u(self, *args):
        """
        Convert parameter(s) from local domain 0. <= u <= 1. to global domain
        a <= u <= b.

        :param float args: Local parameter(s).

        :return: Global parameter(s).
        :rtype: float or list(float)
        """
        return geom_utils.local_to_global_param(self.u1, self.u2, *args)

    def global_to_local_u(self, *args):
        """
        Convert parameter(s) from global domain a <= u <= b to local domain
        0. <= u <= 1.

        :param float args: Global parameter(s).

        :return: Local parameter(s).
        :rtype: float or list(float)
        """
        return geom_utils.global_to_local_param(self.u1, self.u2, *args)

    def eval(self, u):
        """
        Evaluate a point on the curve.

        :param float u: Curve parameter.

        :return: Curve point.
        :rtype: afem.geometry.entities.Point
        """
        p = Point()
        self.object.D0(u, p)
        return p

    def deriv(self, u, d=1):
        """
        Evaluate a derivative on the curve.

        :param float u: Curve parameter.
        :param int d: Derivative to evaluate.

        :return: Curve derivative.
        :rtype: afem.geometry.entities.Vector
        """
        return Vector(self.object.DN(u, d).XYZ())

    def reverse(self):
        """
        Reverse curve direction.

        :return: None.
        """
        self.object.Reverse()

    def reversed_u(self, u):
        """
        Calculate the parameter on the reversed curve.

        :param float u: Curve parameter.

        :return: Reversed parameter.
        :rtype: float
        """
        return self.object.ReversedParameter(u)

    def arc_length(self, u1, u2, tol=1.0e-7):
        """
        Calculate the curve length between the parameters.

        :param float u1: First parameter.
        :param float u2: Last parameter.
        :param float tol: The tolerance.

        :return: Curve length.
        :rtype: float
        """
        if u1 > u2:
            u1, u2 = u2, u1
        adp_crv = GeomAdaptor_Curve(self.object)
        return GCPnts_AbscissaPoint.Length_(adp_crv, u1, u2, tol)

    def invert(self, p):
        """
        Invert the point on the curve to find the parameter.

        :param point_like p: The point.

        :return: The nearest parameter on the curve.
        :rtype: float

        :raise RuntimeError: If algorithm fails.
        """
        p = Point.to_point(p)
        proj = GeomAPI_ProjectPointOnCurve(p, self.object)
        if proj.NbPoints() > 0:
            return proj.LowerDistanceParameter()
        raise RuntimeError('Failed to invert point.')

    @staticmethod
    def wrap(curve):
        """
        Wrap the OpenCASCADE curve based on its type.

        :param OCCT.Geom.Geom_Curve curve: The curve.

        :return: The wrapped curve.
        :rtype: afem.geometry.entities.Curve
        """
        if isinstance(curve, Geom_Line):
            return Line(curve)
        if isinstance(curve, Geom_Circle):
            return Circle(curve)
        if isinstance(curve, Geom_Ellipse):
            return Ellipse(curve)
        if isinstance(curve, Geom_BSplineCurve):
            return NurbsCurve(curve)
        if isinstance(curve, Geom_TrimmedCurve):
            return TrimmedCurve(curve)

        # Catch for unsupported type
        if isinstance(curve, Geom_Curve):
            return Curve(curve)

        raise TypeError('Curve type not supported.')


class Line(Curve):
    """
    Infinite line around ``Geom_Line``.
    """
    _OCC_TYPE = Geom_Line

    @classmethod
    def by_axis(cls, ax1):
        """
        Create a line from an axis.

        :param afem.geometry.entities.Axis1 ax1: The axis.

        :return: The new line.
        :rtype: afem.geometry.entities.Line
        """
        return cls(Geom_Line(ax1))

    @classmethod
    def by_direction(cls, p, d):
        """
        Create a line passing through a point along a direction.

        :param afem.geometry.entities.Point p: The point.
        :param afem.geometry.entities.Direction d: The direction.

        :return: The new line.
        :rtype: afem.geometry.entities.Line
        """
        return cls(Geom_Line(p, d))


class Circle(Curve):
    """
    Circular curve around ``Geom_Circle``.
    """
    _OCC_TYPE = Geom_Circle

    @property
    def radius(self):
        """
        :return: The radius.
        :rtype: float
        """
        return self.object.Radius()

    @property
    def center(self):
        """
        :return: The center point of the circle.
        :rtype: afem.geometry.entities.Point
        """
        return Point(self.object.Location().XYZ())

    def set_radius(self, r):
        """
        Set the radius.

        :param float r: The radius.

        :return: None.
        """
        self.object.SetRadius(r)


class Ellipse(Curve):
    """
    Elliptical curve around ``Geom_Ellipse``.
    """
    _OCC_TYPE = Geom_Ellipse

    @property
    def major_radius(self):
        """
        :return: The major radius.
        :rtype: float
        """
        return self.object.MajorRadius()

    @property
    def minor_radius(self):
        """
        :return: The minor radius.
        :rtype: float
        """
        return self.object.MinorRadius()

    def set_major_radius(self, r):
        """
        Set the major radius.

        :param float r: The major radius.

        :return: None.
        """
        self.object.SetMajorRadius(r)

    def set_minor_radius(self, r):
        """
        Set the minor radius.

        :param float r: The minor radius.

        :return: None.
        """
        self.object.SetMinorRadius(r)


class NurbsCurve(Curve):
    """
    NURBS curve around ``Geom_BSplineCurve``.
    """
    _OCC_TYPE = Geom_BSplineCurve

    @property
    def p(self):
        """
        :return: Degree of curve.
        :rtype: int
        """
        return self.object.Degree()

    @property
    def n(self):
        """
        :return: Number of control points.
        :rtype: int
        """
        return self.object.NbPoles()

    @property
    def knots(self):
        """
        :return: Knot vector.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfReal(1, self.object.NbKnots())
        self.object.Knots(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_array)

    @property
    def mult(self):
        """
        :return: Multiplicity of knot vector.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfInteger(1, self.object.NbKnots())
        self.object.Multiplicities(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_integer(tcol_array)

    @property
    def uk(self):
        """
        :return: Knot sequence.
        :rtype: numpy.ndarray
        """
        tcol_knot_seq = TColStd_Array1OfReal(1, self.object.NbPoles() +
                                             self.object.Degree() + 1)
        self.object.KnotSequence(tcol_knot_seq)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_knot_seq)

    @property
    def cp(self):
        """
        :return: Control points.
        :rtype: numpy.ndarray
        """
        tcol_array = TColgp_Array1OfPnt(1, self.object.NbPoles())
        self.object.Poles(tcol_array)
        return occ_utils.to_np_from_tcolgp_array1_pnt(tcol_array)

    @property
    def w(self):
        """
        :return: Weights of control points.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfReal(1, self.object.NbPoles())
        self.object.Weights(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_array)

    @property
    def cpw(self):
        """
        :return: Homogeneous control points.
        :rtype: numpy.ndarray
        """
        return geom_utils.homogenize_array1d(self.cp, self.w)

    def set_domain(self, u1=0., u2=1.):
        """
        Reparameterize the knot vector between *u1* and *u2*.

        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        if u1 > u2:
            return False
        tcol_knots = TColStd_Array1OfReal(1, self.object.NbKnots())
        self.object.Knots(tcol_knots)
        geom_utils.reparameterize_knots(u1, u2, tcol_knots)
        self.object.SetKnots(tcol_knots)
        return True

    def segment(self, u1, u2):
        """
        Segment the curve between parameters.

        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: *True* if segmented, *False* if not.
        :rtype: bool
        """
        if u1 > u2:
            return False
        self.object.Segment(u1, u2)
        return True

    def set_cp(self, i, cp, weight=None):
        """
        Modify the curve by setting the specified control point and weight.

        :param int i: The point index (1 <= *i* <= *n*).
        :param afem.geometry.entities.Point cp: The point.
        :param weight: The weight.
        :type weight: float or None

        :return: None.
        """
        if weight is None:
            self.object.SetPole(i, cp)
        else:
            self.object.SetPole(i, cp, weight)

    @classmethod
    def by_data(cls, cp, knots, mult, p, weights=None, is_periodic=False):
        """
        Create a NURBS curve by data.

        :param collections.Sequence(point_like) cp: Control points.
        :param collections.Sequence(float) knots: Knot vector.
        :param collections.Sequence(int) mult: Multiplicities of knot vector.
        :param int p: Degree.
        :param collections.Sequence(float) weights: Weights of control points.
        :param bool is_periodic: Flag for periodicity.
        """
        tcol_cp = occ_utils.to_tcolgp_array1_pnt(cp)
        tcol_knots = occ_utils.to_tcolstd_array1_real(knots)
        tcol_mult = occ_utils.to_tcolstd_array1_integer(mult)
        if weights is None:
            weights = [1.] * tcol_cp.Length()
        tcol_weights = occ_utils.to_tcolstd_array1_real(weights)

        geom_crv = Geom_BSplineCurve(tcol_cp, tcol_weights, tcol_knots,
                                     tcol_mult, p, is_periodic)
        return cls(geom_crv)


class TrimmedCurve(Curve):
    """
    Trimmed curve around ``Geom_TrimmedCurve``. This defines a basis curve
    limited by two parameter values.
    """
    _OCC_TYPE = Geom_TrimmedCurve

    @property
    def basis_curve(self):
        """
        :return: The basis curve.
        :rtype: afem.geometry.entities.Curve
        """
        return Curve.wrap(self.object.BasisCurve())

    def set_trim(self, u1, u2, sense=True, adjust_periodic=True):
        """
        Set the trimming parameters on the basis curve.

        :param float u1: The first parameter.
        :param float u2: The last parameter.
        :param bool sense: If the basis curve is periodic, the trimmed curve
            will have the same orientation as the basis curve if ``True`` or
            opposite if ``False``.
        :param bool adjust_periodic: If the basis curve is periodic, the bounds
            of the trimmed curve may be different from *u1* and *u2* if
            ``True``.

        :return: None.

        :raise RuntimeError: If *u1* == *u2*.
        :raise RuntimeError: If *u1* or *u2* is outside the bounds of the basis
            curve.
        """
        self.object.SetTrim(u1, u2, sense, adjust_periodic)

    @classmethod
    def by_parameters(cls, basis_curve, u1=None, u2=None, sense=True,
                      adjust_periodic=True):
        """
        Create a trimmed curve using a basis curve and limiting parameters.

        :param afem.geometry.entities.Curve basis_curve: The basis curve.
        :param float u1: The first parameter. If not provided then the first
            parameter of the basis curve will be used.
        :param float u2: The last parameter. If not provided then the last
            parameter of the basis curve will be used.
        :param bool sense: If the basis curve is periodic, the trimmed curve
            will have the same orientation as the basis curve if ``True`` or
            opposite if ``False``.
        :param bool adjust_periodic: If the basis curve is periodic, the bounds
            of the trimmed curve may be different from *u1* and *u2* if
            ``True``.

        :raise TypeError: If the basis curve is not a valid curve type.
        :raise ValueError: If *u1* >= *u2*.
        """
        if not isinstance(basis_curve, Curve):
            raise TypeError('Invalid type of basis curve.')

        if u1 is None:
            u1 = basis_curve.u1
        if u2 is None:
            u2 = basis_curve.u2

        if u1 >= u2:
            raise ValueError('Parameter values are invalid.')

        geom_crv = Geom_TrimmedCurve(basis_curve.object, u1, u2, sense,
                                     adjust_periodic)
        return cls(geom_crv)


class Surface(Geometry):
    """
    Base class for surfaces around ``Geom_Surface``.

    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_Plane PLANE: Plane type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_Cylinder CYLINDER: Cylinder
        type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_Cone CONE: Cone type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_Sphere SPHERE: Sphere type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_Torus TORUS: Torus type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_BezierSurface BEZIER: Bezier
        type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_BSplineSurface BSPLINE:
        BSpline type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_SurfaceOfRevolution \
        REVOLUTION: Revolution type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion \
        EXTRUSION: Extrusion type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_OffsetSurface OFFSET: Offset
        type.
    :cvar OCCT.GeomAbs.GeomAbs_SurfaceType.GeomAbs_OtherSurface OTHER: Other
        type.
    """
    _OCC_TYPE = Geom_Surface

    # Surface types
    PLANE = GeomAbs_SurfaceType.GeomAbs_Plane
    CYLINDER = GeomAbs_SurfaceType.GeomAbs_Cylinder
    CONE = GeomAbs_SurfaceType.GeomAbs_Cone
    SPHERE = GeomAbs_SurfaceType.GeomAbs_Sphere
    TORUS = GeomAbs_SurfaceType.GeomAbs_Torus
    BEZIER = GeomAbs_SurfaceType.GeomAbs_BezierSurface
    BSPLINE = GeomAbs_SurfaceType.GeomAbs_BSplineSurface
    REVOLUTION = GeomAbs_SurfaceType.GeomAbs_SurfaceOfRevolution
    EXTRUSION = GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion
    OFFSET = GeomAbs_SurfaceType.GeomAbs_OffsetSurface
    OTHER = GeomAbs_SurfaceType.GeomAbs_OtherSurface

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Face
        """
        return BRepBuilderAPI_MakeFace(self.object, 0.).Face()

    @property
    def adaptor(self):
        """
        :return: A surface adaptor.
        :rtype: OCCT.GeomAdaptor.GeomAdaptor_Surface
        """
        return GeomAdaptor_Surface(self.object)

    @property
    def u1(self):
        """
        :return: The first parameter in u-direction.
        :rtype: float
        """
        return self.object.U1()

    @property
    def u2(self):
        """
        :return: The last parameter in u-direction.
        :rtype: float
        """
        return self.object.U2()

    @property
    def v1(self):
        """
        :return: The first parameter in v-direction.
        :rtype: float
        """
        return self.object.V1()

    @property
    def v2(self):
        """
        :return: The last parameter in v-direction.
        :rtype: float
        """
        return self.object.V2()

    @property
    def area(self):
        """
        :return: The surface area.
        :rtype: float
        """
        return self.surface_area(self.u1, self.v1, self.u2, self.v2)

    def copy(self):
        """
        Return a new copy of the surface.

        :return: New surface.
        :rtype: afem.geometry.entities.Surface
        """
        return Surface.wrap(self.object.Copy())

    def is_planar(self, tol=1.0e-7):
        """
        Check if surface is planar.

        :param float tol: The tolerance.

        :return: *True* if planar, *False* if not.
        :rtype: bool
        """
        if isinstance(self, Plane):
            return TrimmedCurve

        return GeomLib_IsPlanarSurface(self.object, tol)

    def as_plane(self, tol=1.0e-7):
        """
        Get a plane if the surface is planar.

        :param float tol: The tolerance.

        :return: Return this plane if it's already a plane, return *None* if
            the surface is not planar, or return a plane.
        :rtype: afem.geometry.entities.Surface or None
        """
        if isinstance(self, Plane):
            return self
        tool = GeomLib_IsPlanarSurface(self.object, tol)
        if not tool.IsPlanar():
            return None

        return Surface.wrap(Geom_Plane(tool.Plan()))

    def eval(self, u=0., v=0.):
        """
        Evaluate a point on the surface.

        :param float u: Surface u-parameter.
        :param float v: Surface v-parameter.

        :return: Surface point.
        :rtype: afem.geometry.entities.Point
        """
        p = Point()
        self.object.D0(u, v, p)
        return p

    def deriv(self, u, v, nu, nv):
        """
        Evaluate a derivative on the surface.

        :param float u: Surface u-parameter.
        :param float v: Surface v-parameter.
        :param int nu: Derivative in u-direction.
        :param int nv: Derivative in v-direction.

        :return: Surface derivative.
        :rtype: afem.geometry.entities.Vector
        """
        return Vector(self.object.DN(u, v, nu, nv).XYZ())

    def norm(self, u, v):
        """
        Evaluate a normal on the surface.

        :param float u: Surface u-parameter.
        :param float v: Surface v-parameter.

        :return: Surface normal.
        :rtype: afem.geometry.entities.Vector
        """
        du = self.deriv(u, v, 1, 0)
        dv = self.deriv(u, v, 0, 1)
        return Vector(du.Crossed(dv).XYZ())

    def surface_area(self, u1, v1, u2, v2, tol=1.0e-7):
        """
        Calculate the surface area between the parameters.

        :param float u1: The first parameter in u-direction.
        :param float v1: The first parameter in v-direction.
        :param float u2: The last parameter in u-direction.
        :param float v2: The last parameter in v-direction.
        :param float tol: The tolerance.

        :return: The area.
        :rtype: float
        """
        f = BRepBuilderAPI_MakeFace(self.object, u1, u2, v1, v2, tol).Face()
        sprops = GProp_GProps()
        BRepGProp.SurfaceProperties_(f, sprops, tol)
        return sprops.Mass()

    def u_iso(self, u):
        """
        Get a iso-parametric curve at a constant u-parameter.

        :param float u: The u-parameter.

        :return: The curve.
        :rtype: afem.geometry.entities.Curve
        """
        return Curve.wrap(self.object.UIso(u))

    def v_iso(self, v):
        """
        Get a iso-parametric curve at a constant v-parameter.

        :param float v: The v-parameter.

        :return: The curve.
        :rtype: afem.geometry.entities.Curve
        """
        return Curve.wrap(self.object.VIso(v))

    def invert(self, p):
        """
        Invert the point on the surface to find the parameters.

        :param point_like p: The point.

        :return: The nearest parameters on the curve.
        :rtype: tuple(float, float)

        :raise RuntimeError: If algorithm fails.
        """
        p = Point.to_point(p)
        proj = GeomAPI_ProjectPointOnSurf(p, self.object)
        if proj.NbPoints() > 0:
            return proj.LowerDistanceParameters(0., 0.)
        raise RuntimeError('Failed to invert point.')

    @staticmethod
    def wrap(surface):
        """
        Wrap the OpenCASCADE surface based on its type.

        :param OCCT.Geom.Geom_Surface surface: The curve.

        :return: The wrapped surface.
        :rtype: afem.geometry.entities.Surface
        """
        if isinstance(surface, Geom_Plane):
            return Plane(surface)
        if isinstance(surface, Geom_BSplineSurface):
            return NurbsSurface(surface)

        # Catch for unsupported type
        if isinstance(surface, Geom_Surface):
            return Surface(surface)

        raise TypeError('Surface type not supported.')


class Plane(Surface):
    """
    Infinite plane around ``Geom_Plane``.
    """
    _OCC_TYPE = Geom_Plane

    @property
    def origin(self):
        """
        :return: The origin of the plane. This simply evaluates the plane at
            u=0 and v=0.
        :rtype: afem.geometry.entities.Point
        """
        return self.eval()

    @property
    def axis(self):
        """
        :return: The main axis (normal) of the plane.
        :rtype: afem.geometry.entities.Axis1
        """
        ax = self.gp_pln.Axis()
        return Axis1(ax.Location(), ax.Direction())

    @property
    def gp_pln(self):
        """
        :return: The underlying gp_Pln.
        :rtype: OCCT.gp.gp_Pln
        """
        return self.object.Pln()

    def distance(self, pnt):
        """
        Compute the distance between a point and this plane.

        :param point_like pnt: The point.

        :return: The distance.
        :rtype: float

        :raises TypeError: If *pnt* cannot be converted to a point.
        """
        pnt = Point.to_point(pnt)
        return self.gp_pln.Distance(pnt)

    def rotate_x(self, angle):
        """
        Rotate the plane about its x-axis.

        :param float angle: The rotation angle in degrees.

        :return: None.
        """
        pln = self.gp_pln
        pln.Rotate(pln.XAxis(), radians(angle))
        self.object.SetPln(pln)

    def rotate_y(self, angle):
        """
        Rotate the plane about its y-axis.

        :param float angle: The rotation angle in degrees.

        :return: None.
        """
        pln = self.gp_pln
        pln.Rotate(pln.YAxis(), radians(angle))
        self.object.SetPln(pln)

    @classmethod
    def by_system(cls, ax3):
        """
        Create a plane by a coordinate system.

        :param afem.geometry.entities.Axis3 ax3: The system.

        :return: The new plane.
        :rtype: afem.geometry.entities.Plane
        """
        return cls(Geom_Plane(ax3))

    @classmethod
    def by_normal(cls, p, n):
        """
        Create a plane by an origin and a normal.

        :param afem.geometry.entities.Point p: The origin.
        :param afem.geometry.entities.Direction n: The normal direction.

        :return: The new plane.
        :rtype: afem.geometry.entities.Plane
        """
        return cls(Geom_Plane(p, n))


class NurbsSurface(Surface):
    """
    NURBS surface around ``Geom_BSplineSurface``.
    """
    _OCC_TYPE = Geom_BSplineSurface

    @property
    def p(self):
        """
        :return: Degree in u-direction.
        :rtype: int
        """
        return self.object.UDegree()

    @property
    def q(self):
        """
        :return: Degree in v-direction.
        :rtype: int
        """
        return self.object.VDegree()

    @property
    def n(self):
        """
        :return: Number of control points in u-direction.
        :rtype: int
        """
        return self.object.NbUPoles()

    @property
    def m(self):
        """
        :return: Number of control points in v-direction.
        :rtype: int
        """
        return self.object.NbVPoles()

    @property
    def uknots(self):
        """
        :return: Knot vector in u-direction.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfReal(1, self.object.NbUKnots())
        self.object.UKnots(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_array)

    @property
    def umult(self):
        """
        :return: Multiplicity of knot vector in u-direction.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfInteger(1, self.object.NbUKnots())
        self.object.UMultiplicities(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_integer(tcol_array)

    @property
    def uk(self):
        """
        :return: Knot sequence in u-direction.
        :rtype: numpy.ndarray
        """
        tcol_knot_seq = TColStd_Array1OfReal(1, self.object.NbUPoles() +
                                             self.object.UDegree() + 1)
        self.object.UKnotSequence(tcol_knot_seq)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_knot_seq)

    @property
    def vknots(self):
        """
        :return: Knot vector in v-direction.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfReal(1, self.object.NbVKnots())
        self.object.VKnots(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_array)

    @property
    def vmult(self):
        """
        :return: Multiplicity of knot vector in v-direction.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array1OfInteger(1, self.object.NbVKnots())
        self.object.VMultiplicities(tcol_array)
        return occ_utils.to_np_from_tcolstd_array1_integer(tcol_array)

    @property
    def vk(self):
        """
        :return: Knot sequence in v-direction.
        :rtype: numpy.ndarray
        """
        tcol_knot_seq = TColStd_Array1OfReal(1, self.object.NbVPoles() +
                                             self.object.VDegree() + 1)
        self.object.VKnotSequence(tcol_knot_seq)
        return occ_utils.to_np_from_tcolstd_array1_real(tcol_knot_seq)

    @property
    def cp(self):
        """
        :return: Control points.
        :rtype: numpy.ndarray
        """
        tcol_array = TColgp_Array2OfPnt(1, self.object.NbUPoles(),
                                        1, self.object.NbVPoles())
        self.object.Poles(tcol_array)
        return occ_utils.to_np_from_tcolgp_array2_pnt(tcol_array)

    @property
    def w(self):
        """
        :return: Weights of control points.
        :rtype: numpy.ndarray
        """
        tcol_array = TColStd_Array2OfReal(1, self.object.NbUPoles(),
                                          1, self.object.NbVPoles())
        self.object.Weights(tcol_array)
        return occ_utils.to_np_from_tcolstd_array2_real(tcol_array)

    @property
    def cpw(self):
        """
        :return: Homogeneous control points.
        :rtype: numpy.ndarray
        """
        return geom_utils.homogenize_array2d(self.cp, self.w)

    def set_udomain(self, u1=0., u2=1.):
        """
        Reparameterize the knot vector between *u1* and *u2*.

        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        if u1 > u2:
            return False
        tcol_knots = TColStd_Array1OfReal(1, self.object.NbUKnots())
        self.object.UKnots(tcol_knots)
        geom_utils.reparameterize_knots(u1, u2, tcol_knots)
        self.object.SetUKnots(tcol_knots)
        return True

    def set_vdomain(self, v1=0., v2=1.):
        """
        Reparameterize the knot vector between *v1* and *v2*.

        :param float v1: First parameter.
        :param float v2: Last parameter.

        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        if v1 > v2:
            return False
        tcol_knots = TColStd_Array1OfReal(1, self.object.NbVKnots())
        self.object.VKnots(tcol_knots)
        geom_utils.reparameterize_knots(v1, v2, tcol_knots)
        self.object.SetVKnots(tcol_knots)
        return True

    def local_to_global_param(self, d, *args):
        """
        Convert parameter(s) from local domain 0. <= u,v <= 1. to global domain
        a <= u,v <= b.

        :param str d: Parameter direction ('u' or 'v').
        :param float args: Local parameter(s).

        :return: Global parameter(s).
        :rtype: float or list(float)
        """
        if d.lower() in ['u']:
            return geom_utils.local_to_global_param(self.u1, self.u2, *args)
        else:
            return geom_utils.local_to_global_param(self.v1, self.v2, *args)

    def global_to_local_param(self, d, *args):
        """
        Convert surface parameter(s) from global domain a <= u,v <= b to local
        domain 0. <= u,v <= 1.

        :param str d: Parameter direction ('u' or 'v').
        :param float args: Global parameter(s).

        :return: Local parameter(s).
        :rtype: float or list(float)
        """
        if d.lower() in ['u']:
            return geom_utils.global_to_local_param(self.u1, self.u2, *args)
        else:
            return geom_utils.global_to_local_param(self.v1, self.v2, *args)

    def segment(self, u1, u2, v1, v2):
        """
        Segment the surface between parameters.

        :param float u1: First parameter in u-direction.
        :param float u2: Last parameter in u-direction.
        :param float v1: First parameter in v-direction.
        :param float v2: Last parameter in v-direction.

        :return: *True* if segmented, *False* if not.
        :rtype: bool
        """
        if u1 > u2 or v1 > v2:
            return False
        self.object.CheckAndSegment(u1, u2, v1, v2)
        return True

    def locate_u(self, u, tol2d=1.0e-9, with_knot_repetition=False):
        """
        Locate the u-value in the knot sequence.

        :param float u: The parameter.
        :param int tol2d: The parametric tolerance. Used to determine if *u* is
            at an existing knot.
        :param bool with_knot_repetition: Considers location of knot value
            with repetition of multiple knot value if ``True``.

        :return: Bounding knot locations (i1, i2).
        :rtype: tuple(int)
        """
        return self.object.LocateU(u, tol2d, 0, 0, with_knot_repetition)

    def locate_v(self, v, tol2d=1.0e-9, with_knot_repetition=False):
        """
        Locate the v-value in the knot sequence.

        :param float v: The parameter.
        :param int tol2d: The parametric tolerance. Used to determine if *v* is
            at an existing knot.
        :param bool with_knot_repetition: Considers location of knot value
            with repetition of multiple knot value if ``True``.

        :return: Bounding knot locations (i1, i2).
        :rtype: tuple(int)
        """
        return self.object.LocateV(v, tol2d, 0, 0, with_knot_repetition)

    def insert_uknot(self, u, m=1, tol2d=1.0e-9):
        """
        Insert a u-value in the knot sequence.

        :param float u: The knot value.
        :param int m: If the knot already exists, the multiplicity of the
            knot is increased if the previous multiplicity is lower than
            *m*. Otherwise it does nothing.
        :param float tol2d: Parametric tolerance for comparing knot values.

        :return: None.
        """
        self.object.InsertUKnot(u, m, tol2d)

    def insert_vknot(self, v, m=1, tol2d=1.0e-9):
        """
        Insert a v-value in the knot sequence.

        :param float v: The knot value.
        :param int m: If the knot already exists, the multiplicity of the
            knot is increased if the previous multiplicity is lower than
            *m*. Otherwise it does nothing.
        :param float tol2d: Parametric tolerance for comparing knot values.

        :return: None.
        """
        self.object.InsertVKnot(v, m, tol2d)

    def set_uknots(self, uknots):
        """
        Set the knots of the surface in the u-direction.

        :param collections.Sequence(float) uknots: Knot values.

        :return: None.

        :raise ValueError: If the number of the given knots does not equal the
            number of the existing knots.
        """
        uk = occ_utils.to_tcolstd_array1_real(uknots)
        if uk.Size() != self.object.NbUKnots():
            raise ValueError('Incorrect number of knot values.')
        self.object.SetUKnots(uk)

    def set_vknots(self, vknots):
        """
        Set the knots of the surface in the u-direction.

        :param collections.Sequence(float) vknots: Knot values.

        :return: None.

        :raise ValueError: If the number of the given knots does not equal the
            number of the existing knots.
        """
        vk = occ_utils.to_tcolstd_array1_real(vknots)
        if vk.Size() != self.object.NbVKnots():
            raise ValueError('Incorrect number of knot values.')
        self.object.SetVKnots(vk)

    def set_cp(self, i, j, cp, weight=None):
        """
        Modify the surface by setting the specified control point and weight.

        :param int i: The point index in u-direction (1 <= *i* <= *n*).
        :param int j: The point index in v-direction (1 <= *j* <= *m*).
        :param afem.geometry.entities.Point cp: The point.
        :param weight: The weight.
        :type weight: float or None

        :return: None.
        """
        if weight is None:
            self.object.SetPole(i, j, cp)
        else:
            self.object.SetPole(i, j, cp, weight)

    def set_cp_row(self, u_index, cp, weights=None):
        """
        Modify the surface by setting the specified row of control points
        and weights.

        :param int u_index: The row index (1 <= *u_index* <= *n*).
        :param list(afem.geometry.entities.Point) cp: The points.
        :param weights: The weights.
        :type weights: list(float) or None

        :return: None.
        """
        tcol_gp = occ_utils.to_tcolgp_array1_pnt(cp)
        if weights is None:
            self.object.SetPoleRow(u_index, tcol_gp)
        else:
            tcol_w = occ_utils.to_tcolstd_array1_real(weights)
            self.object.SetPoleRow(u_index, tcol_gp, tcol_w)

    def set_cp_col(self, v_index, cp, weights=None):
        """
        Modify the surface by setting the specified column of control points
        and weights.

        :param int v_index: The column index (1 <= *v_index* <= *m*).
        :param list(afem.geometry.entities.Point) cp: The points.
        :param weights: The weights.
        :type weights: list(float) or None

        :return: None.
        """
        tcol_gp = occ_utils.to_tcolgp_array1_pnt(cp)
        if weights is None:
            self.object.SetPoleCol(v_index, tcol_gp)
        else:
            tcol_w = occ_utils.to_tcolstd_array1_real(weights)
            self.object.SetPoleCol(v_index, tcol_gp, tcol_w)

    @classmethod
    def by_data(cls, cp, uknots, vknots, umult, vmult, p, q, weights=None,
                is_u_periodic=False, is_v_periodic=False):
        """
        Create a NURBS surface by data.

        :param list(list(point_like)) cp: Two-dimensional list of control
            points.
        :param list(float) uknots: Knot vector for u-direction.
        :param list(float) vknots: Knot vector for v-direction.
        :param list(int) umult: Multiplicities of knot vector in u-direction.
        :param list(int) vmult: Multiplicities of knot vector in v-direction.
        :param int p: Degree in u-direction.
        :param int q: Degree in v-direction.
        :param list(list(float)) weights: Two-dimensional list of control
            point weights.
        :param bool is_u_periodic: Flag for periodicity in u-direction.
        :param bool is_v_periodic: Flag for periodicity in v-direction.
        """
        tcol_cp = occ_utils.to_tcolgp_array2_pnt(cp)
        tcol_uknots = occ_utils.to_tcolstd_array1_real(uknots)
        tcol_umult = occ_utils.to_tcolstd_array1_integer(umult)
        tcol_vknots = occ_utils.to_tcolstd_array1_real(vknots)
        tcol_vmult = occ_utils.to_tcolstd_array1_integer(vmult)
        if weights is None:
            weights = ones((tcol_cp.ColLength(), tcol_cp.RowLength()))
        tcol_weights = occ_utils.to_tcolstd_array2_real(weights)

        geom_srf = Geom_BSplineSurface(tcol_cp, tcol_uknots, tcol_vknots,
                                       tcol_umult, tcol_vmult, p, q,
                                       is_u_periodic, is_v_periodic)

        # Set the weights since using in construction causes an error.
        for i in range(1, tcol_weights.ColLength() + 1):
            for j in range(1, tcol_weights.RowLength() + 1):
                geom_srf.SetWeight(i, j, tcol_weights.Value(i, j))

        return cls(geom_srf)
