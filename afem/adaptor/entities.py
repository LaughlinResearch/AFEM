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
from OCCT.Adaptor3d import Adaptor3d_Curve, Adaptor3d_Surface
from OCCT.BRepAdaptor import (BRepAdaptor_Curve, BRepAdaptor_CompCurve,
                              BRepAdaptor_Surface)
from OCCT.GCPnts import GCPnts_AbscissaPoint
from OCCT.GeomAdaptor import GeomAdaptor_Curve, GeomAdaptor_Surface

__all__ = ["AdaptorBase", "AdaptorCurve", "GeomAdaptorCurve",
           "EdgeAdaptorCurve", "WireAdaptorCurve",
           "AdaptorSurface", "GeomAdaptorSurface", "FaceAdaptorSurface"]


class AdaptorBase(object):
    """
    Base class for adaptor types.

    :param obj: The underlying OpenCASCADE type.
    :type obj: OCCT.Adaptor3d.Adaptor3d_Curve or
        OCCT.Adaptor3d.Adaptor3d_Surface
    """
    # Expected type
    _OCC_TYPE = None

    def __init__(self, obj):
        if not isinstance(obj, self._OCC_TYPE):
            n1 = self._OCC_TYPE.__name__
            n2 = obj.__class__.__name__
            msg = ('Error wrapping an OpenCASCADE adaptor type. '
                   'Expected a {} but got a {}.'.format(n1, n2))
            raise TypeError(msg)
        self._object = obj


class AdaptorCurve(AdaptorBase):
    """
    Base class for adaptor curves around ``Adaptor3d_Curve``.
    """
    # Expected type
    _OCC_TYPE = Adaptor3d_Curve

    @property
    def object(self):
        """
        :return: The underlying OpenCASCADE object.
        :rtype: OCCT.Adaptor3d.Adaptor3d_Curve
        """
        return self._object

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
    def continuity(self):
        """
        :return: The continuity of the adaptor curve.
        :rtype: OCCT.GeomAbs.GeomAbs_Shape
        """
        return self.object.Continuity()

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
    def type(self):
        """
        :return: The type of the adaptor curve.
        :rtype: OCCT.GeomAbs.GeomAbs_CurveType
        """
        return self.object.GetType()

    @property
    def length(self):
        """
        :return: Curve length.
        :rtype: float
        """
        return self.arc_length(self.u1, self.u2)

    def eval(self, u):
        """
        Evaluate a point on the curve.

        :param float u: Curve parameter.

        :return: Curve point.
        :rtype: afem.geometry.entities.Point
        """
        from afem.geometry.entities import Point

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
        from afem.geometry.entities import Vector

        return Vector(self.object.DN(u, d).XYZ())

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
        return GCPnts_AbscissaPoint.Length_(self.object, u1, u2, tol)

    @staticmethod
    def to_adaptor(entity):
        """
        Convert the entity to an adaptor type if possible.

        :param entity: The entity.
        :type entity: afem.adaptor.entities.AdaptorCurve or
            afem.geometry.entities.Curve or afem.topology.entities.Edge or
            afem.topology.entities.Wire

        :return: The adaptor curve.
        :rtype: afem.adaptor.entities.AdaptorCurve

        :raise TypeError: If *entity* cannot be converted to an adaptor.
        """
        # Avoid circular import
        from afem.geometry.entities import Curve
        from afem.topology.entities import Edge, Wire

        if isinstance(entity, AdaptorCurve):
            return entity
        if isinstance(entity, Curve):
            return GeomAdaptorCurve.by_curve(entity)
        if isinstance(entity, Edge):
            return EdgeAdaptorCurve.by_edge(entity)
        if isinstance(entity, Wire):
            return WireAdaptorCurve.by_wire(entity)

        raise TypeError('Could not convert to adaptor curve.')


class GeomAdaptorCurve(AdaptorCurve):
    """
    Geometry adaptor curve around ``GeomAdaptor_Curve``.
    """
    # Expected type
    _OCC_TYPE = GeomAdaptor_Curve

    @classmethod
    def by_curve(cls, curve, u1=None, u2=None):
        """
        Create by a curve.

        :param afem.geometry.entities.Curve curve: The curve.
        :param float u1: First parameter.
        :param float u2: Last parameter.

        :return: The adaptor curve.
        :rtype: afem.adaptor.entities.GeomAdaptorCurve

        .. note::

            Both *u1* and *u2* must be provided in order to be used.
        """
        if None not in [u1, u2]:
            adp_crv = GeomAdaptor_Curve(curve.object, u1, u2)
        else:
            adp_crv = GeomAdaptor_Curve(curve.object)

        return cls(adp_crv)


class EdgeAdaptorCurve(AdaptorCurve):
    """
    Edge adaptor curve around ``BRepAdaptor_Curve``.
    """
    # Expected type
    _OCC_TYPE = BRepAdaptor_Curve

    @classmethod
    def by_edge(cls, edge, face=None):
        """
        Create by an edge.

        :param afem.topology.entities.Edge edge: The edge.
        :param afem.topology.entities.Edge face: The face the edge lies in.

        :return: The adaptor curve.
        :rtype: afem.adaptor.entities.EdgeAdaptorCurve
        """
        if face is None:
            adp_crv = BRepAdaptor_Curve(edge.object)
        else:
            adp_crv = BRepAdaptor_Curve(edge.object, face.object)
        return cls(adp_crv)


class WireAdaptorCurve(AdaptorCurve):
    """
    Wire adaptor curve around ``BRepAdaptor_CompCurve``.
    """
    # Expected type
    _OCC_TYPE = BRepAdaptor_CompCurve

    @classmethod
    def by_wire(cls, wire, curvilinear_knots=False):
        """
        Create by a wire.

        :param afem.topology.entities.Wire wire: The wire.
        :param bool curvilinear_knots: Option to determine the knot values of
            the curve by their curvilinear abscissa.

        :return: The adaptor curve.
        :rtype: afem.adaptor.entities.WireAdaptorCurve
        """
        adp_crv = BRepAdaptor_CompCurve(wire.object, curvilinear_knots)
        return cls(adp_crv)


class AdaptorSurface(AdaptorBase):
    """
    Base class for adaptor surfaces around ``Adaptor3d_Surface``.
    """
    # Expected type
    _OCC_TYPE = Adaptor3d_Surface

    @property
    def object(self):
        """
        :return: The underlying OpenCASCADE object.
        :rtype: OCCT.Adaptor3d.Adaptor3d_Surface
        """
        return self._object

    @property
    def u1(self):
        """
        :return: The first parameter in u-direction.
        :rtype: float
        """
        return self.object.FirstUParameter()

    @property
    def u2(self):
        """
        :return: The last parameter in u-direction.
        :rtype: float
        """
        return self.object.LastUParameter()

    @property
    def v1(self):
        """
        :return: The first parameter in v-direction.
        :rtype: float
        """
        return self.object.FirstVParameter()

    @property
    def v2(self):
        """
        :return: The last parameter in v-direction.
        :rtype: float
        """
        return self.object.LastVParameter()

    @property
    def type(self):
        """
        :return: The type of the adaptor surface.
        :rtype: OCCT.GeomAbs.GeomAbs_SurfaceType
        """
        return self.object.GetType()

    def eval(self, u=0., v=0.):
        """
        Evaluate a point on the surface.

        :param float u: Surface u-parameter.
        :param float v: Surface v-parameter.

        :return: Surface point.
        :rtype: afem.geometry.entities.Point
        """
        from afem.geometry.entities import Point

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
        from afem.geometry.entities import Vector

        return Vector(self.object.DN(u, v, nu, nv).XYZ())

    def norm(self, u, v):
        """
        Evaluate a normal on the surface.

        :param float u: Surface u-parameter.
        :param float v: Surface v-parameter.

        :return: Surface normal.
        :rtype: afem.geometry.entities.Vector
        """
        from afem.geometry.entities import Vector

        du = self.deriv(u, v, 1, 0)
        dv = self.deriv(u, v, 0, 1)
        return Vector(du.Crossed(dv).XYZ())

    @staticmethod
    def to_adaptor(entity):
        """
        Convert the entity to an adaptor type if possible.

        :param entity: The entity.
        :type entity: afem.adaptor.entities.AdaptorSurface or
            afem.geometry.entities.Surface or afem.topology.entities.Face

        :return: The adaptor surface.
        :rtype: afem.adaptor.entities.AdaptorSurface

        :raise TypeError: If *entity* cannot be converted to an adaptor.
        """
        # Avoid circular import
        from afem.geometry.entities import Surface
        from afem.topology.entities import Face

        if isinstance(entity, AdaptorSurface):
            return entity
        if isinstance(entity, Surface):
            return GeomAdaptorSurface.by_surface(entity)
        if isinstance(entity, Face):
            return FaceAdaptorSurface.by_face(entity)

        raise TypeError('Could not convert to adaptor surface.')


class GeomAdaptorSurface(AdaptorSurface):
    """
    Geometry adaptor surface around ``GeomAdaptor_Surface``.
    """
    # Expected type
    _OCC_TYPE = GeomAdaptor_Surface

    @classmethod
    def by_surface(cls, surface, u1=None, u2=None, v1=None, v2=None,
                   tolu=0., tolv=0.):
        """
        Create by a surface.

        :param afem.geometry.entities.Surface surface: The surface.
        :param u1: The lower u-parameter.
        :param u2: The upper u-parameter.
        :param v1: The lower v-parameter.
        :param v2: The upper v-parameter.
        :param tolu: Tolerance for u-direction.
        :param tolv: Tolerance for v-direction.

        :return: The adaptor surface.
        :rtype: afem.adaptor.entities.GeomAdaptorSurface

        .. note::

            All *u1*, *u2*, *v1*, and *v2* must be provided to be used.
        """
        if None not in [u1, u2, v1, v2]:
            adp_srf = GeomAdaptor_Surface(surface.object, u1, u2, v1, v2,
                                          tolu, tolv)
        else:
            adp_srf = GeomAdaptor_Surface(surface.object)

        return cls(adp_srf)


class FaceAdaptorSurface(AdaptorSurface):
    """
    Face adaptor surface around ``BRepAdaptor_Surface``.
    """
    # Expected type
    _OCC_TYPE = BRepAdaptor_Surface

    @classmethod
    def by_face(cls, face, restrict=True):
        """
        Create by a face.

        :param afem.topology.entities.Face face: The face.
        :param bool restrict: Option to restruct the uv-domain of the surface
            to the boundary of the face.

        :return: The adaptor surface.
        :rtype: afem.adaptor.entities.FaceAdaptorSurface
        """
        adp_srf = BRepAdaptor_Surface(face.object, restrict)
        return cls(adp_srf)
