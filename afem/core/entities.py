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
from collections.abc import Sequence

from afem.base.entities import NamedItem, ViewableItem
from afem.config import logger
from afem.geometry.check import CheckGeom
from afem.geometry.create import (PointFromParameter, PlaneFromParameter,
                                  PlaneByPoints)
from afem.geometry.entities import TrimmedCurve
from afem.geometry.project import ProjectPointToCurve, ProjectPointToSurface
from afem.topology.bop import IntersectShapes
from afem.topology.create import (CompoundByShapes, PointsAlongShapeByNumber,
                                  PointsAlongShapeByDistance, ShellByFaces,
                                  WiresByConnectedEdges, FaceBySurface,
                                  PlanesAlongShapeByNumber,
                                  PlanesAlongShapeByDistance)
from afem.topology.distance import DistancePointToShapes
from afem.topology.entities import Shape, Edge, BBox
from afem.topology.modify import DivideC0Shape, DivideClosedShape

__all__ = ["ShapeHolder"]


class ShapeHolder(NamedItem, ViewableItem):
    """
    Core class that holds a shape plus reference geometry and common methods.

    :param str name: The name.
    :param cref: The reference curve. If it is not a :class:`.TrimmedCurve`,
        then it will be converted to one.
    :type cref: afem.geometry.entities.Curve or None
    :param sref: The reference surface.
    :type sref: afem.geometry.entities.Surface or None
    :param expected_types: The expected type(s).
    :type expected_types: Type(afem.topology.entities.Shape) or
        collections.Sequence(Type(afem.topology.entities.Shape))
    """

    def __init__(self, name, shape, cref=None, sref=None,
                 expected_types=(Shape,)):
        super(ShapeHolder, self).__init__(name)
        ViewableItem.__init__(self)

        # Set expected types
        if isinstance(expected_types, Sequence):
            self._types = expected_types
        else:
            self._types = (expected_types,)
        self._shape = None
        if shape is not None:
            self.set_shape(shape)

        # Random color
        self.random_color()

        # Set reference geometry if available
        self._cref, self._sref = None, None
        if cref is not None:
            self.set_cref(cref)
        if sref is not None:
            self.set_sref(sref)

        # Shape of reference surface for robustness
        self._sref_shape = None

    @property
    def type_name(self):
        """
        :return: The class name.
        :rtype: str
        """
        return self.__class__.__name__

    @property
    def shape(self):
        """
        :Getter: The shape.
        :Setter: Set the shape.
        :type: afem.topology.entities.Shape
        """
        return self._shape

    @shape.setter
    def shape(self, shape):
        self.set_shape(shape)

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Shape
        """
        return self.shape.object

    @property
    def cref(self):
        """
        :Getter: The reference curve.
        :Setter: Set the reference curve.
        :type: afem.geometry.entities.TrimmedCurve
        """
        return self._cref

    @cref.setter
    def cref(self, cref):
        self.set_cref(cref)

    @property
    def has_cref(self):
        """
        :return: *True* if reference curve is available, *False* if not.
        :rtype: bool
        """
        return CheckGeom.is_curve(self.cref)

    @property
    def sref(self):
        """
        :Getter: The reference surface.
        :Setter: Set the reference surface.
        :type: afem.geometry.entities.Surface
        """
        return self._sref

    @sref.setter
    def sref(self, sref):
        self.set_sref(sref)

    @property
    def has_sref(self):
        """
        :return: *True* if reference surface is available, *False* if not.
        :rtype: bool
        """
        return CheckGeom.is_surface(self.sref)

    @property
    def plane(self):
        """
        :return: The reference surface if it is a plane.
        :rtype: afem.geometry.entities.Plane

        :raise TypeError: If the reference surface is not a plane.
        """
        if CheckGeom.is_plane(self.sref):
            return self.sref
        raise TypeError('Reference surface is not a plane.')

    @property
    def sref_shape(self):
        """
        :return: The reference shape. This should be the same as the reference
            surface except it is processed (e.g., dividing closed surfaces and
            C0 boundaries) for better robustness in some operations.
        :rtype: afem.topology.entities.Shape
        """
        return self._sref_shape

    @property
    def edge_compound(self):
        """
        :return: A compound containing the edges.
        :rtype: afem.topology.entities.Compound
        """
        return CompoundByShapes(self._shape.edges).compound

    @property
    def face_compound(self):
        """
        :return: A compound containing the faces.
        :rtype: afem.topology.entities.Compound
        """
        return CompoundByShapes(self._shape.faces).compound

    def set_shape(self, shape):
        """
        Set the shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        if not isinstance(shape, self._types):
            this = self.__class__.__name__
            other = shape.__class__.__name__

            expected = []
            for type_ in self._types:
                expected.append(type_.__name__)
            if len(expected) == 1:
                expected = 'a ' + expected[0]
            else:
                expected = 'one of (' + ', '.join(expected) + ')'
            name = 'Unknown'
            if isinstance(self, NamedItem):
                name = self.name
            msg = ('Invalid shape provided for a {} object with name {}. '
                   'Got a {} but expected {}.'.format(this, name, other,
                                                      expected))
            logger.warning(msg)

        self._shape = shape

    def set_cref(self, cref):
        """
        Set the reference curve.

        :param afem.geometry.entities.Curve cref: The curve. If it is not a
            :class:`.TrimmedCurve`, then it will be converted to one. Access
            the original curve using the *basis_curve* property (i.e.,
            part.cref.basis_curve).
        :param cref: The reference curve.

        :return: None.

        :raise TypeError: If *cref* is not a curve.
        """
        if not CheckGeom.is_curve(cref):
            raise TypeError('Invalid curve type.')

        if isinstance(cref, TrimmedCurve):
            self._cref = cref
        else:
            self._cref = TrimmedCurve.by_parameters(cref)

    def set_sref(self, sref):
        """
        Set the reference surface. This method also automatically creates a
        "reference surface shape" by converting the given surface into a face,
        dividing it if it is closed, and then dividing it at C0 boundaries.
        This shape is made available to improve robustness for certain
        topological operations and can be accessed via the `sref_shape`
        property.

        :param afem.geometry.entities.Surface sref: The surface.

        :return: None.

        :raise TypeError: If *sref* is not a surface.
        """
        if not CheckGeom.is_surface(sref):
            msg = 'Invalid surface type.'
            raise TypeError(msg)

        # Set the surface
        self._sref = sref

        # Convert to a shape for robustness
        shape = FaceBySurface(sref).face
        shape = DivideClosedShape(shape).shape
        shape = DivideC0Shape(shape).shape
        self._sref_shape = shape

    def set_u1(self, u1):
        """
        Set the first parameter of the reference curve.

        :param float u1: The parameter.

        :return: None.

        :raise ValueError: If the *u1* is greater than or equal to *u2*.
        """
        if u1 >= self._cref.u2:
            msg = ('First parameter greater than or equal to second '
                   'parameter of curve.')
            raise ValueError(msg)

        self._cref.set_trim(u1, self._cref.u2)

    def set_u2(self, u2):
        """
        Set the last parameter of the reference curve.

        :param float u2: The parameter.

        :return: None.

        :raise ValueError: If the *u2* is less than or equal to *u1*.
        """
        if u2 <= self._cref.u1:
            msg = ('Second parameter less than or equal to first '
                   'parameter of curve.')
            raise ValueError(msg)

        self._cref.set_trim(self._cref.u1, u2)

    def set_p1(self, p1):
        """
        Set the first parameter of the reference curve by inverting the point.

        :param point_like p1: The point.

        :return: None.

        :raise RuntimeError: If no projection can be found.
        """
        u1 = self.cref.invert(p1)
        if u1 is None:
            msg = 'No projection found on reference curve.'
            raise RuntimeError(msg)

        self.set_u1(u1)

    def set_p2(self, p2):
        """
        Set the last parameter of the reference curve by inverting the point.

        :param point_like p2: The point.

        :return: None.

        :raise RuntimeError: If no projection can be found.
        """
        u2 = self.cref.invert(p2)
        if u2 is None:
            msg = 'No projection found on reference curve.'
            raise RuntimeError(msg)

        self.set_u2(u2)

    def trim_u1(self, entity):
        """
        Trim the first parameter of the reference curve by interesting it with
        the entity.

        :param entity: The entity.
        :type entity: afem.geometry.entities.Geometry or
            afem.topology.entities.Shape

        :return: None.

        :raise RuntimeError: If an intersection with the reference curve cannot
            be found.
        """
        shape1 = Shape.to_shape(self.cref)
        shape2 = Shape.to_shape(entity)
        bop = IntersectShapes(shape1, shape2)
        if not bop.is_done:
            raise RuntimeError('Failed to intersect reference curve.')

        pnts = [v.point for v in bop.vertices]
        p = CheckGeom.nearest_point(self.cref.p1, pnts)
        self.set_p1(p)

    def trim_u2(self, entity):
        """
        Trim the last parameter of the reference curve by interesting it with
        the entity.

        :param entity: The entity.
        :type entity: afem.geometry.entities.Geometry or
            afem.topology.entities.Shape

        :return: None.

        :raise RuntimeError: If an intersection with the reference curve cannot
            be found.
        """
        shape1 = Shape.to_shape(self.cref)
        shape2 = Shape.to_shape(entity)
        bop = IntersectShapes(shape1, shape2)
        if not bop.is_done:
            raise RuntimeError('Failed to intersect reference curve.')

        pnts = [v.point for v in bop.vertices]
        p = CheckGeom.nearest_point(self.cref.p2, pnts)
        self.set_p2(p)

    def point_from_parameter(self, ds, u0=None, is_rel=False):
        """
        Evaluate point on reference curve at a distance from a parameter.

        :param float ds: The distance.
        :param float u0: The parameter. If not provided the first parameter
            of the reference curve will be used.
        :param bool is_rel: Option specifying if the distance is absolute or
            a relative to the length of the reference curve. If relative, then
            *ds* is multiplied by the curve length to get the absolute value
            for the :class:`.PointFromParameter` method.

        :return: The point.
        :rtype: afem.geometry.entities.Point
        """
        if u0 is None:
            u0 = self._cref.u1

        if is_rel:
            ds *= self._cref.length

        return PointFromParameter(self._cref, u0, ds).point

    def points_by_number(self, n, d1=None, d2=None, shape1=None,
                         shape2=None):
        """
        Create a specified number of points along the reference curve.

        :param int n: Number of points to create (*n* > 0).
        :param float d1: An offset distance for the first point. This is
            typically a positive number indicating a distance from *u1*
            towards *u2*.
        :param float d2: An offset distance for the last point. This is
            typically a negative number indicating a distance from *u2*
            towards *u1*.
        :param afem.topology.entities.Shape shape1: A shape to define the first
            point. This shape is intersected with the edge or wire.
        :param afem.topology.entities.Shape shape2: A shape to define the last
            point. This shape is intersected with the edge or wire.

        :return: The points.
        :rtype: list(afem.geometry.entities.Point)
        """
        edge = Edge.by_curve(self._cref)
        builder = PointsAlongShapeByNumber(edge, n, d1, d2, shape1, shape2)
        return builder.points

    def points_by_distance(self, maxd, nmin=0, d1=None, d2=None, shape1=None,
                           shape2=None):
        """
        Create a points along the reference curve by distance.

        :param float maxd: The maximum allowed spacing between points. The
            actual spacing will be adjusted to not to exceed this value.
        :param int nmin: Minimum number of points to create.
        :param float d1: An offset distance for the first point. This is
            typically a positive number indicating a distance from *u1*
            towards *u2*.
        :param float d2: An offset distance for the last point. This is
            typically a negative number indicating a distance from *u2*
            towards *u1*.
        :param afem.topology.entities.Shape shape1: A shape to define the first
            point. This shape is intersected with the edge or wire.
        :param afem.topology.entities.Shape shape2: A shape to define the last
            point. This shape is intersected with the edge or wire.

        :return: The points.
        :rtype: list(afem.geometry.entities.Point)
        """
        edge = Edge.by_curve(self._cref)
        builder = PointsAlongShapeByDistance(edge, maxd, d1, d2, shape1,
                                             shape2, nmin)
        return builder.points

    def point_to_cref(self, pnt, direction=None):
        """
        Project a point to reference curve.

        :param afem.geometry.entities.Point pnt: The point. Position will be
            updated.
        :param vector_like direction: Projection direction.

        :return: *True* if projected, *False* if not.
        :rtype: bool
        """
        proj = ProjectPointToCurve(pnt, self._cref, direction, update=True)
        if not proj.success:
            return False
        return True

    def points_to_cref(self, pnts, direction=None):
        """
        Project points to the reference curve.

        :param list(afem.geometry.entities.Point) pnts: The points. Position
            will be updated.
        :param vector_like direction: Projection direction.

        :return: List of status for each point.
        :rtype: list(bool)
        """
        success = []
        for p in pnts:
            status = self.point_to_cref(p, direction)
            success.append(status)
        return success

    def point_to_sref(self, pnt, direction=None):
        """
        Project a point to reference surface.

        :param afem.geometry.entities.Point pnt: The point. Position will be
            updated.
        :param vector_like direction: Projection direction.

        :return: *True* if projected, *False* if not.
        :rtype: bool
        """
        proj = ProjectPointToSurface(pnt, self._sref, direction)
        if not proj.success:
            return False

        p = proj.nearest_point
        pnt.set_xyz(p.xyz)
        return True

    def points_to_sref(self, pnts, direction=None):
        """
        Project points to reference surface.

        :param list(afem.geometry.entities.Point) pnts: The points. Position
            will be updated.
        :param vector_like direction: Projection direction.

        :return: List of status for each point.
        :rtype: list(bool)
        """
        success = []
        for p in pnts:
            status = self.point_to_sref(p, direction)
            success.append(status)
        return success

    def plane_from_parameter(self, ds, u0=None, is_rel=False, ref_pln=None,
                             tol=1.0e-7):
        """
        Get a plane along the reference curve.

        :param float ds: The distance.
        :param float u0: The parameter. If not provided the first parameter
            of the reference curve will be used.
        :param bool is_rel: Option specifying if the distance is absolute or
            a relative to the length of the reference curve. If relative, then
            *ds* is multiplied by the curve length to get the absolute value
            for the :class:`.PlaneFromParameter` method.
        :param afem.geometry.entities.Plane ref_pln: The normal of this plane
            will be used to define the normal of the new plane. If no plane is
            provided, then the first derivative of the curve will define the
            plane normal.
        :param float tol: Tolerance.

        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        if u0 is None:
            u0 = self.cref.u1

        if is_rel:
            ds *= self.cref.length

        return PlaneFromParameter(self._cref, u0, ds, ref_pln, tol).plane

    def planes_by_number(self, n, ref_pln=None, d1=None, d2=None,
                         shape1=None, shape2=None):
        """
        Create a specified number of planes along the reference curve.

        :param int n: Number of points to create (*n* > 0).
        :param afem.geometry.entities.Plane ref_pln: The normal of this plane
            will be used to define the normal of all planes along the curve. If
            no plane is provided, then the first derivative of the curve will
            define the plane normal.
        :param float d1: An offset distance for the first point. This is
            typically a positive number indicating a distance from *u1* towards
            *u2*.
        :param float d2: An offset distance for the last point. This is
            typically a negative number indicating a distance from *u2* towards
            *u1*.
        :param afem.topology.entities.Shape shape1: A shape to define the first
            point. This shape is intersected with the reference curve.
        :param afem.topology.entities.Shape shape2: A shape to define the last
            point. This shape is intersected with the reference curve.

        :return: List of planes along the curve.
        :rtype: list(afem.geometry.entities.Plane)

        :raise TypeError: If *shape* if not an edge or wire.
        :raise RuntimeError: If OCC method fails.
        """
        edge = Edge.by_curve(self._cref)
        return PlanesAlongShapeByNumber(edge, n, ref_pln, d1, d2, shape1,
                                        shape2).planes

    def planes_by_distance(self, maxd, ref_pln=None, d1=None, d2=None,
                           shape1=None, shape2=None, nmin=0):
        """
        Create planes along the reference curve by distance between them.

        :param float maxd: The maximum allowed spacing between planes. The
            actual spacing will be adjusted to not to exceed this value.
        :param afem.geometry.entities.Plane ref_pln: The normal of this plane
            will be used to define the normal of all planes along the curve. If
            no plane is provided, then the first derivative of the curve will
            define the plane normal.
        :param float d1: An offset distance for the first point. This is
            typically a positive number indicating a distance from *u1* towards
            *u2*.
        :param float d2: An offset distance for the last point. This is
            typically a negative number indicating a distance from *u2* towards
            *u1*.
        :param afem.topology.entities.Shape shape1: A shape to define the first
            point. This shape is intersected with the reference curve.
        :param afem.topology.entities.Shape shape2: A shape to define the last
            point. This shape is intersected with the reference curve.
        :param int nmin: Minimum number of planes to create.

        :return: List of planes along the curve.
        :rtype: list(afem.geometry.entities.Plane)

        :raise TypeError: If *shape* if not an edge or wire.
        :raise RuntimeError: If OCC method fails.
        """
        edge = Edge.by_curve(self._cref)
        return PlanesAlongShapeByDistance(edge, maxd, ref_pln, d1, d2, shape1,
                                          shape2, nmin).planes

    def make_shell(self):
        """
        Attempt to make a shell from the faces of the part. This method
        only constructs the faces into a shell without checking for a valid
        shape.

        :return: A new shell from the shape of the part.
        :rtype: afem.topology.entities.Shell
        """
        return ShellByFaces(self._shape.faces).shell

    def bbox(self, tol=None):
        """
        Return a bounding box of the body.

        :param tol: Optional tolerance to enlarge the bounding box.
        :type tol: float or None

        :return: Bounding box of the body.
        :rtype: afem.topology.entities.BBox
        """
        bbox = BBox()
        bbox.add_shape(self._shape)
        if tol is not None:
            bbox.enlarge(tol)
        return bbox

    def extract_plane(self, u1, v1, u2, v2):
        """
        Extract a plane between parameters on the reference surface. The
        plane will be defined by three points. The first point is at (u1, v1),
        the second point is at (u2, v2), and the third point will be offset
        from the first point in the direction of the reference surface
        normal. The points should not be collinear.

        :param float u1: First u-parameter.
        :param float v1: First v-parameter.
        :param float u2: Second u-parameter.
        :param float v2: Second v-parameter.

        :return: The plane.
        :rtype: afem.geometry.entities.Plane
        """
        p1 = self.sref.eval(u1, v1)
        p2 = self.sref.eval(u2, v2)
        vn = self.sref.norm(u1, v1)
        p3 = p1.copy()
        p3.translate(vn)
        return PlaneByPoints(p1, p2, p3).plane

    def extract_curve(self, u1, v1, u2, v2, basis_shape=None):
        """
        Extract a trimmed curve within the reference surface between the
        parameters.

        :param float u1: First u-parameter.
        :param float v1: First v-parameter.
        :param float u2: Second u-parameter.
        :param float v2: Second v-parameter.
        :param basis_shape: The shape that will be used to intersect with
            the reference shape. If not provided a plane will be
            created using the *extract_plane()* method. The parameters
            should create points that are on or very near the intersection
            between these two shapes. If they are not they will be projected to
            the intersection which could yield unanticipated results.
        :type basis_shape: afem.geometry.entities.Surface or
            afem.topology.entities.Shape

        :return: The curve.
        :rtype: afem.geometry.entities.TrimmedCurve

        :raise RuntimeError: If method fails.
        """
        p1 = self.sref.eval(u1, v1)
        p2 = self.sref.eval(u2, v2)

        if basis_shape is None:
            basis_shape = self.extract_plane(u1, v1, u2, v2)
        basis_shape = Shape.to_shape(basis_shape)

        bop = IntersectShapes(basis_shape, self.sref_shape, approximate=True)
        shape = bop.shape

        edges = shape.edges
        builder = WiresByConnectedEdges(edges)
        if builder.nwires == 0:
            msg = 'Failed to extract any curves.'
            raise RuntimeError(msg)

        if builder.nwires == 1:
            wire = builder.wires[0]
        else:
            dist = DistancePointToShapes(p1, builder.wires)
            wire = dist.nearest_shape
        crv = wire.curve

        proj = ProjectPointToCurve(p1, crv)
        if not proj.success:
            msg = 'Failed to project point to reference curve.'
            raise RuntimeError(msg)
        u1c = proj.nearest_param

        proj = ProjectPointToCurve(p2, crv)
        if not proj.success:
            msg = 'Failed to project point to reference curve.'
            raise RuntimeError(msg)
        u2c = proj.nearest_param

        if u1c > u2c:
            crv.reverse()
            u1c, u2c = crv.reversed_u(u1c), crv.reversed_u(u2c)

        return TrimmedCurve.by_parameters(crv, u1c, u2c)
