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
from math import radians, tan
from warnings import warn

from afem.adaptor.entities import WireAdaptorCurve
from afem.geometry.check import CheckGeom
from afem.geometry.create import (NurbsCurveByPoints,
                                  PlanesBetweenPlanesByNumber,
                                  PlanesBetweenPlanesByDistance,
                                  PlanesAlongCurveByNumber,
                                  PlanesAlongCurveByDistance,
                                  PlaneByOrientation,
                                  PlanesAlongCurveAndSurfaceByDistance)
from afem.geometry.entities import Curve, Surface
from afem.structure.entities import (Part, CurvePart, Beam1D, SurfacePart,
                                     WingPart, Spar, Rib, FuselagePart,
                                     Bulkhead, Floor, Frame, Skin,
                                     Stiffener1D, Stiffener2D, Stringer,
                                     Beam2D)
from afem.structure.utils import shape_of_entity
from afem.topology.bop import (IntersectShapes, CommonShapes, CutShapes,
                               TrimOpenWire)
from afem.topology.create import (EdgeByCurve, WiresByConnectedEdges,
                                  FaceBySurface, WireByPlanarOffset,
                                  FaceByPlanarWire, WireByConcat,
                                  EdgeByPoints, CompoundByShapes, WiresByShape)
from afem.topology.distance import DistanceShapeToShape
from afem.topology.entities import Shape, Edge, Wire
from afem.topology.explore import ExploreFreeEdges
from afem.topology.modify import SewShape
from afem.topology.offset import SweepShapeWithNormal, SweepShape
from afem.topology.props import LengthOfShapes

__all__ = [
    "CreatePartByName", "PartBuilder", "PartsBuilder",
    "CurvePartByShape",
    "Beam1DByShape", "Beam1DByCurve", "Beam1DByPoints",
    "SurfacePartByShape", "SurfacePartByParameters", "SurfacePartByPoints",
    "SurfacePartByEnds", "SurfacePartBetweenShapes",
    "SurfacePartsBetweenPlanesByNumber", "SurfacePartsBetweenPlanesByDistance",
    "SurfacePartsAlongCurveByNumber", "SurfacePartsAlongCurveByDistance",
    "SparByShape", "SparByParameters", "SparByPoints", "SparByEnds",
    "SparBetweenShapes", "SparsBetweenPlanesByNumber",
    "SparsBetweenPlanesByDistance", "SparsAlongCurveByNumber",
    "SparsAlongCurveByDistance",
    "RibByShape", "RibByParameters", "RibByPoints", "RibByEnds",
    "RibBetweenShapes", "RibByOrientation", "RibsBetweenPlanesByNumber",
    "RibsBetweenPlanesByDistance", "RibsAlongCurveByNumber",
    "RibsAlongCurveByDistance", "RibsAlongCurveAndSurfaceByDistance",
    "BulkheadByShape",
    "FloorByShape",
    "FrameByPlane", "FramesByPlanes",
    "FramesBetweenPlanesByNumber", "FramesBetweenPlanesByDistance",
    "SkinBySolid", "SkinByBody",
    "StringerByShape",
    "Beam2DBySweep"
]

_type_to_part = {
    'Part': Part,
    'CurvePart': CurvePart,
    'Beam1D': Beam1D,
    'SurfacePart': SurfacePart,
    'WingPart': WingPart,
    'Spar': Spar,
    'Rib': Rib,
    'FuselagePart': FuselagePart,
    'Bulkhead': Bulkhead,
    'Floor': Floor,
    'Frame': Frame,
    'Skin': Skin,
    'Stiffener1D': Stiffener1D,
    'Stiffener2D': Stiffener2D,
    'Stringer': Stringer,
    'Beam2D': Beam2D
}


# PART ------------------------------------------------------------------------
class CreatePartByName(object):
    """
    Create a part by its type name. This is a tool mostly for loading models
    and it not recommended for general use.

    :param str type_: The type of part.
    """

    def __init__(self, type_, *args, **kwargs):
        self._part = _type_to_part[type_](*args, **kwargs)

    @property
    def part(self):
        """
        :return: The part.
        :rtype: afem.structure.entities.Part
        """
        return self._part


class PartBuilder(object):
    """
    Base class for creating a part.

    :param str name: Part name.
    :param cref: The reference curve.
    :type cref: afem.geometry.entities.Curve or None
    :param sref: The reference surface.
    :type sref: afem.geometry.entities.Surface or None
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, shape, cref=None, sref=None, group=None,
                 type_=Part):
        self._part = type_(name, shape, cref, sref, group)

    @property
    def part(self):
        """
        :return: The part. The return type will be defined by ``type_``.
        :rtype: afem.structure.entities.SurfacePart
        """
        return self._part


class PartsBuilder(object):
    """
    Base class for creating multiple parts.
    """

    def __init__(self):
        self._parts = []
        self._ds = None
        self._next_index = 1

    @property
    def nparts(self):
        """
        :return: The number of parts.
        :rtype: int
        """
        return len(self._parts)

    @property
    def parts(self):
        """
        :return: The parts.
        :rtype: list(afem.structure.entities.Part)
        """
        return self._parts

    @property
    def spacing(self):
        """
        :return: The spacing between first and second parts.
        :rtype: float
        """
        return self._ds

    @property
    def next_index(self):
        """
        :return: The next index.
        :rtype: int
        """
        return self._next_index


# CURVE PART ------------------------------------------------------------------

class CurvePartByShape(PartBuilder):
    """
    Create a curve part using a shape.

    :param str name: Part name.
    :param shape: The shape.
    :type shape: afem.geometry.entities.Curve or afem.topology.entities.Shape
    :param afem.geometry.entities.Curve cref: The reference curve. If not
        provided then a curve will be extracted from the shape.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, shape, cref=None, group=None, type_=CurvePart):
        if isinstance(shape, Curve):
            cref = shape
            shape = EdgeByCurve(shape).edge

        if cref is None:
            cref = shape.curve

        super(CurvePartByShape, self).__init__(name, shape, cref, None, group,
                                               type_)


# BEAM1D ----------------------------------------------------------------------

class Beam1DByShape(CurvePartByShape):
    """
    Create a beam using a shape.

    :param str name: Part name.
    :param shape: The shape.
    :type shape: afem.geometry.entities.Curve or afem.topology.entities.Shape
    :param afem.geometry.entities.Curve cref: The reference curve. If not
        provided then a curve will be extracted from the shape.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, shape, cref=None, group=None):
        super(Beam1DByShape, self).__init__(name, shape, cref, group, Beam1D)


class Beam1DByCurve(Beam1DByShape):
    """
    Create a beam using a curve.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, group=None):
        e = EdgeByCurve(crv).edge
        super(Beam1DByCurve, self).__init__(name, e, crv, group)


class Beam1DByPoints(Beam1DByCurve):
    """
    Create a beam between two points.

    :param str name: Part name.
    :param point_like p1: First point.
    :param point_like p2: Second point.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, p1, p2, group=None):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        c = NurbsCurveByPoints([p1, p2]).curve
        super(Beam1DByPoints, self).__init__(name, c, group)


# SURFACE PART ----------------------------------------------------------------
class SurfacePartByShape(PartBuilder):
    """
    Create a surface part using a basis shape.

    :param str name: Part name.
    :param afem.topology.entities.Shape basis_shape: The basis shape.
    :param afem.oml.entities.Body body: The body.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.

    .. note::

        * The reference curve of the part will be untrimmed in this method. It
          will be determined by the intersection between *shape* and the body
          reference shape.

        * The reference surface of the part will be taken as the underlying
          surface of the largest face in the basis shape.
    """

    def __init__(self, name, basis_shape, body, group=None, type_=SurfacePart):
        # Build reference curve
        section = IntersectShapes(basis_shape, body.sref_shape,
                                  approximate=True)
        edges = section.shape.edges
        wires = WiresByConnectedEdges(edges).wires
        w = LengthOfShapes(wires).longest_shape
        cref = None
        if isinstance(w, (Edge, Wire)):
            cref = w.curve
            # Orient cref so that p1 is nearest (u1, v1) on the body
            umin, vmin = body.sref.u1, body.sref.v1
            p0 = body.sref.eval(umin, vmin)
            p1 = cref.eval(cref.u1)
            p2 = cref.eval(cref.u2)
            d1 = p0.distance(p1)
            d2 = p0.distance(p2)
            if d2 < d1:
                cref.reverse()

        # Build part shape
        common = CommonShapes(basis_shape, body.shape)
        if not common.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = common.shape

        # Get reference surface
        sref = shape.surface

        super(SurfacePartByShape, self).__init__(name, shape, cref, sref,
                                                 group, type_)


class SurfacePartByParameters(PartBuilder):
    """
    Create a surface part between wing parameters.

    :param str name: Part name.
    :param float u1: Starting point u-parameter.
    :param float v1: Starting point v-parameter.
    :param float u2: Ending point u-parameter.
    :param float v2: Ending point v-parameter.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.

    :raise RuntimeError: If Boolean operation fails.

    .. note::

        * If a basis shape is provided, then the starting and ending parameters
          should be near the intersection between this shape and the body
          reference shape.
    """

    def __init__(self, name, u1, v1, u2, v2, body, basis_shape=None,
                 group=None, type_=SurfacePart):

        # Determine reference surface and basis shape
        if basis_shape is None:
            pln = body.extract_plane(u1, v1, u2, v2)
            sref = pln
            basis_shape = FaceBySurface(pln).face
        elif isinstance(basis_shape, Surface):
            sref = basis_shape
            basis_shape = FaceBySurface(sref).face
        else:
            sref = basis_shape.surface

        # Extract cref
        cref = body.extract_curve(u1, v1, u2, v2, basis_shape)

        # Build part shape
        common = CommonShapes(basis_shape, body.shape)
        if not common.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = common.shape

        super(SurfacePartByParameters, self).__init__(name, shape, cref, sref,
                                                      group, type_)


class SurfacePartByPoints(SurfacePartByParameters):
    """
    Create a rib between two points.

    :param str name: Part name.
    :param point_like p1: Starting point.
    :param point_like p2: End point.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.

    .. note::

        * The starting and ending points should be on or near the body
          reference surface since they are projected to find parameters.

        * If a basis shape is provided, then the starting and ending points
          should be near the intersection between this shape and the body
          reference shape.
    """

    def __init__(self, name, p1, p2, body, basis_shape=None, group=None,
                 type_=SurfacePart):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)

        # Invert points
        u1, v1 = body.sref.invert(p1)
        u2, v2 = body.sref.invert(p2)

        # Use SparByParameters
        super(SurfacePartByPoints, self).__init__(name, u1, v1, u2, v2, body,
                                                  basis_shape, group, type_)


class SurfacePartByEnds(SurfacePartByParameters):
    """
    Create a surface part by defining its endpoints which can be either points
    or parameters on a reference surface.

    :param str name: Part name.
    :param e1: Start point as a point or surface parameters (u1, v1).
    :type e1: point_like or collections.Sequence[float]
    :param e2: End point as a point or surface parameters (u2, v2).
    :type e2: point_like or collections.Sequence[float]
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.

    :raise TypeError: If *e1* or *e2* are not *point_like* or a sequence of
        two surface parameters.
    """

    def __init__(self, name, e1, e2, body, basis_shape=None, group=None,
                 type_=SurfacePart):
        if len(e1) == 2:
            u1, v1 = e1
        elif CheckGeom.is_point_like(e1):
            u1, v1 = body.sref.invert(e1)
        else:
            raise TypeError('Invalid type for e1.')

        if len(e2) == 2:
            u2, v2 = e2
        elif CheckGeom.is_point_like(e2):
            u2, v2 = body.sref.invert(e2)
        else:
            raise TypeError('Invalid type for e2.')

        super(SurfacePartByEnds, self).__init__(name, u1, v1, u2, v2, body,
                                                basis_shape, group, type_)


class SurfacePartBetweenShapes(SurfacePartByPoints):
    """
    Create a surface part between shapes.

    :param str name: Part name.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.body body: The body.
    :param basis_shape: The basis shape.
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, shape1, shape2, body, basis_shape, group=None,
                 type_=SurfacePart):
        if isinstance(basis_shape, Surface):
            basis_shape = FaceBySurface(basis_shape).face

        shape1 = shape_of_entity(shape1)
        shape2 = shape_of_entity(shape2)

        wing_basis_edges = IntersectShapes(basis_shape, body.sref_shape).shape
        p1_shape = IntersectShapes(shape1, wing_basis_edges).shape
        p2_shape = IntersectShapes(shape2, wing_basis_edges).shape
        v1 = p1_shape.vertices[0]
        v2 = p2_shape.vertices[0]
        p1 = v1.point
        p2 = v2.point

        super(SurfacePartBetweenShapes, self).__init__(name, p1, p2, body,
                                                       basis_shape, group,
                                                       type_)


class SurfacePartsBetweenPlanesByNumber(PartsBuilder):
    """
    Create a specified number of surface parts between two planes.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, pln1, pln2, n, shape1, shape2, body, d1=None,
                 d2=None, first_index=1, delimiter=' ', group=None,
                 type_=SurfacePart):
        super(SurfacePartsBetweenPlanesByNumber, self).__init__()

        n = int(n)
        first_index = int(first_index)

        builder = PlanesBetweenPlanesByNumber(pln1, pln2, n, d1, d2)

        self._ds = builder.spacing
        for pln in builder.planes:
            basis_shape = FaceBySurface(pln).face
            label_indx = delimiter.join([name, str(first_index)])
            part = SurfacePartBetweenShapes(label_indx, shape1, shape2, body,
                                            basis_shape, group, type_).part
            first_index += 1
            self._parts.append(part)
        self._next_index = first_index


class SurfacePartsBetweenPlanesByDistance(PartsBuilder):
    """
    Create surface parts between two planes using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param float maxd: The maximum allowed spacing. The actual spacing will
        be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of parts to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, pln1, pln2, maxd, shape1, shape2, body, d1=None,
                 d2=None, nmin=0, first_index=1, delimiter=' ', group=None,
                 type_=SurfacePart):
        super(SurfacePartsBetweenPlanesByDistance, self).__init__()

        first_index = int(first_index)

        builder = PlanesBetweenPlanesByDistance(pln1, pln2, maxd, d1, d2, nmin)

        self._ds = builder.spacing
        for pln in builder.planes:
            basis_shape = FaceBySurface(pln).face
            label_indx = delimiter.join([name, str(first_index)])
            part = SurfacePartBetweenShapes(label_indx, shape1, shape2, body,
                                            basis_shape, group, type_).part
            first_index += 1
            self._parts.append(part)
        self._next_index = first_index


class SurfacePartsAlongCurveByNumber(PartsBuilder):
    """
    Create a specified number of surface parts along a curve.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, crv, n, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None, type_=SurfacePart):
        super(SurfacePartsAlongCurveByNumber, self).__init__()

        n = int(n)
        first_index = int(first_index)

        builder = PlanesAlongCurveByNumber(crv, n, ref_pln, u1, u2, d1, d2,
                                           tol)

        self._ds = builder.spacing
        for pln in builder.planes:
            basis_shape = FaceBySurface(pln).face
            label_indx = delimiter.join([name, str(first_index)])
            part = SurfacePartBetweenShapes(label_indx, shape1, shape2, body,
                                            basis_shape, group, type_).part
            first_index += 1
            self._parts.append(part)
        self._next_index = first_index


class SurfacePartsAlongCurveByDistance(PartsBuilder):
    """
    Create surface parts along a curve using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    :param Type[afem.structure.entities.Part] type_: The type of part to
        create.
    """

    def __init__(self, name, crv, maxd, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, nmin=0, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None, type_=SurfacePart):
        super(SurfacePartsAlongCurveByDistance, self).__init__()

        first_index = int(first_index)

        builder = PlanesAlongCurveByDistance(crv, maxd, ref_pln, u1, u2, d1,
                                             d2, nmin, tol)

        self._ds = builder.spacing
        for pln in builder.planes:
            basis_shape = FaceBySurface(pln).face
            label_indx = delimiter.join([name, str(first_index)])
            part = SurfacePartBetweenShapes(label_indx, shape1, shape2, body,
                                            basis_shape, group, type_).part
            first_index += 1
            self._parts.append(part)
        self._next_index = first_index


# SPAR ------------------------------------------------------------------------

class SparByShape(SurfacePartByShape):
    """
    Create a spar using a basis shape.

    :param str name: Part name.
    :param basis_shape: The basis shape or surface.
    :type basis_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param afem.oml.entities.Body body: The body.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    .. note::

        * The reference curve of the part will be untrimmed in this method. It
          will be determined by the intersection between *shape* and the body
          reference shape.

        * The reference surface of the part will be taken as the underlying
          surface of the largest face in the basis shape.
    """

    def __init__(self, name, basis_shape, body, group=None):
        if isinstance(basis_shape, Surface):
            basis_shape = FaceBySurface(basis_shape).face
        super(SparByShape, self).__init__(name, basis_shape, body, group, Spar)


class SparByParameters(SurfacePartByParameters):
    """
    Create a spar between wing parameters.

    :param str name: Part name.
    :param float u1: Starting point u-parameter.
    :param float v1: Starting point v-parameter.
    :param float u2: Ending point u-parameter.
    :param float v2: Ending point v-parameter.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, u1, v1, u2, v2, body, basis_shape=None,
                 group=None):
        super(SparByParameters, self).__init__(name, u1, v1, u2, v2, body,
                                               basis_shape, group, Spar)


class SparByPoints(SurfacePartByPoints):
    """
    Create a spar between two points. This method inverts the starting and
    ending points and then uses :class:`.SparByParameters`.

    :param str name: Part name.
    :param point_like p1: Starting point.
    :param point_like p2: End point.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    .. note::

        * The starting and ending points should be on or near the body
          reference surface since they are projected to find parameters.

        * If a basis shape is provided, then the starting and ending points
          should be near the intersection between this shape and the body
          reference shape.
    """

    def __init__(self, name, p1, p2, body, basis_shape=None, group=None):
        super(SparByPoints, self).__init__(name, p1, p2, body, basis_shape,
                                           group, Spar)


class SparByEnds(SurfacePartByEnds):
    """
    Create a spar by defining its endpoints which can be either points or
    parameters on a reference surface.

    :param str name: Part name.
    :param e1: Start point as a point or surface parameters (u1, v1).
    :type e1: point_like or collections.Sequence[float]
    :param e2: End point as a point or surface parameters (u2, v2).
    :type e2: point_like or collections.Sequence[float]
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, e1, e2, body, basis_shape=None, group=None):
        super(SparByEnds, self).__init__(name, e1, e2, body, basis_shape,
                                         group, Spar)


class SparBetweenShapes(SurfacePartBetweenShapes):
    """
    Create a spar between shapes.

    :param str name: Part name.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.body body: The body.
    :param basis_shape: The basis shape.
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, shape1, shape2, body, basis_shape, group=None):
        super(SparBetweenShapes, self).__init__(name, shape1, shape2, body,
                                                basis_shape, group, Spar)


class SparsBetweenPlanesByNumber(SurfacePartsBetweenPlanesByNumber):
    """
    Create a specified number of planar spars between two planes.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, n, shape1, shape2, body, d1=None,
                 d2=None, first_index=1, delimiter=' ', group=None):
        super(SparsBetweenPlanesByNumber, self).__init__(name, pln1, pln2, n,
                                                         shape1, shape2, body,
                                                         d1, d2, first_index,
                                                         delimiter, group,
                                                         Spar)


class SparsBetweenPlanesByDistance(SurfacePartsBetweenPlanesByDistance):
    """
    Create planar spars between two planes using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param float maxd: The maximum allowed spacing. The actual spacing will
        be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of parts to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, maxd, shape1, shape2, body, d1=None,
                 d2=None, nmin=0, first_index=1, delimiter=' ', group=None):
        super(SparsBetweenPlanesByDistance, self).__init__(name, pln1, pln2,
                                                           maxd, shape1,
                                                           shape2, body, d1,
                                                           d2, nmin,
                                                           first_index,
                                                           delimiter,
                                                           group, Spar)


class SparsAlongCurveByNumber(SurfacePartsAlongCurveByNumber):
    """
    Create a specified number of planar spars along a curve.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, n, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None):
        super(SparsAlongCurveByNumber, self).__init__(name, crv, n, shape1,
                                                      shape2, body, ref_pln,
                                                      u1, u2, d1, d2,
                                                      first_index, delimiter,
                                                      tol, group, Spar)


class SparsAlongCurveByDistance(SurfacePartsAlongCurveByDistance):
    """
    Create planar spars along a curve using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, maxd, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, nmin=0, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None):
        super(SparsAlongCurveByDistance, self).__init__(name, crv, maxd,
                                                        shape1, shape2, body,
                                                        ref_pln, u1, u2, d1,
                                                        d2, nmin, first_index,
                                                        delimiter, tol, group,
                                                        Spar)


# RIB -------------------------------------------------------------------------

class RibByShape(SurfacePartByShape):
    """
    Create a rib using a basis shape.

    :param str name: Part name.
    :param basis_shape: The basis shape or surface.
    :type basis_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param afem.oml.entities.Body body: The body.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    .. note::

        * The reference curve of the part will be untrimmed in this method. It
          will be determined by the intersection between *shape* and the body
          reference shape.

        * The reference surface of the part will be taken as the underlying
          surface of the largest face in the basis shape.
    """

    def __init__(self, name, basis_shape, body, group=None):
        if isinstance(basis_shape, Surface):
            basis_shape = FaceBySurface(basis_shape).face
        super(RibByShape, self).__init__(name, basis_shape, body, group, Rib)


class RibByParameters(SurfacePartByParameters):
    """
    Create a rib between wing parameters.

    :param str name: Part name.
    :param float u1: Starting point u-parameter.
    :param float v1: Starting point v-parameter.
    :param float u2: Ending point u-parameter.
    :param float v2: Ending point v-parameter.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, u1, v1, u2, v2, body, basis_shape=None,
                 group=None):
        super(RibByParameters, self).__init__(name, u1, v1, u2, v2, body,
                                              basis_shape, group, Rib)


class RibByPoints(SurfacePartByPoints):
    """
    Create a rib between two points.

    :param str name: Part name.
    :param point_like p1: Starting point.
    :param point_like p2: End point.
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    .. note::

        * The starting and ending points should be on or near the body
          reference surface since they are projected to find parameters.

        * If a basis shape is provided, then the starting and ending points
          should be near the intersection between this shape and the body
          reference shape.
    """

    def __init__(self, name, p1, p2, body, basis_shape=None, group=None):
        super(RibByPoints, self).__init__(name, p1, p2, body, basis_shape,
                                          group, Rib)


class RibByEnds(SurfacePartByEnds):
    """
    Create a rib by defining its endpoints which can be either points or
    parameters on a reference surface.

    :param str name: Part name.
    :param e1: Start point as a point or surface parameters (u1, v1).
    :type e1: point_like or collections.Sequence[float]
    :param e2: End point as a point or surface parameters (u2, v2).
    :type e2: point_like or collections.Sequence[float]
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape to define the shape of the part. If
        not provided, then a plane will be defined between (u1, v1),
        (u2, v2), and a point translated from the reference surface normal at
        (u1, v1).
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, e1, e2, body, basis_shape=None, group=None):
        super(RibByEnds, self).__init__(name, e1, e2, body, basis_shape,
                                        group, Rib)


class RibBetweenShapes(SurfacePartBetweenShapes):
    """
    Create a rib between shapes.

    :param str name: Part name.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param basis_shape: The basis shape.
    :type basis_shape: afem.geometry.entities.Surface or
        afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, shape1, shape2, body, basis_shape, group=None):
        super(RibBetweenShapes, self).__init__(name, shape1, shape2, body,
                                               basis_shape, group, Rib)


class RibByOrientation(RibByShape):
    """
    Create a planar rib using rotation angles.

    :param str name: Part name.
    :param point_like origin: The origin of the plane after rotating and
        translating from the global origin.
    :param afem.oml.entities.Body body: The body.
    :param float alpha: Rotation in degrees about global x-axis.
    :param float beta: Rotation in degrees about global y-axis.
    :param float gamma: Rotation in degrees about global z-axis.
    :param str axes: The axes for the original plane before rotation and
        translation ('xy', 'xz', 'yz').
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, origin, body, alpha=0., beta=0., gamma=0.,
                 axes='xz', group=None):
        pln = PlaneByOrientation(origin, axes, alpha, beta, gamma).plane

        super(RibByOrientation, self).__init__(name, pln, body, group)


class RibsBetweenPlanesByNumber(SurfacePartsBetweenPlanesByNumber):
    """
    Create a specified number of planar ribs between two planes.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, n, shape1, shape2, body, d1=None,
                 d2=None, first_index=1, delimiter=' ', group=None):
        super(RibsBetweenPlanesByNumber, self).__init__(name, pln1, pln2, n,
                                                        shape1, shape2, body,
                                                        d1, d2, first_index,
                                                        delimiter, group, Rib)


class RibsBetweenPlanesByDistance(SurfacePartsBetweenPlanesByDistance):
    """
    Create planar ribs between two planes using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param float maxd: The maximum allowed spacing. The actual spacing will
        be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of parts to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, maxd, shape1, shape2, body, d1=None,
                 d2=None, nmin=0, first_index=1, delimiter=' ', group=None):
        super(RibsBetweenPlanesByDistance, self).__init__(name, pln1, pln2,
                                                          maxd, shape1,
                                                          shape2, body, d1,
                                                          d2, nmin,
                                                          first_index,
                                                          delimiter,
                                                          group, Rib)


class RibsAlongCurveByNumber(SurfacePartsAlongCurveByNumber):
    """
    Create a specified number of planar ribs along a curve.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param int n: The number of parts.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, n, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None):
        super(RibsAlongCurveByNumber, self).__init__(name, crv, n, shape1,
                                                     shape2, body, ref_pln,
                                                     u1, u2, d1, d2,
                                                     first_index, delimiter,
                                                     tol, group, Rib)


class RibsAlongCurveByDistance(SurfacePartsAlongCurveByDistance):
    """
    Create planar ribs along a curve using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of planes to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, maxd, shape1, shape2, body, ref_pln=None,
                 u1=None, u2=None, d1=None, d2=None, nmin=0, first_index=1,
                 delimiter=' ', tol=1.0e-7, group=None):
        super(RibsAlongCurveByDistance, self).__init__(name, crv, maxd,
                                                       shape1, shape2, body,
                                                       ref_pln, u1, u2, d1,
                                                       d2, nmin, first_index,
                                                       delimiter, tol, group,
                                                       Rib)


class RibsAlongCurveAndSurfaceByDistance(PartsBuilder):
    """
    Create planar ribs along a curve and surface using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Curve crv: The curve.
    :param afem.geometry.entities.Surface srf: The surface.
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param shape1: Starting shape.
    :type shape1: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param shape2: Ending shape.
    :type shape2: afem.topology.entities.Shape or afem.geometry.entities.Curve
        or afem.geometry.entities.Surface or afem.structure.entities.Part
    :param afem.oml.entities.Body body: The body.
    :param float u1: The parameter of the first plane (default=crv.u1).
    :param float u2: The parameter of the last plane (default=crv.u2).
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param float rot_x: The rotation angles of each plane along their local
        x-axis in degrees.
    :param float rot_y: The rotation angles of each plane along their local
        y-axis in degrees.
    :param int nmin: Minimum number of planes to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param float tol: Tolerance.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, crv, srf, maxd, shape1, shape2, body,
                 u1=None, u2=None, d1=None, d2=None, rot_x=None, rot_y=None,
                 nmin=0, first_index=1, delimiter=' ', tol=1.0e-7, group=None):
        super(RibsAlongCurveAndSurfaceByDistance, self).__init__()

        first_index = int(first_index)

        builder = PlanesAlongCurveAndSurfaceByDistance(crv, srf, maxd, u1, u2,
                                                       d1, d2, nmin, tol)
        if rot_x is not None:
            builder.rotate_x(rot_x)
        if rot_y is not None:
            builder.rotate_y(rot_y)

        self._ds = builder.spacing
        for pln in builder.planes:
            basis_shape = FaceBySurface(pln).face
            label_indx = delimiter.join([name, str(first_index)])
            part = RibBetweenShapes(label_indx, shape1, shape2, body,
                                    basis_shape, group).part
            first_index += 1
            self._parts.append(part)
        self._next_index = first_index


# BULKHEAD --------------------------------------------------------------------

class BulkheadByShape(PartBuilder):
    """
    Create a bulkhead using a shape.

    :param str name: Part name.
    :param basis_shape: The basis shape or surface.
    :type basis_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param afem.oml.entities.Body body: The body.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    :raise RuntimeError: If Boolean operation failed.
    """

    def __init__(self, name, basis_shape, body, group=None):
        sref = None
        if isinstance(basis_shape, Surface):
            sref = basis_shape
            basis_shape = FaceBySurface(sref).face

        if sref is None:
            sref = basis_shape.surface

        # Build part shape
        common = CommonShapes(basis_shape, body.shape)
        if not common.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = common.shape

        super(BulkheadByShape, self).__init__(name, shape, None, sref, group,
                                              Bulkhead)


# FLOOR -----------------------------------------------------------------------

class FloorByShape(PartBuilder):
    """
    Create a floor using a shape.

    :param str name: Part name.
    :param basis_shape: The basis shape or surface.
    :type basis_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param afem.oml.entities.Body body: The body.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    :raise RuntimeError: If Boolean operation failed.
    """

    def __init__(self, name, basis_shape, body, group=None):
        sref = None
        if isinstance(basis_shape, Surface):
            sref = basis_shape
            basis_shape = FaceBySurface(sref).face

        if sref is None:
            sref = basis_shape.surface

        # Build part shape
        common = CommonShapes(basis_shape, body.shape)
        if not common.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = common.shape

        super(FloorByShape, self).__init__(name, shape, None, sref, group,
                                           Floor)


# FRAME -----------------------------------------------------------------------

class FrameByPlane(PartBuilder):
    """
    Create a frame using a plane. A plane is required since the shape of
    frame is formed using an offset algorithm that requires planar shapes.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln: The plane.
    :param afem.oml.entities.Body body: The body.
    :param float height: The height. The absolute value is used.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None

    :raise RuntimeError: If Boolean operation failed.
    """

    def __init__(self, name, pln, body, height, group=None):
        basis_shape = FaceBySurface(pln).face

        # Find initial shape
        common = CommonShapes(basis_shape, body.shape)
        if not common.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = common.shape

        # Get outer (free) edge of shape which should be a closed wire. Use
        #  the longest wire if necessary.
        closed_wires = ExploreFreeEdges(shape).closed_wires
        if len(closed_wires) > 1:
            outer_wire = LengthOfShapes(closed_wires).longest_shape
        else:
            outer_wire = closed_wires[0]

        # Offset the outer wire and concatenate it
        inner_wire = WireByPlanarOffset(outer_wire, -abs(height)).wire
        inner_wire = WireByConcat(inner_wire).wire

        # Create inner and outer shapes and cut one from the other
        inner_face = FaceByPlanarWire(inner_wire).face
        cut = CutShapes(shape, inner_face)
        if not cut.is_done:
            msg = 'Boolean operation failed.'
            raise RuntimeError(msg)
        shape = cut.shape

        super(FrameByPlane, self).__init__(name, shape, None, pln, group,
                                           Frame)


class FramesByPlanes(PartsBuilder):
    """
    Create frames using a list of planes. This method uses
    :class:`.FrameByPlane`.

    :param str name: Part name.
    :param list(afem.geometry.entities.Plane) plns: The planes.
    :param afem.oml.entities.Body body: The body.
    :param float height: The height.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, plns, body, height, first_index=1,
                 delimiter=' ', group=None):
        super(FramesByPlanes, self).__init__()

        first_index = int(first_index)

        for pln in plns:
            label_indx = delimiter.join([name, str(first_index)])
            frame = FrameByPlane(label_indx, pln, body, height, group).part
            first_index += 1
            self._parts.append(frame)
        self._next_index = first_index


class FramesBetweenPlanesByNumber(PartsBuilder):
    """
    Create a specified number of frames between two planes.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param int n: The number of parts.
    :param afem.oml.entities.Body body: The body.
    :param float height: The height.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, n, body, height, d1=None,
                 d2=None, first_index=1, delimiter=' ', group=None):
        super(FramesBetweenPlanesByNumber, self).__init__()

        n = int(n)
        first_index = int(first_index)

        builder = PlanesBetweenPlanesByNumber(pln1, pln2, n, d1, d2)

        self._ds = builder.spacing
        for pln in builder.planes:
            label_indx = delimiter.join([name, str(first_index)])
            frame = FrameByPlane(label_indx, pln, body, height, group).part
            first_index += 1
            self._parts.append(frame)
        self._next_index = first_index


class FramesBetweenPlanesByDistance(PartsBuilder):
    """
    Create frames between two planes using a maximum spacing.

    :param str name: Part name.
    :param afem.geometry.entities.Plane pln1: The first plane.
    :param afem.geometry.entities.Plane pln2: The second plane.
    :param float maxd: The maximum allowed spacing. The actual spacing will
        be adjusted to not to exceed this value.
    :param afem.oml.entities.Body body: The body.
    :param float height: The height.
    :param float d1: An offset distance for the first plane. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last plane. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param int nmin: Minimum number of parts to create.
    :param int first_index: The first index appended to the part name as
        parts are created successively.
    :param str delimiter: The delimiter to use when joining the part name
        with the index. The final part name will be
        'name' + 'delimiter' + 'index'.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, pln1, pln2, maxd, body, height, d1=None,
                 d2=None, nmin=0, first_index=1, delimiter=' ', group=None):
        super(FramesBetweenPlanesByDistance, self).__init__()

        first_index = int(first_index)

        builder = PlanesBetweenPlanesByDistance(pln1, pln2, maxd, d1, d2, nmin)

        self._ds = builder.spacing
        for pln in builder.planes:
            label_indx = delimiter.join([name, str(first_index)])
            frame = FrameByPlane(label_indx, pln, body, height, group).part
            first_index += 1
            self._parts.append(frame)
        self._next_index = first_index


# SKIN ------------------------------------------------------------------------

class SkinBySolid(PartBuilder):
    """
    Create a skin part from the outer shell of the solid.

    :param str name: Part name.
    :param afem.topology.entities.Solid solid: The solid.
    :param bool copy: Option to copy the outer shell.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, solid, copy=False, group=None):
        shell = solid.outer_shell
        if copy:
            shell = shell.copy(False)

        super(SkinBySolid, self).__init__(name, shell, None, None, group, Skin)


class SkinByBody(SkinBySolid):
    """
    Create a skin part from the outer shell of a body.

    :param str name: Part name.
    :param afem.oml.entities.Body body: The body.
    :param bool copy: Option to copy the outer shell.
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, body, copy=False, group=None):
        super(SkinByBody, self).__init__(name, body.shape, copy, group)


# STRINGER --------------------------------------------------------------------

class StringerByShape(PartBuilder):
    """
    Create a Stringer using a basis shape.

    :param str name: Part name.
    :param basis_shape: The basis shape that will define the path of the
        stringer.
    :type basis_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param support_shape: The shape that will defines the normal direction
        along the path.
    :type support_shape: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param float height: The height.
    :param float angle: The runout angle at each end.
    :param shape1: The starting shape.
    :type shape1: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param shape2: The ending shape.
    :type shape2: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, basis_shape, support_shape, height, angle=30.,
                 shape1=None, shape2=None, group=None):

        # Convert to shapes
        support_shape = Shape.to_shape(support_shape)
        shape1 = Shape.to_shape(shape1)
        shape2 = Shape.to_shape(shape2)

        # Intersect to find spine
        spine = IntersectShapes(support_shape, basis_shape, True).shape

        # Build the wire
        tool = WiresByShape(spine)
        if tool.nwires == 0:
            msg = 'Failed to find a wire for the stringer.'
            raise RuntimeError(msg)
        elif tool.nwires > 1:
            msg = 'Found more than one wire for the stringer. Using longest.'
            warn(msg, RuntimeWarning)
            spine = LengthOfShapes(tool.wires).longest_shape
        else:
            spine = tool.wires[0]

        # Trim spine between shapes
        if (shape1 and shape2) is not None:
            spine = TrimOpenWire(spine, shape1, shape2).trimmed_wire
        elif shape1 is not None and shape2 is None:
            spine = TrimOpenWire(spine, shape1).last_wire
        elif shape1 is None and shape2 is not None:
            spine = TrimOpenWire(spine, shape2).first_wire

        # Determine length along spine given height and angle
        dx = height / tan(radians(angle))

        # Make the profiles
        adp_crv = WireAdaptorCurve.by_wire(spine, True)

        p0 = adp_crv.eval(dx)
        dss = DistanceShapeToShape(support_shape, p0)
        vn = CheckGeom.to_vector(dss.normal_on_shape1(1))
        vn.scale(height)
        p1 = p0.copy()
        p1.translate(vn)
        profile1 = EdgeByPoints(p0, p1).edge

        p0 = adp_crv.eval(adp_crv.u2 - dx)
        dss = DistanceShapeToShape(support_shape, p0)
        vn = CheckGeom.to_vector(dss.normal_on_shape1(1))
        vn.scale(height)
        p1 = p0.copy()
        p1.translate(vn)
        profile2 = EdgeByPoints(p0, p1).edge

        # Trim spine at the first and last profiles
        trim = TrimOpenWire(spine, profile1, profile2)
        first_wire = trim.first_wire
        middle_wire = trim.trimmed_wire
        last_wire = trim.last_wire

        # Make first segment
        pipe = SweepShapeWithNormal(first_wire, support_shape)
        pipe.add_profile(trim.all_vertices[0], True)
        pipe.add_profile(profile1, True)
        pipe.build()
        first_shape = pipe.shape

        # Make middle segment
        pipe = SweepShapeWithNormal(middle_wire, support_shape)
        pipe.add_profile(profile1, True)
        pipe.add_profile(profile2, True)
        pipe.build()
        middle_shape = pipe.shape

        # Make last segment
        pipe = SweepShapeWithNormal(last_wire, support_shape)
        pipe.add_profile(profile2, True)
        pipe.add_profile(trim.all_vertices[-1], True)
        pipe.build()
        last_shape = pipe.shape

        # Sew segments
        cmp = CompoundByShapes(
            [first_shape, middle_shape, last_shape]).compound
        shape = SewShape(cmp).sewed_shape

        super(StringerByShape, self).__init__(name, shape, None, None, group,
                                              Stringer)


# BEAM2D ----------------------------------------------------------------------

class Beam2DBySweep(PartBuilder):
    """
    Create a Beam2D by sweeping a profile along a path.

    :param str name: Part name.
    :param spine: The path for the sweep.
    :type spine: afem.geometry.entities.Curve or afem.topology.entities.Edge or
        afem.topology.entities.Wire
    :param profile: The profile to sweep.
    :type profile: afem.geometry.entities.Curve or afem.topology.entities.Shape
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """

    def __init__(self, name, spine, profile, group=None):
        cref = None
        if CheckGeom.is_curve(spine):
            cref = spine
            spine = Wire.by_curve(spine)
        elif isinstance(spine, Edge):
            cref = spine.curve
            spine = Wire.by_edge(spine)

        profile = Shape.to_shape(profile)

        if cref is None:
            cref = spine.curve

        tool = SweepShape(spine, profile)

        super(Beam2DBySweep, self).__init__(name, tool.shape, cref, None,
                                            group, Beam2D)
