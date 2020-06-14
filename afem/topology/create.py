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
from OCCT.BRep import BRep_Builder, BRep_Tool
from OCCT.BRepAlgo import BRepAlgo
from OCCT.BRepBuilderAPI import (BRepBuilderAPI_FindPlane,
                                 BRepBuilderAPI_MakeEdge,
                                 BRepBuilderAPI_MakeFace,
                                 BRepBuilderAPI_MakePolygon,
                                 BRepBuilderAPI_MakeShell,
                                 BRepBuilderAPI_MakeWire,
                                 BRepBuilderAPI_Sewing)
from OCCT.BRepMesh import BRepMesh_IncrementalMesh
from OCCT.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
from OCCT.BRepPrimAPI import (BRepPrimAPI_MakeCylinder,
                              BRepPrimAPI_MakeHalfSpace, BRepPrimAPI_MakePrism,
                              BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeBox)
from OCCT.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCCT.TopLoc import TopLoc_Location
from OCCT.TopTools import TopTools_HSequenceOfShape
from OCCT.TopoDS import TopoDS_Compound, TopoDS_Shell

from afem.adaptor.entities import AdaptorCurve
from afem.geometry.check import CheckGeom
from afem.geometry.create import (CircleBy3Points, PlaneByApprox,
                                  PointFromParameter, PointsAlongCurveByNumber,
                                  PointsAlongCurveByDistance,
                                  PlanesAlongCurveByNumber,
                                  PlanesAlongCurveByDistance)
from afem.geometry.entities import Geometry, Curve, Plane
from afem.geometry.project import ProjectPointToCurve
from afem.topology.bop import IntersectShapes
from afem.topology.entities import (Shape, Vertex, Edge, Wire, Face, Shell,
                                    Solid, Compound)

__all__ = ["VertexByPoint",
           "EdgeByPoints", "EdgeByVertices", "EdgeByCurve", "EdgeByDrag",
           "EdgeByWireConcat",
           "WireByEdges", "WiresByConnectedEdges", "WireByPlanarOffset",
           "WiresByShape", "WireByPoints", "WireByConcat",
           "FaceBySurface", "FaceByPlane", "FaceByPlanarWire", "FaceByDrag",
           "ShellBySurface", "ShellByFaces", "ShellBySewing", "ShellByDrag",
           "SolidByShell", "SolidByPlane", "SolidByDrag",
           "CompoundByShapes",
           "HalfspaceByShape", "HalfspaceBySurface",
           "ShapeByFaces", "ShapeByDrag",
           "BoxBuilder", "BoxBySize", "BoxBy2Points",
           "CylinderByAxis",
           "SphereByRadius", "SphereBy3Points",
           "PointAlongShape", "PointsAlongShapeByNumber",
           "PointsAlongShapeByDistance",
           "PlaneByEdges", "PlaneByIntersectingShapes",
           "PlanesAlongShapeByNumber", "PlanesAlongShapeByDistance"]


# VERTEX ----------------------------------------------------------------------

class VertexByPoint(object):
    """
    Create a vertex using a point.

    :param point_like pnt: The point.
    """

    def __init__(self, pnt):
        self._v = Vertex.by_point(pnt)

    @property
    def vertex(self):
        """
        :return: The vertex.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v


# EDGE ------------------------------------------------------------------------

class EdgeByPoints(object):
    """
    Create an edge between two points.

    :param point_like p1: The first point.
    :param point_like p2: The second point.
    """

    def __init__(self, p1, p2):
        # Build
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        builder = BRepBuilderAPI_MakeEdge(p1, p2)
        self._e = Edge(builder.Edge())
        self._v1 = Vertex(builder.Vertex1())
        self._v2 = Vertex(builder.Vertex2())

    @property
    def edge(self):
        """
        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._e

    @property
    def vertex1(self):
        """
        :return: The first vertex.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v1

    @property
    def vertex2(self):
        """
        :return: The second vertex.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v2


class EdgeByVertices(object):
    """
    Create an edge between two vertices.

    :param afem.topology.entities.Vertex v1: The first vertex.
    :param afem.topology.entities.Vertex v2: The second vertex.
    """

    def __init__(self, v1, v2):
        # Build
        self._e = Edge(BRepBuilderAPI_MakeEdge(v1.object, v2.object).Edge())

    @property
    def edge(self):
        """
        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._e


class EdgeByCurve(object):
    """
    Create an edge using a curve.

    :param afem.geometry.entities.Curve crv: The curve.
    """

    def __init__(self, crv):
        # Build
        builder = BRepBuilderAPI_MakeEdge(crv.object)
        self._e = Edge(builder.Edge())
        self._v1 = Vertex(builder.Vertex1())
        self._v2 = Vertex(builder.Vertex2())

    @property
    def edge(self):
        """
        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._e

    @property
    def vertex1(self):
        """
        :return: The first vertex.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v1

    @property
    def vertex2(self):
        """
        :return: The second vertex.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v2


class EdgeByDrag(object):
    """
    Create an edge by dragging a vertex along a vector.

    :param afem.topology.entities.Vertex vertex: The vertex.
    :param vector_like v: The vector to drag the shape.
    """

    def __init__(self, vertex, v):
        v = CheckGeom.to_vector(v)
        builder = BRepPrimAPI_MakePrism(vertex.object, v)
        self._e = Edge(builder.Shape())
        self._v1 = Vertex(builder.FirstShape())
        self._v2 = Vertex(builder.LastShape())

    @property
    def edge(self):
        """
        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._e

    @property
    def first_vertex(self):
        """
        :return: The vertex at the bottom of the edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._v1

    @property
    def last_vertex(self):
        """
        :return: The vertex at the top of the edge.
        :rtype: afem.topology.entities.Vertex
        """
        return self._v2


class EdgeByWireConcat(object):
    """
    Create an edge by concatenating all the edges of a wire. The edge may
    have C0 continuity.

    :param afem.topology.entities.Wire wire: The wire.
    """

    def __init__(self, wire):
        self._e = Edge(BRepAlgo.ConcatenateWireC0_(wire.object))

    @property
    def edge(self):
        """
        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return self._e


# WIRE ------------------------------------------------------------------------


class WireByEdges(object):
    """
    Create a wire using topologically connected edges.

    :param collections.Sequence(afem.topology.entities.Edge) edges: The edges.
        They must share a common vertex to be connected.
    """

    def __init__(self, *edges):
        # Build
        builder = BRepBuilderAPI_MakeWire()
        for e in edges:
            if e is not None and not e.is_null and e.is_edge:
                builder.Add(e.object)

        self._w = Wire(builder.Wire())
        self._last_e = Edge(builder.Edge())
        self._last_v = Vertex(builder.Vertex())

    @property
    def wire(self):
        """
        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        return self._w

    @property
    def last_edge(self):
        """
        :return: The last edge added to the wire.
        :rtype: afem.topology.entities.Edge
        """
        return self._last_e

    @property
    def last_vertex(self):
        """
        :return: The last vertex added to the wire.
        :rtype: afem.topology.entities.Vertex
        """
        return self._last_v


class WiresByConnectedEdges(object):
    """
    Create wires from a list of unsorted edges.

    :param collections.Sequence(afem.topology.entities.Edge) edges: The edges.
    :param float tol: Connection tolerance. If *None* if provided then the
        maximum tolerance of all edge will be used.
    :param bool shared: Option to use only shared vertices to connect edges.
        If *False* then geometric coincidence will be also checked.
    """

    def __init__(self, edges, tol=None, shared=False):
        # Build
        hedges = TopTools_HSequenceOfShape()
        for e in edges:
            hedges.Append(e.object)

        if tol is None:
            tol = max([e.tol_max for e in edges])

        hwires = ShapeAnalysis_FreeBounds.ConnectEdgesToWires_(hedges, tol,
                                                               shared)

        wires = []
        for i in range(1, hwires.Length() + 1):
            w = Wire(hwires.Value(i))
            wires.append(w)

        self._wires = wires

    @property
    def nwires(self):
        """
        :return: Number of wires.
        :rtype: int
        """
        return len(self._wires)

    @property
    def wires(self):
        """
        :return: The wires.
        :rtype: list(afem.topology.entities.Wire)
        """
        return self._wires


class WireByPlanarOffset(object):
    """
    Create a wire by offsetting a planar wire or face.

    :param spine: The wire to offset. If a face is provided the outer wire
        will be used.
    :type spine: afem.topology.entities.Wire or afem.topology.entities.Face
    :param float distance: Offset distance in the plane.
    :param float altitude: Offset altitude normal to the plane.
    :param OCCT.GeomAbs.GeomAbs_JoinType join: Join type.
    """

    def __init__(self, spine, distance, altitude=0., join=Geometry.ARC,
                 is_open=False):
        offset = BRepOffsetAPI_MakeOffset(spine.object, join, is_open)
        offset.Perform(distance, altitude)
        self._w = Wire(offset.Shape())

    @property
    def wire(self):
        """
        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        return self._w


class WiresByShape(WiresByConnectedEdges):
    """
    Create wires by connecting all the edges of a shape. This method gathers
    all the unique edges of a shape and then uses
    :class:`.WiresByConnectedEdges`.

    :param afem.topology.entities.Shape shape: The shape.

    :raise ValueError: If no edges are found in the shape.
    """

    def __init__(self, shape):
        edges = shape.edges
        if not edges:
            raise ValueError('There are no edges in the shape to connect.')

        super(WiresByShape, self).__init__(edges)


class WireByPoints(object):
    """
    Create a polygonal wire by connecting points.

    :param collections.Sequence(point_like) pnts: The ordered points.
    :param bool close: Option to close the wire.
    """

    def __init__(self, pnts, close=False):
        builder = BRepBuilderAPI_MakePolygon()
        for p in pnts:
            p = CheckGeom.to_point(p)
            builder.Add(p)
        if close:
            builder.Close()
        self._w = Wire(builder.Wire())

    @property
    def wire(self):
        """
        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        return self._w


class WireByConcat(object):
    """
    Create a wire by concatenating all the edges of the wire. The wire may
    have C0 continuity.

    :param afem.topology.entities.Wire wire: The wire.
    """

    def __init__(self, wire):
        edge = BRepAlgo.ConcatenateWireC0_(wire.object)
        self._wire = Wire(BRepBuilderAPI_MakeWire(edge).Wire())

    @property
    def wire(self):
        """
        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        return self._wire


# FACE ------------------------------------------------------------------------

class FaceBySurface(object):
    """
    Create a face from a surface.

    :param afem.geometry.entities.Surface srf: The surface.
    :param float tol: Tolerance for resolution of degenerate edges.
    """

    def __init__(self, srf, tol=1.0e-7):
        self._f = Face(BRepBuilderAPI_MakeFace(srf.object, tol).Face())

    @property
    def face(self):
        """
        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return self._f


class FaceByPlane(object):
    """
    Create a finite face from a plane.

    :param afem.geometry.entities.Plane pln: The plane.
    :param float umin: Minimum u-parameter.
    :param float umax: Maximum u-parameter.
    :param float vmin: Minimum v-parameter.
    :param float vmax: Maximum v-parameter.
    """

    def __init__(self, pln, umin, umax, vmin, vmax):
        builder = BRepBuilderAPI_MakeFace(pln.gp_pln, umin, umax, vmin, vmax)
        self._f = Face(builder.Face())

    @property
    def face(self):
        """
        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return self._f


class FaceByPlanarWire(object):
    """
    Create a face from a planar wire.

    :param wire: The wire.
    :type wire: afem.topology.entities.Wire or afem.topology.entities.Edge or
        afem.geometry.entities.Curve
    """

    def __init__(self, wire):
        if isinstance(wire, Curve):
            wire = Wire.by_curve(wire)
        elif isinstance(wire, Edge):
            wire = Wire.by_edge(wire)
        self._f = Face(BRepBuilderAPI_MakeFace(wire.object, True).Face())

    @property
    def face(self):
        """
        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return self._f


class FaceByDrag(object):
    """
    Create a face by dragging an edge along a vector.

    :param afem.topology.entities.Edge edge: The edge.
    :param vector_like v: The vector to drag the shape.
    """

    def __init__(self, edge, v):
        v = CheckGeom.to_vector(v)
        builder = BRepPrimAPI_MakePrism(edge.object, v)
        self._f = Face(builder.Shape())
        self._e1 = Edge(builder.FirstShape())
        self._e2 = Edge(builder.LastShape())

    @property
    def face(self):
        """
        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return self._f

    @property
    def first_edge(self):
        """
        :return: The edge at the bottom of the face.
        :rtype: afem.topology.entities.Edge
        """
        return self._e1

    @property
    def last_edge(self):
        """
        :return: The edge at the top of the face.
        :rtype: afem.topology.entities.Edge
        """
        return self._e2


# SHELL -----------------------------------------------------------------------

class ShellBySurface(object):
    """
    Create a shell from a surface.

    :param afem.geometry.entities.Surface srf: The surface.
    """

    def __init__(self, srf):
        self._shell = Shell(BRepBuilderAPI_MakeShell(srf.object).Shell())

    @property
    def shell(self):
        """
        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        return self._shell


class ShellByFaces(object):
    """
    Create a shell from connected faces. This method initializes a shell an
    then simply adds the faces to it. The faces should already have shared
    edges. This is not checked.

    :param collections.Sequence(afem.topology.entities.Face) faces: The faces.
    """

    def __init__(self, faces):
        topods_shell = TopoDS_Shell()
        builder = BRep_Builder()
        builder.MakeShell(topods_shell)
        for face in faces:
            builder.Add(topods_shell, face.object)
        self._shell = Shell(topods_shell)

    @property
    def shell(self):
        """
        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        return self._shell


class ShellByDrag(object):
    """
    Create a shell by dragging a wire along a vector.

    :param afem.topology.entities.Wire wire: The wire.
    :param vector_like v: The vector to drag the shape.
    """

    def __init__(self, wire, v):
        v = CheckGeom.to_vector(v)
        builder = BRepPrimAPI_MakePrism(wire.object, v)
        self._shell = Shell(builder.Shape())
        self._w1 = Wire(builder.FirstShape())
        self._w2 = Wire(builder.LastShape())

    @property
    def shell(self):
        """
        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        return self._shell

    @property
    def first_wire(self):
        """
        :return: The wire at the bottom of the shell.
        :rtype: afem.topology.entities.Wire
        """
        return self._w1

    @property
    def last_wire(self):
        """
        :return: The wire at the top of the shell.
        :rtype: afem.topology.entities.Wire
        """
        return self._w2


class ShellBySewing(object):
    """
    Create a shell by sewing faces.

    :param collections.Sequence(afem.topology.entities.Face) faces: The faces.
    :param float tol: Sewing tolerance. If *None* the maximum tolerance of all
        the faces is used.
    :param bool cut_free_edges: Option for cutting of free edges.
    :param bool non_manifold: Option for non-manifold processing.

    :raise RuntimeError: If an invalid shape type results.
    """

    def __init__(self, faces, tol=None, cut_free_edges=False,
                 non_manifold=False):
        if tol is None:
            tol = max([f.tol_max for f in faces])

        builder = BRepBuilderAPI_Sewing(tol, True, True, cut_free_edges,
                                        non_manifold)
        for f in faces:
            builder.Add(f.object)
        builder.Perform()

        shape = Shape.wrap(builder.SewedShape())
        self._shells = []
        self._shell = None
        self._nshells = 0
        if shape.is_compound:
            self._shells = shape.shells
            self._nshells = len(self._shells)
        elif shape.is_face:
            self._shell = shape.to_shell()
            self._nshells = 1
        elif shape.is_shell:
            self._shell = shape
            self._nshells = 1
        else:
            raise RuntimeError('Invalid shape type in ShellBySewing.')

    @property
    def nshells(self):
        """
        :return: Number of shells.
        :rtype: int
        """
        return self._nshells

    @property
    def shell(self):
        """
        :return: The sewn shell.
        :rtype: afem.topology.entities.Shell
        """
        return self._shell

    @property
    def shells(self):
        """
        :return: The sewn shells if more than one is found.
        :rtype: list(afem.topology.entities.Shell)
        """
        return self._shells


# SOLID -----------------------------------------------------------------------

class SolidByShell(object):
    """
    Create a solid using a shell. The shell can either be closed (finite
    solid) or open (infinite solid).

    :param afem.topology.entities.Shell shell: The shell.
    """

    def __init__(self, shell):
        self._solid = Solid.by_shell(shell)

    @property
    def solid(self):
        """
        :return: The solid.
        :rtype: afem.topology.entities.Solid
        """
        return self._solid


class SolidByPlane(object):
    """
    Create a solid box using a plane. The plane will be extruded in the
    direction of the plane's normal. The solid's width and height will be
    centered at the plane's origin.

    :param afem.geometry.entities.Plane pln: The plane.
    :param float width: Width of the box.
    :param float height: Height of the box.
    :param float depth: Depth of the box.
    """

    def __init__(self, pln, width, height, depth):
        w = width / 2.
        h = height / 2.
        gp_pln = pln.gp_pln
        topods_face = BRepBuilderAPI_MakeFace(gp_pln, -w, w, -h, h).Face()
        vn = pln.norm(0., 0.)
        vn.Normalize()
        vn.Scale(depth)
        self._solid = Solid(BRepPrimAPI_MakePrism(topods_face, vn).Shape())

    @property
    def solid(self):
        """
        :return: The solid.
        :rtype: afem.topology.entities.Solid
        """
        return self._solid


class SolidByDrag(object):
    """
    Create a solid by dragging a face along a vector.

    :param afem.topology.entities.Face face: The face.
    :param vector_like v: The vector to drag the shape.
    """

    def __init__(self, face, v):
        v = CheckGeom.to_vector(v)
        builder = BRepPrimAPI_MakePrism(face.object, v)
        self._solid = Solid(builder.Shape())
        self._f1 = Face(builder.FirstShape())
        self._f2 = Face(builder.LastShape())

    @property
    def solid(self):
        """
        :return: The solid.
        :rtype: afem.topology.entities.Solid
        """
        return self._solid

    @property
    def first_face(self):
        """
        :return: The face at the bottom of the solid.
        :rtype: afem.topology.entities.Face
        """
        return self._f1

    @property
    def last_face(self):
        """
        :return: The face at the top of the solid.
        :rtype: afem.topology.entities.Face
        """
        return self._f2


# COMPOUND --------------------------------------------------------------------

class CompoundByShapes(object):
    """
    Create a compound from a list of shapes.

    :param shapes: List of shapes.
    :type: collections.Sequence(afem.topology.entities.Shape)
    """

    def __init__(self, shapes):
        topods_compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(topods_compound)
        for shape in shapes:
            shape = Shape.to_shape(shape)
            if isinstance(shape, Shape):
                builder.Add(topods_compound, shape.object)
        self._cp = Compound(topods_compound)

    @property
    def compound(self):
        """
        :return: The compound.
        :rtype: afem.topology.entities.Compound
        """
        return self._cp


# HALFSPACE -------------------------------------------------------------------

class HalfspaceByShape(object):
    """
    Create a half-space by a face or shell and a reference point.A half-space
    is an infinite solid, limited by a surface. It is built from a face or a
    shell, which bounds it, and with a reference point, which specifies the
    side of the surface where the matter of the half-space is located. A
    half-space is a tool commonly used in topological operations to cut another
    shape.

    :param shape: The face or shell.
    :type shape: afem.topology.entities.Face or afem.topology.entities.Shell
    :param point_like pnt: The reference point where the matter is located.

    :raise RuntimeError: If *shape* is not a face or shell.
    """

    def __init__(self, shape, pnt):
        pnt = CheckGeom.to_point(pnt)

        if shape.shape_type not in [Shape.FACE, Shape.SHELL]:
            raise RuntimeError('Invalid shape for creating half-space.')
        self._solid = Solid(BRepPrimAPI_MakeHalfSpace(shape.object,
                                                      pnt).Solid())

    @property
    def solid(self):
        """
        :return: The half-space as a solid.
        :rtype: afem.topology.entities.Solid
        """
        return self._solid


class HalfspaceBySurface(HalfspaceByShape):
    """
    Create a half-space using a surface.

    :param afem.geometry.entities.Surface: The surface.
    :param point_like pnt: The reference point where the matter is located.
    """

    def __init__(self, srf, pnt):
        face = FaceBySurface(srf).face
        super(HalfspaceBySurface, self).__init__(face, pnt)


# SHAPE -----------------------------------------------------------------------

class ShapeByFaces(object):
    """
    Create either a face or a shell from faces.

    :param collections.Sequence(afem.topology.entities.Face) faces: The faces.
    :param bool sew: Option to sew the faces if more than one face is provided.
    :param float tol: Sewing tolerance. If *None* the maximum tolerance of all
        the faces is used.
    :param bool cut_free_edges: Option for cutting of free edges.
    :param bool non_manifold: Option for non-manifold processing.
    """

    def __init__(self, faces, sew=False, tol=None, cut_free_edges=False,
                 non_manifold=False):

        if len(faces) == 1:
            self._shape = faces[0]
        elif len(faces) > 1 and not sew:
            self._shape = ShellByFaces(faces).shell
        elif len(faces) > 1 and sew:
            builder = ShellBySewing(faces, tol, cut_free_edges, non_manifold)
            if builder.nshells > 1:
                self._shape = CompoundByShapes(builder.shells).compound
            elif builder.nshells == 1:
                self._shape = builder.shell
            else:
                raise RuntimeError('Failed to create any shells.')
        else:
            raise ValueError('No faces provided.')

    @property
    def shape(self):
        """
        :return: The shape. Will be a face if there was only one face in the
            list, it will be a shell if only one shell was formed, or it will
            be a compound of shells if more than one shell was found after
            sewing.
        :rtype: afem.topology.entities.Face or afem.topology.entities.Shell or
            afem.topology.entities.Compound
        """
        return self._shape

    @property
    def is_face(self):
        """
        :return: *True* if the shape is a face, *False* if not.
        :rtype: bool
        """
        return self._shape.is_face

    @property
    def is_shell(self):
        """
        :return: *True* if the shape is a shell, *False* if not.
        :rtype: bool
        """
        return self._shape.is_shell

    @property
    def is_compound(self):
        """
        :return: *True* if the shape is a compound, *False* if not.
        :rtype: bool
        """
        return self._shape.is_compound


class ShapeByDrag(object):
    """
    Create a shape by dragging another shape along a vector.

    :param afem.topology.entities.Shape shape: The shape.
    :param vector_like v: The vector to drag the shape.
    """

    def __init__(self, shape, v):
        v = CheckGeom.to_vector(v)
        builder = BRepPrimAPI_MakePrism(shape.object, v)
        self._shape = Shape.wrap(builder.Shape())
        self._s1 = Shape.wrap(builder.FirstShape())
        self._s2 = Shape.wrap(builder.LastShape())

    @property
    def shape(self):
        """
        :return: The shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape

    @property
    def first_shape(self):
        """
        :return: The shape at the bottom.
        :rtype: afem.topology.entities.Shape
        """
        return self._s1

    @property
    def last_shape(self):
        """
        :return: The shape at the top.
        :rtype: afem.topology.entities.Shape
        """
        return self._s2

    @property
    def is_edge(self):
        """
        :return: *True* if shape is an edge, *False* if not.
        :rtype: bool
        """
        return self._shape.is_edge

    @property
    def is_face(self):
        """
        :return: *True* if shape is a face, *False* if not.
        :rtype: bool
        """
        return self._shape.is_face

    @property
    def is_shell(self):
        """
        :return: *True* if shape is a shell, *False* if not.
        :rtype: bool
        """
        return self._shape.is_shell

    @property
    def is_solid(self):
        """
        :return: *True* if shape is a solid, *False* if not.
        :rtype: bool
        """
        return self._shape.is_solid

    @property
    def is_compsolid(self):
        """
        :return: *True* if shape is a compsolid, *False* if not.
        :rtype: bool
        """
        return self._shape.is_compsolid

    @property
    def is_compound(self):
        """
        :return: *True* if shape is a compound, *False* if not.
        :rtype: bool
        """
        return self._shape.is_compound


# BOX -------------------------------------------------------------------------

class BoxBuilder(object):
    """
    Base class for building boxes.
    """

    def __init__(self, *args):
        self._builder = BRepPrimAPI_MakeBox(*args)

    @property
    def shell(self):
        """
        :return: The box as a shell.
        :rtype: afem.topology.entities.Shell
        """
        return Shell(self._builder.Shell())

    @property
    def solid(self):
        """
        :return: The box as a solid.
        :rtype: afem.topology.entities.Solid
        """
        return Solid(self._builder.Solid())

    @property
    def bottom_face(self):
        """
        :return: The bottom face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.BottomFace())

    @property
    def back_face(self):
        """
        :return: The back face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.BackFace())

    @property
    def front_face(self):
        """
        :return: The front face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.FrontFace())

    @property
    def left_face(self):
        """
        :return: The left face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.LeftFace())

    @property
    def right_face(self):
        """
        :return: The right face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.RightFace())

    @property
    def top_face(self):
        """
        :return: The top face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.TopFace())


class BoxBySize(BoxBuilder):
    """
    Build a box with the corner at (0, 0, 0) and the other at (dx, dy, dz).

    :param float dx: The corner x-location.
    :param float dy: The corner y-location.
    :param float dz: The corner z-location.
    """

    def __init__(self, dx=1., dy=1., dz=1.):
        super(BoxBySize, self).__init__(dx, dy, dz)


class BoxBy2Points(BoxBuilder):
    """
    Build a box between two points.

    :param point_like p1: The first corner point.
    :param point_like p2: The other corner point.
    """

    def __init__(self, p1, p2):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        super(BoxBy2Points, self).__init__(p1, p2)


# CYLINDER --------------------------------------------------------------------

class CylinderByAxis(object):
    """
    Create a cylinder.

    :param float radius: The radius.
    :param float height: The height.
    :param axis2: Not yet implemented. Solid will be constructed in xy-plane.

    :raise NotImplementedError: If an axis is provided.
    """

    def __init__(self, radius, height, axis2=None):
        if axis2 is None:
            self._builder = BRepPrimAPI_MakeCylinder(radius, height)
        else:
            raise NotImplementedError('Providing Axis2 not yet implemented.')

    @property
    def face(self):
        """
        :return: The lateral face of the cylinder
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.Face())

    @property
    def shell(self):
        """
        :return: The cylinder as a shell.
        :rtype: afem.topology.entities.Shell
        """
        return Shell(self._builder.Shell())

    @property
    def solid(self):
        """
        :return: The cylinder as a solid.
        :rtype: afem.topology.entities.Solid
        """
        return Solid(self._builder.Solid())


# SPHERE ----------------------------------------------------------------------

class SphereByRadius(object):
    """
    Create a sphere by a center and radius.

    :param point_like origin: The origin.
    :param float radius: The radius.
    """

    def __init__(self, origin=(0., 0., 0.), radius=1.):
        origin = CheckGeom.to_point(origin)

        self._builder = BRepPrimAPI_MakeSphere(origin, radius)

    @property
    def face(self):
        """
        :return: The sphere as a face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._builder.Face())

    @property
    def shell(self):
        """
        :return: The sphere as a shell.
        :rtype: afem.topology.entities.Shell
        """
        return Shell(self._builder.Shell())

    @property
    def solid(self):
        """
        :return: The sphere as a solid.
        :rtype: afem.topology.entities.Face
        """
        return Solid(self._builder.Solid())

    @property
    def sphere(self):
        """
        :return: The sphere primitive.
        :rtype: OCCT.BRepPrim.BRepPrim_Sphere
        """
        return self._builder.Sphere()


class SphereBy3Points(SphereByRadius):
    """
    Create a sphere using three points.

    :param point_like p1: The first point.
    :param point_like p2: The second point.
    :param point_like p3: The third point.
    """

    def __init__(self, p1, p2, p3):
        p1 = CheckGeom.to_point(p1)
        p2 = CheckGeom.to_point(p2)
        p3 = CheckGeom.to_point(p3)

        circle = CircleBy3Points(p1, p2, p3).circle

        super(SphereBy3Points, self).__init__(circle.center, circle.radius)


# POINTS ----------------------------------------------------------------------

class PointAlongShape(PointFromParameter):
    """
    Create a point along an edge or wire at a specified distance from the first
    parameter.

    :param shape: The shape.
    :type shape: afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float ds: The distance along the curve from the given parameter.
    :param float tol: Tolerance.

    :raise TypeError: If *shape* if not a curve or wire.
    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, shape, ds, tol=1.0e-7):
        adp_crv = AdaptorCurve.to_adaptor(shape)
        u0 = adp_crv.u1
        super(PointAlongShape, self).__init__(adp_crv, u0, ds, tol)


class PointsAlongShapeByNumber(PointsAlongCurveByNumber):
    """
    Create a specified number of points along an edge or wire.

    :param shape: The shape.
    :type shape: afem.topology.entities.Edge or afem.topology.entities.Wire
    :param int n: Number of points to create (*n* > 0).
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param afem.topology.entities.Shape shape1: A shape to define the first
        point. This shape is intersected with the edge or wire.
    :param afem.topology.entities.Shape shape2: A shape to define the last
        point. This shape is intersected with the edge or wire.

    :raise TypeError: If *shape* if not an edge or wire.
    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, shape, n, d1=None, d2=None, shape1=None, shape2=None):
        adp_crv = AdaptorCurve.to_adaptor(shape)

        u1 = adp_crv.u1
        u2 = adp_crv.u2

        # Adjust parameters of start/end shapes are provided
        if isinstance(shape1, Shape):
            u1 = _param_on_adp_crv(adp_crv, shape, shape1)

        if isinstance(shape2, Shape):
            u2 = _param_on_adp_crv(adp_crv, shape, shape2)

        super(PointsAlongShapeByNumber, self).__init__(adp_crv, n, u1, u2,
                                                       d1, d2)


class PointsAlongShapeByDistance(PointsAlongCurveByDistance):
    """
    Create a specified number of points along an edge or wire.

    :param shape: The shape.
    :type shape: afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float maxd: The maximum allowed spacing between points. The
        actual spacing will be adjusted to not to exceed this value.
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param afem.topology.entities.Shape shape1: A shape to define the first
        point. This shape is intersected with the edge or wire.
    :param afem.topology.entities.Shape shape2: A shape to define the last
        point. This shape is intersected with the edge or wire.
    :param int nmin: Minimum number of points to create.

    :raise TypeError: If *shape* if not a curve or wire.
    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, shape, maxd, d1=None, d2=None, shape1=None, shape2=None,
                 nmin=0):
        adp_crv = AdaptorCurve.to_adaptor(shape)

        u1 = adp_crv.u1
        u2 = adp_crv.u2

        # Adjust parameters of start/end shapes are provided
        if isinstance(shape1, Shape):
            u1 = _param_on_adp_crv(adp_crv, shape, shape1)

        if isinstance(shape2, Shape):
            u2 = _param_on_adp_crv(adp_crv, shape, shape2)

        super(PointsAlongShapeByDistance, self).__init__(adp_crv, maxd, u1, u2,
                                                         d1, d2, nmin)


# PLANES ----------------------------------------------------------------------

class PlaneByEdges(object):
    """
    Create a plane by fitting it to all the edges of a shape.

    :param afem.topology.entities.Shape shape: The shape containing the edges.
    :param float tol: Edges must be within this planar tolerance. The
        tolerance is the largest value between the value provided or the
        largest tolerance of any one of the edges in the shape.
    """

    def __init__(self, shape, tol=-1.):
        builder = BRepBuilderAPI_FindPlane(shape.object, tol)
        self._found = builder.Found()
        self._pln = None
        if self._found:
            self._pln = Plane(builder.Plane())

    @property
    def found(self):
        """
        :return: *True* if plane was found, *False* if not.
        :rtype: bool
        """
        return self._found

    @property
    def plane(self):
        """
        :return: The plane. Returns *None* if no plane was found.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlaneByIntersectingShapes(object):
    """
    Create a plane by intersection two shapes. If no additional point is
    provided, then :class:`.PlaneByEdges`. is used. If a point is provided,
    then the edges are tessellated and the point is added to this list. Then
    the tool :class:`.PlaneByApprox` is used.

    :param afem.topology.entities.Shape shape1: The first shape.
    :param afem.topology.entities.Shape shape2: The second shape.
    :param afem.geometry.entities.Point pnt: Additional point to add to the
        edges since they might be collinear.
    :param float tol: Edges must be within this planar tolerance. The
        tolerance is the largest value between the value provided or the
        largest tolerance of any one of the edges in the shape.

    :raises ValueError: If there are less than three points after
        tessellating the edges.
    """

    def __init__(self, shape1, shape2, pnt=None, tol=-1.):
        shape = IntersectShapes(shape1, shape2).shape

        pnt = CheckGeom.to_point(pnt)

        self._found = False
        self._pln = None
        if pnt is None:
            tool = PlaneByEdges(shape, tol)
            self._found = tool.found
            self._pln = tool.plane
        elif CheckGeom.is_point(pnt):
            edges = shape.edges
            pnts = [pnt]
            for edge in edges:
                BRepMesh_IncrementalMesh(edge.object, 0.001)
                loc = TopLoc_Location()
                poly3d = BRep_Tool.Polygon3D_(edge.object, loc)
                if poly3d.NbNodes == 0:
                    continue
                tcol_pnts = poly3d.Nodes()
                for i in range(1, tcol_pnts.Length() + 1):
                    gp_pnt = tcol_pnts.Value(i)
                    pnt = CheckGeom.to_point(gp_pnt)
                    pnts.append(pnt)
            if len(pnts) < 3:
                raise ValueError('Less than three points to fit a plane.')
            if tol < 0.:
                tol = 1.0e-7
            tool = PlaneByApprox(pnts, tol)
            self._found = True
            self._pln = tool.plane
        else:
            raise TypeError('Invalid input.')

    @property
    def found(self):
        """
        :return: *True* if plane was found, *False* if not.
        :rtype: bool
        """
        return self._found

    @property
    def plane(self):
        """
        :return: The plane. Returns *None* if no plane was found.
        :rtype: afem.geometry.entities.Plane
        """
        return self._pln


class PlanesAlongShapeByNumber(PlanesAlongCurveByNumber):
    """
    Create a specified number of planes along an edge or wire.

    :param shape: The shape.
    :type shape: afem.topology.entities.Edge or afem.topology.entities.Wire
    :param int n: Number of points to create (*n* > 0).
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param afem.topology.entities.Shape shape1: A shape to define the first
        point. This shape is intersected with the edge or wire.
    :param afem.topology.entities.Shape shape2: A shape to define the last
        point. This shape is intersected with the edge or wire.

    :raise TypeError: If *shape* if not an edge or wire.
    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, shape, n, ref_pln=None, d1=None, d2=None, shape1=None,
                 shape2=None):
        adp_crv = AdaptorCurve.to_adaptor(shape)

        u1 = adp_crv.u1
        u2 = adp_crv.u2

        # Adjust parameters of start/end shapes are provided
        if isinstance(shape1, Shape):
            u1 = _param_on_adp_crv(adp_crv, shape, shape1)

        if isinstance(shape2, Shape):
            u2 = _param_on_adp_crv(adp_crv, shape, shape2)

        super(PlanesAlongShapeByNumber, self).__init__(adp_crv, n, ref_pln,
                                                       u1, u2, d1, d2)


class PlanesAlongShapeByDistance(PlanesAlongCurveByDistance):
    """
    Create planes along an edge or wire by distance between them.

    :param shape: The shape.
    :type shape: afem.topology.entities.Edge or afem.topology.entities.Wire
    :param float maxd: The maximum allowed spacing between planes. The
        actual spacing will be adjusted to not to exceed this value.
    :param afem.geometry.entities.Plane ref_pln: The normal of this plane
        will be used to define the normal of all planes along the curve. If
        no plane is provided, then the first derivative of the curve will
        define the plane normal.
    :param float d1: An offset distance for the first point. This is typically
        a positive number indicating a distance from *u1* towards *u2*.
    :param float d2: An offset distance for the last point. This is typically
        a negative number indicating a distance from *u2* towards *u1*.
    :param afem.topology.entities.Shape shape1: A shape to define the first
        point. This shape is intersected with the edge or wire.
    :param afem.topology.entities.Shape shape2: A shape to define the last
        point. This shape is intersected with the edge or wire.
    :param int nmin: Minimum number of planes to create.

    :raise TypeError: If *shape* if not an edge or wire.
    :raise RuntimeError: If OCC method fails.
    """

    def __init__(self, shape, maxd, ref_pln=None, d1=None, d2=None,
                 shape1=None, shape2=None, nmin=0):
        adp_crv = AdaptorCurve.to_adaptor(shape)

        u1 = adp_crv.u1
        u2 = adp_crv.u2

        # Adjust parameters of start/end shapes are provided
        if isinstance(shape1, Shape):
            u1 = _param_on_adp_crv(adp_crv, shape, shape1)

        if isinstance(shape2, Shape):
            u2 = _param_on_adp_crv(adp_crv, shape, shape2)

        super(PlanesAlongShapeByDistance, self).__init__(adp_crv, maxd,
                                                         ref_pln, u1, u2, d1,
                                                         d2, nmin)


def _param_on_adp_crv(adp_crv, shape, other_shape):
    """
    Determine the parameter on the adaptor curve by intersecting the shape.

    :param afem.adaptor.entities.AdaptorCurve adp_crv: The curve.
    :param afem.topology.entities.Shape shape: The shape that defines the
        curve. This should be the same shape that created the adaptor curve.
    :param afem.topology.entities.Shape other_shape: The other shape that
        intersects the adaptor curve and will be used to find the parameter.

    :return: The parameter on the curve or *None* if not found.
    :rtype: float or None
    """
    shape = IntersectShapes(shape, other_shape).shape
    verts = shape.vertices
    prms = [adp_crv.u1]
    for v in verts:
        pnt = v.point
        proj = ProjectPointToCurve(pnt, adp_crv)
        if proj.npts < 1:
            continue
        umin = proj.nearest_param
        prms.append(umin)
    return min(prms)
