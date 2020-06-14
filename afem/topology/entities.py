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
from math import sqrt

from OCCT.BRep import BRep_Tool, BRep_Builder
from OCCT.BRepAdaptor import BRepAdaptor_Curve
from OCCT.BRepBndLib import BRepBndLib
from OCCT.BRepBuilderAPI import (BRepBuilderAPI_Copy,
                                 BRepBuilderAPI_MakeVertex,
                                 BRepBuilderAPI_MakeEdge,
                                 BRepBuilderAPI_MakeWire,
                                 BRepBuilderAPI_MakeFace,
                                 BRepBuilderAPI_MakePolygon)
from OCCT.BRepClass3d import BRepClass3d
from OCCT.BRepGProp import BRepGProp
from OCCT.BRepTools import BRepTools, BRepTools_WireExplorer
from OCCT.Bnd import Bnd_Box
from OCCT.GProp import GProp_GProps
from OCCT.GeomConvert import GeomConvert_CompCurveToBSplineCurve
from OCCT.ShapeAnalysis import ShapeAnalysis_Edge, ShapeAnalysis_ShapeTolerance
from OCCT.ShapeFix import ShapeFix_Solid
from OCCT.TopAbs import TopAbs_ShapeEnum
from OCCT.TopExp import TopExp
from OCCT.TopTools import TopTools_IndexedMapOfShape
from OCCT.TopoDS import (TopoDS, TopoDS_Vertex, TopoDS_Edge, TopoDS_Wire,
                         TopoDS_Face, TopoDS_Shell, TopoDS_Solid,
                         TopoDS_Compound, TopoDS_CompSolid, TopoDS_Shape,
                         TopoDS_Iterator)

from afem.base.entities import ViewableItem
from afem.geometry.check import CheckGeom
from afem.geometry.entities import Point, Curve, Surface

__all__ = ["Shape", "Vertex", "Edge", "Wire", "Face", "Shell", "Solid",
           "Compound", "CompSolid",
           "BBox"]


class Shape(ViewableItem):
    """
    Shape.

    :param OCCT.TopoDS.TopoDS_Shape shape: The underlying shape.

    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_SHAPE SHAPE: Shape type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_VERTEX VERTEX: Vertex type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_EDGE EDGE: Edge type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_WIRE WIRE: Wire type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_FACE FACE: Face type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_SHELL SHELL: Shell type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_SOLID SOLID: Solid type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_COMPSOLID COMPSOLID: CompSolid
        type.
    :cvar OCCT.TopAbs.TopAbs_ShapeEnum.TopAbs_COMPOUND COMPOUND: Compound type.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Shape``.
    """

    SHAPE = TopAbs_ShapeEnum.TopAbs_SHAPE
    VERTEX = TopAbs_ShapeEnum.TopAbs_VERTEX
    EDGE = TopAbs_ShapeEnum.TopAbs_EDGE
    WIRE = TopAbs_ShapeEnum.TopAbs_WIRE
    FACE = TopAbs_ShapeEnum.TopAbs_FACE
    SHELL = TopAbs_ShapeEnum.TopAbs_SHELL
    SOLID = TopAbs_ShapeEnum.TopAbs_SOLID
    COMPSOLID = TopAbs_ShapeEnum.TopAbs_COMPSOLID
    COMPOUND = TopAbs_ShapeEnum.TopAbs_COMPOUND

    def __init__(self, shape):
        if not isinstance(shape, TopoDS_Shape):
            raise TypeError('A TopoDS_Shape was not provided.')
        super(Shape, self).__init__()

        # The underlying OCCT shape
        self._shape = shape

    def __hash__(self):
        """
        Use the hash code of the shape.
        """
        return hash(self.hash_code)

    def __eq__(self, other):
        """
        Check equality using is_same.
        """
        if not isinstance(other, Shape):
            return False
        return self.is_same(other)

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Shape
        """
        return self.object

    @property
    def object(self):
        """
        :return: The underlying shape.
        :rtype: OCCT.TopoDS.TopoDS_Shape
        """
        return self._shape

    @property
    def hash_code(self):
        """
        :return: The hash code of the shape computed using the TShape and
            Location. Orientation is not used. The upper limit is 99,999.
        :rtype: int
        """
        return self.object.HashCode(99999)

    @property
    def is_null(self):
        """
        :return: *True* if shape is null, *False* if not.
        :rtype: bool
        """
        return self.object.IsNull()

    @property
    def shape_type(self):
        """
        :return: The shape type.
        :rtype: OCCT.TopAbs.TopAbs_ShapeEnum
        """
        return self.object.ShapeType()

    @property
    def is_vertex(self):
        """
        :return: *True* if a Vertex, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.VERTEX

    @property
    def is_edge(self):
        """
        :return: *True* if an Edge, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.EDGE

    @property
    def is_wire(self):
        """
        :return: *True* if a Wire, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.WIRE

    @property
    def is_face(self):
        """
        :return: *True* if a Face, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.FACE

    @property
    def is_shell(self):
        """
        :return: *True* if a Shell, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.SHELL

    @property
    def is_solid(self):
        """
        :return: *True* if a Solid, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.SOLID

    @property
    def is_compsolid(self):
        """
        :return: *True* if a CompSolid, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.COMPSOLID

    @property
    def is_compound(self):
        """
        :return: *True* if a Compound, *False* if not.
        :rtype: bool
        """
        return self.shape_type == Shape.COMPOUND

    @property
    def closed(self):
        """
        :return: The closed flag of the shape.
        :rtype: bool
        """
        return self.object.Closed()

    @property
    def infinite(self):
        """
        :return: The infinite flag of the shape.
        :rtype: bool
        """
        return self.object.Infinite()

    @property
    def vertices(self):
        """
        :return: The vertices of the shape.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self._get_shapes(self.VERTEX)

    @property
    def edges(self):
        """
        :return: The edges of the shape.
        :rtype: list(afem.topology.entities.Edge)
        """
        return self._get_shapes(self.EDGE)

    @property
    def wires(self):
        """
        :return: The wires of the shape.
        :rtype: list(afem.topology.entities.Wire)
        """
        return self._get_shapes(self.WIRE)

    @property
    def faces(self):
        """
        :return: The Face of the shape.
        :rtype: list(afem.topology.entities.Face)
        """
        return self._get_shapes(self.FACE)

    @property
    def shells(self):
        """
        :return: The shells of the shape.
        :rtype: list(afem.topology.entities.Shell)
        """
        return self._get_shapes(self.SHELL)

    @property
    def solids(self):
        """
        :return: The solids of the shape.
        :rtype: list(afem.topology.entities.Solid)
        """
        return self._get_shapes(self.SOLID)

    @property
    def compounds(self):
        """
        :return: The compounds of the shape.
        :rtype: list(afem.topology.entities.Compound)
        """
        return self._get_shapes(self.COMPOUND)

    @property
    def compsolids(self):
        """
        :return: The compsolids of the shape.
        :rtype: list(afem.topology.entities.CompSolid)
        """
        return self._get_shapes(self.COMPSOLID)

    @property
    def num_vertices(self):
        """
        :return: The number of vertices in the shape.
        :rtype: int
        """
        map_ = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.VERTEX, map_)
        return map_.Extent()

    @property
    def num_edges(self):
        """
        :return: The number of edges in the shape.
        :rtype: int
        """
        map_ = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.EDGE, map_)
        return map_.Extent()

    @property
    def num_faces(self):
        """
        :return: The number of faces in the shape.
        :rtype: int
        """
        map_ = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.FACE, map_)
        return map_.Extent()

    @property
    def tol_avg(self):
        """
        :return: The average global tolerance.
        :rtype: float
        """
        tol = ShapeAnalysis_ShapeTolerance()
        tol.AddTolerance(self.object)
        return tol.GlobalTolerance(0)

    @property
    def tol_min(self):
        """
        :return: The minimum global tolerance.
        :rtype: float
        """
        tol = ShapeAnalysis_ShapeTolerance()
        tol.AddTolerance(self.object)
        return tol.GlobalTolerance(-1)

    @property
    def tol_max(self):
        """
        :return: The minimum global tolerance.
        :rtype: float
        """
        tol = ShapeAnalysis_ShapeTolerance()
        tol.AddTolerance(self.object)
        return tol.GlobalTolerance(1)

    @property
    def shape_iter(self):
        """
        :return: Yield the underlying sub-shape(s).
        :rtype: collections.Iterable(afem.topology.entities.Shape)
        """
        it = TopoDS_Iterator(self.object)
        while it.More():
            yield Shape.wrap(it.Value())
            it.Next()

    @property
    def length(self):
        """
        :return: The length of all edges of the shape.
        :rtype: float
        """
        props = GProp_GProps()
        BRepGProp.LinearProperties_(self.object, props, True)
        return props.Mass()

    @property
    def area(self):
        """
        :return: The area of all faces of the shape.
        :rtype: float
        """
        props = GProp_GProps()
        BRepGProp.SurfaceProperties_(self.object, props, True)
        return props.Mass()

    @property
    def volume(self):
        """
        :return: The voume of all solids of the shape.
        :rtype: float
        """
        props = GProp_GProps()
        BRepGProp.VolumeProperties_(self.object, props, True)
        return props.Mass()

    @property
    def point(self):
        """
        :return: *None* unless overridden in derived class.
        :rtype: afem.geometry.entities.Point or None
        """
        return None

    @property
    def curve(self):
        """
        :return: *None* unless overridden in derived class.
        :rtype: afem.geometry.entities.Curve or None
        """
        return None

    @property
    def surface(self):
        """
        :return: *None* unless overridden in derived class.
        :rtype: afem.geometry.entities.Surface or None
        """
        return None

    def _get_shapes(self, type_):
        """
        Get sub-shapes of a specified type from the shape.
        """
        map_ = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, type_, map_)
        shapes = []
        for i in range(1, map_.Size() + 1):
            shapes.append(Shape.wrap(map_.FindKey(i)))
        return shapes

    def nullify(self):
        """
        Destroy reference to underlying shape and make it null.

        :return: None.
        """
        self.object.Nullify()

    def reverse(self):
        """
        Reverse the orientation of the shape.

        :return: None.
        """
        self.object.Reverse()

    def reversed(self):
        """
        Create a new shape with reversed orientation.

        :return: The reversed shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self.object.Reversed())

    def is_partner(self, other):
        """
        Check if this shape shares the same TShape with the other. Locations
        and Orientations may differ.

        :param afem.topology.entities.Shape other: Other shape.

        :return: *True* if partner, *False* if not.
        :rtype: bool
        """
        return self.object.IsPartner(other.object)

    def is_same(self, other):
        """
        Check if this shape shares the same TShape and Location with the other.
        Orientations may differ.

        :param afem.topology.entities.Shape other: Other shape.

        :return: *True* if same, *False* if not.
        :rtype: bool
        """
        return self.object.IsSame(other.object)

    def is_equal(self, other):
        """
        Check if this shape is equal to the other. That is, they share the same
        TShape, Location, and Orientation.

        :param afem.topology.entities.Shape other: Other shape.

        :return: *True* if equal, *False* if not.
        :rtype: bool
        """
        return self.object.IsEqual(other.object)

    def copy(self, geom=True):
        """
        Copy this shape.

        :param bool geom: Option to copy geometry.

        :return: The copied shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(BRepBuilderAPI_Copy(self.object, geom).Shape())

    def shared_vertices(self, other, as_compound=False):
        """
        Get shared vertices between this shape and the other.

        :param afem.topology.entities.Shape other: The other shape.
        :param bool as_compound: Option to return shared shapes as a single
            compound.

        :return: Shared vertices.
        :rtype: list(afem.topology.entities.Vertex) or
            afem.topology.entities.Compound
        """
        this_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.VERTEX, this_map)
        if this_map.Extent() == 0:
            return []

        other_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(other.object, Shape.VERTEX, other_map)
        if other_map.Extent() == 0:
            return []

        verts = []
        for i in range(1, this_map.Size() + 1):
            v1 = this_map.FindKey(i)
            if other_map.Contains(v1):
                verts.append(Shape.wrap(v1))

        if as_compound:
            return Compound.by_shapes(verts)
        return verts

    def shared_edges(self, other, as_compound=False):
        """
        Get shared edges between this shape and the other.

        :param afem.topology.entities.Shape other: The other shape.
        :param bool as_compound: Option to return shared shapes as a single
            compound.

        :return: Shared edges.
        :rtype: list(afem.topology.entities.Edge) or
            afem.topology.entities.Compound
        """
        this_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.EDGE, this_map)
        if this_map.Extent() == 0:
            return []

        other_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(other.object, Shape.EDGE, other_map)
        if other_map.Extent() == 0:
            return []

        edges = []
        for i in range(1, this_map.Size() + 1):
            e1 = this_map.FindKey(i)
            if other_map.Contains(e1):
                edges.append(Shape.wrap(e1))

        if as_compound:
            return Compound.by_shapes(edges)
        return edges

    def shared_faces(self, other, as_compound=False):
        """
        Get shared faces between this shape and the other.

        :param afem.topology.entities.Shape other: The other shape.
        :param bool as_compound: Option to return shared shapes as a single
            compound.

        :return: Shared faces.
        :rtype: list(afem.topology.entities.Face) or
            afem.topology.entities.Compound
        """
        this_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(self.object, Shape.FACE, this_map)
        if this_map.Extent() == 0:
            return []

        other_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_(other.object, Shape.FACE, other_map)
        if other_map.Extent() == 0:
            return []

        faces = []
        for i in range(1, this_map.Size() + 1):
            f1 = this_map.FindKey(i)
            if other_map.Contains(f1):
                faces.append(Shape.wrap(f1))

        if as_compound:
            return Compound.by_shapes(faces)
        return faces

    @staticmethod
    def wrap(shape):
        """
        Wrap the OpenCASCADE shape based on its type.

        :param OCCT.TopoDS.TopoDS_Shape shape: The shape.

        :return: The wrapped shape.
        :rtype: afem.topology.entities.Shape or afem.topology.entities.Vertex
            or afem.topology.entities.Edge or afem.topology.entities.Wire or
            afem.topology.entities.Face or afem.topology.entities.Shell or
            afem.topology.entities.Solid or afem.topology.entities.CompSolid or
            afem.topology.entities.Compound

        .. warning::

            If a null shape is provided then it is returned as a ``Shape``.
        """
        if shape.IsNull():
            return Shape(shape)

        if shape.ShapeType() == Shape.VERTEX:
            return Vertex(shape)
        if shape.ShapeType() == Shape.EDGE:
            return Edge(shape)
        if shape.ShapeType() == Shape.WIRE:
            return Wire(shape)
        if shape.ShapeType() == Shape.FACE:
            return Face(shape)
        if shape.ShapeType() == Shape.SHELL:
            return Shell(shape)
        if shape.ShapeType() == Shape.SOLID:
            return Solid(shape)
        if shape.ShapeType() == Shape.COMPSOLID:
            return CompSolid(shape)
        if shape.ShapeType() == Shape.COMPOUND:
            return Compound(shape)

        return Shape(shape)

    @staticmethod
    def to_shape(entity):
        """
        Convent an entity to a shape. If already a shape the entity is
        returned. If the entity is geometry it is converted to its
        corresponding shape.

        :param entity: The entity.
        :type entity: afem.topology.entities.Shape or
            afem.geometry.entities.Curve or afem.geometry.entities.Surface or
            point_like

        :return: The shape.
        :rtype: afem.topology.entities.Shape

        :raise TypeError: If entity cannot be converted to a shape.
        """
        if entity is None:
            return None

        if isinstance(entity, Shape):
            return entity

        if CheckGeom.is_point_like(entity):
            return Vertex.by_point(entity)
        elif CheckGeom.is_curve(entity):
            return Edge.by_curve(entity)
        elif CheckGeom.is_surface(entity):
            return Face.by_surface(entity)
        else:
            n = entity.__class__.__name__
            raise TypeError('Cannot convert a {} to a shape.'.format(n))

    @staticmethod
    def from_topods_list(topods_list):
        """
        Create a Python list of shapes from a TopoDS_ListOfShape.

        :param OCCT.TopoDS.TopoDS_ListOfShape topods_list: The list.

        :return: The list of shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return [Shape.wrap(s) for s in topods_list]


class Vertex(Shape):
    """
    Vertex.

    :param OCCT.TopoDS.TopoDS_Vertex shape: The vertex.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Vertex``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.VERTEX:
                raise TypeError('Shape is not a TopoDS_Vertex.')
            if not isinstance(shape, TopoDS_Vertex):
                shape = TopoDS.Vertex_(shape)
        super(Vertex, self).__init__(shape)

    @property
    def point(self):
        """
        :return: A point at vertex location.
        :rtype: afem.geometry.entities.Point
        """
        gp_pnt = BRep_Tool.Pnt_(self.object)
        return Point(gp_pnt.X(), gp_pnt.Y(), gp_pnt.Z())

    def parameter(self, edge, face=None):
        pass

    @staticmethod
    def by_point(pnt):
        """
        Create a vertex by a point.

        :param point_like pnt: The point.

        :return: The vertex.
        :rtype: afem.topology.entities.Vertex
        """
        pnt = CheckGeom.to_point(pnt)
        return Vertex(BRepBuilderAPI_MakeVertex(pnt).Vertex())


class Edge(Shape):
    """
    Edge.

    :param OCCT.TopoDS.TopoDS_Edge shape: The edge.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Edge``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.EDGE:
                raise TypeError('Shape is not a TopoDS_Edge.')
            if not isinstance(shape, TopoDS_Edge):
                shape = TopoDS.Edge_(shape)
        super(Edge, self).__init__(shape)

    @property
    def curve(self):
        """
        :return: The underlying curve of the edge.
        :rtype: afem.geometry.entities.Curve
        """
        geom_curve, _, _ = BRep_Tool.Curve_(self.object, 0., 0.)
        return Curve.wrap(geom_curve)

    @property
    def first_vertex(self):
        """
        :return: The first vertex of the edge.
        :rtype: afem.topology.entities.Vertex
        """
        return Vertex(ShapeAnalysis_Edge().FirstVertex(self.object))

    @property
    def last_vertex(self):
        """
        :return: The last vertex of the edge.
        :rtype: afem.topology.entities.Vertex
        """
        return Vertex(ShapeAnalysis_Edge().LastVertex(self.object))

    @property
    def same_parameter(self):
        """
        :return: The same parameter flag for the edge.
        :rtype: bool
        """
        return BRep_Tool.SameParameter_(self.object)

    @property
    def same_range(self):
        """
        :return: The same range flag for the edge.
        :rtype: bool
        """
        return BRep_Tool.SameRange_(self.object)

    @staticmethod
    def by_curve(curve):
        """
        Create an edge by a curve.

        :param afem.geometry.entities.Curve curve: The curve.

        :return: The edge.
        :rtype: afem.topology.entities.Edge
        """
        return Edge(BRepBuilderAPI_MakeEdge(curve.object).Edge())


class Wire(Shape):
    """
    Wire.

    :param OCCT.TopoDS.TopoDS_Wire shape: The wire.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Wire``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.WIRE:
                raise TypeError('Shape is not a TopoDS_Wire.')
            if not isinstance(shape, TopoDS_Wire):
                shape = TopoDS.Wire_(shape)
        super(Wire, self).__init__(shape)

    @property
    def curve(self):
        """
        :return: The curve formed by concatenating all the underlying curves
            of the edges.
        :rtype: afem.geometry.entities.NurbsCurve
        """
        geom_convert = GeomConvert_CompCurveToBSplineCurve()
        exp = BRepTools_WireExplorer(self.object)
        tol = self.tol_max
        while exp.More():
            e = TopoDS.Edge_(exp.Current())
            exp.Next()
            adp_crv = BRepAdaptor_Curve(e)
            geom_convert.Add(adp_crv.BSpline(), tol)
        geom_curve = geom_convert.BSplineCurve()
        return Curve.wrap(geom_curve)

    @staticmethod
    def by_curve(curve):
        """
        Create a wire from a curve.

        :param afem.geometry.entities.Curve curve: The curve.

        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        edge = Edge.by_curve(curve)
        return Wire.by_edge(edge)

    @staticmethod
    def by_edge(edge):
        """
        Create a wire from an edge.

        :param afem.topology.entities.Edge edge: The edge.

        :return: The wire.
        :rtype: afem.topology.entities.Wire
        """
        return Wire(BRepBuilderAPI_MakeWire(edge.object).Wire())

    @staticmethod
    def by_points(pnts, close=False):
        """
        Create polygonal wire by connecting points.

        :param collections.Sequence(point_like) pnts: The ordered points.
        :param bool close: Option to close the wire.

        :return: The new wire.
        :rtype: afem.topology.entities.Wire
        """
        builder = BRepBuilderAPI_MakePolygon()
        for p in pnts:
            p = CheckGeom.to_point(p)
            builder.Add(p)
        if close:
            builder.Close()
        return Wire(builder.Wire())


class Face(Shape):
    """
    Face.

    :param OCCT.TopoDS.TopoDS_Face shape: The face.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Face``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.FACE:
                raise TypeError('Shape is not a TopoDS_Face.')
            if not isinstance(shape, TopoDS_Face):
                shape = TopoDS.Face_(shape)
        super(Face, self).__init__(shape)

    @property
    def surface(self):
        """
        :return: The underlying surface of the face.
        :rtype: afem.geometry.entities.Surface
        """
        geom_surface = BRep_Tool.Surface_(self.object)
        return Surface.wrap(geom_surface)

    @property
    def outer_wire(self):
        """
        :return: The outer wire of the face.
        :rtype: afem.topology.entities.Wire
        """
        return Wire(BRepTools.OuterWire_(self.object))

    def to_shell(self):
        """
        Create a shell from the face.

        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        topods_shell = TopoDS_Shell()
        builder = BRep_Builder()
        builder.MakeShell(topods_shell)
        builder.Add(topods_shell, self.object)
        return Shell(topods_shell)

    @staticmethod
    def by_surface(surface):
        """
        Create a face by a surface.

        :param afem.geometry.entities.Surface surface: The surface.

        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return Face(BRepBuilderAPI_MakeFace(surface.object, 1.0e-7).Face())

    @staticmethod
    def by_wire(wire):
        """
        Create a face by a planar wire.

        :param afem.topology.entities.Wire  wire: The wire.

        :return: The new face.
        :rtype: afem.topology.entities.Face
        """
        return Face(BRepBuilderAPI_MakeFace(wire.object, True).Face())


class Shell(Shape):
    """
    Shell.

    :param OCCT.TopoDS.TopoDS_Shell shape: The shell.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Shell``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.SHELL:
                raise TypeError('Shape is not a TopoDS_Shell.')
            if not isinstance(shape, TopoDS_Shell):
                shape = TopoDS.Shell_(shape)
        super(Shell, self).__init__(shape)

    @property
    def surface(self):
        """
        :return: The underlying surface of the largest face.
        :rtype: afem.geometry.entities.Surface
        """
        areas = []
        faces = self.faces

        if len(faces) == 1:
            return faces[0].surface

        for f in faces:
            areas.append((f.area, f))
        areas.sort()
        fmax = areas[-1][1]

        return fmax.surface

    @staticmethod
    def by_surface(surface):
        """
        Create a shell from a surface.

        :param afem.geometry.entities.Surface surface: The surface.

        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        face = Face.by_surface(surface)
        return Shell.by_face(face)

    @staticmethod
    def by_face(face):
        """
        Create a shell from a face.

        :param afem.topology.entities.Face face: The face.

        :return: The shell.
        :rtype: afem.topology.entities.Shell
        """
        topods_shell = TopoDS_Shell()
        builder = BRep_Builder()
        builder.MakeShell(topods_shell)
        builder.Add(topods_shell, face.object)
        return Shell(topods_shell)


class Solid(Shape):
    """
    Solid.

    :param OCCT.TopoDS.TopoDS_Solid shape: The solid.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Solid``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.SOLID:
                raise TypeError('Shape is not a TopoDS_Solid.')
            if not isinstance(shape, TopoDS_Solid):
                shape = TopoDS.Solid_(shape)
        super(Solid, self).__init__(shape)

    @property
    def outer_shell(self):
        """
        :return: The outer shell of the face.
        :rtype: afem.topology.entities.Shell
        """
        return Shell(BRepClass3d.OuterShell_(self.object))

    @staticmethod
    def by_shell(shell):
        """
        Create a solid from the shell.

        :param afem.topology.entities.Shell shell: The shell.

        :return: The new solid.
        :rtype: afem.topology.entities.Solid
        """
        return Solid(ShapeFix_Solid().SolidFromShell(shell.object))


class CompSolid(Shape):
    """
    CompSolid.

    :param OCCT.TopoDS.TopoDS_CompSolid shape: The compsolid.

    :raise TypeError: If ``shape`` is not a ``TopoDS_CompSolid``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.COMPSOLID:
                raise TypeError('Shape is not a TopoDS_CompSolid.')
            if not isinstance(shape, TopoDS_CompSolid):
                shape = TopoDS.CompSolid_(shape)
        super(CompSolid, self).__init__(shape)


class Compound(Shape):
    """
    Compound.

    :param OCCT.TopoDS.TopoDS_Compound shape: The compound.

    :raise TypeError: If ``shape`` is not a ``TopoDS_Compound``.
    """

    def __init__(self, shape):
        if not shape.IsNull():
            if not shape.ShapeType() == Shape.COMPOUND:
                raise TypeError('Shape is not a TopoDS_Compound.')
            if not isinstance(shape, TopoDS_Compound):
                shape = TopoDS.Compound_(shape)
        super(Compound, self).__init__(shape)

    @property
    def surface(self):
        """
        :return: The underlying surface of the largest face, or *None* if
            compound has no faces.
        :rtype: afem.geometry.entities.Surface
        """
        areas = []
        faces = self.faces
        nfaces = len(faces)

        if nfaces < 0:
            return None
        if nfaces == 1:
            return faces[0].surface

        for f in faces:
            areas.append((f.area, f))
        areas.sort()
        fmax = areas[-1][1]

        return fmax.surface

    @staticmethod
    def by_shapes(shapes):
        """
        Create a new compound from a collection of shapes.

        :param collections.Sequence(afem.topology.entities.Shape) shapes: The
            shapes.

        :return: The new compound.
        :rtype: afem.topology.entities.Compound
        """
        topods_compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(topods_compound)
        for shape in shapes:
            if isinstance(shape, Shape):
                builder.Add(topods_compound, shape.object)
        return Compound(topods_compound)


class BBox(Bnd_Box):
    """
    Bounding box in 3-D space.
    """

    def __init__(self):
        super(BBox, self).__init__()

    @property
    def is_void(self):
        """
        :return: *True* if bounding box is empty, *False* if not.
        :rtype: bool
        """
        return self.IsVoid()

    @property
    def pmin(self):
        """
        :return: Lower corner of bounding box. *None* if empty.
        :rtype: afem.geometry.entities.Point
        """
        if self.is_void:
            return None
        return Point(self.CornerMin().XYZ())

    @property
    def pmax(self):
        """
        :return: Upper corner of bounding box. *None* if empty.
        :rtype: afem.geometry.entities.Point
        """
        if self.is_void:
            return None
        return Point(self.CornerMax().XYZ())

    @property
    def xmin(self):
        """
        :return: Minimum x-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMin().X()

    @property
    def xmax(self):
        """
        :return: Maximum x-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMax().X()

    @property
    def ymin(self):
        """
        :return: Minimum y-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMin().Y()

    @property
    def ymax(self):
        """
        :return: Maximum y-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMax().Y()

    @property
    def zmin(self):
        """
        :return: Minimum z-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMin().Z()

    @property
    def zmax(self):
        """
        :return: Maximum z-component.
        :rtype: float
        """
        if self.is_void:
            return None
        return self.CornerMax().Z()

    @property
    def gap(self):
        """
        :return: The gap of the bounding box.
        :rtype: float
        """
        return self.GetGap()

    @property
    def diagonal(self):
        """
        :return: The diagonal length of the box.
        :rtype: float
        """
        return sqrt(self.SquareExtent())

    def set_gap(self, gap):
        """
        Set the gap of the bounding box.

        :param float gap: The gap.

        :return: None.
        """
        self.SetGap(abs(gap))

    def enlarge(self, tol):
        """
        Enlarge the box with a tolerance value.

        :param float tol: The tolerance.

        :return: None.
        """
        self.Enlarge(tol)

    def add_box(self, bbox):
        """
        Add the other box to this one.

        :param afem.topology.entities.BBox bbox: The other box.

        :return: None.

        :raise TypeError: If *bbox* cannot be converted to a bounding box.
        """
        if not isinstance(bbox, Bnd_Box):
            msg = 'Methods requires a BBox instance.'
            raise TypeError(msg)

        self.Add(bbox)

    def add_pnt(self, pnt):
        """
        Add a point to the bounding box.

        :param point_like pnt: The point.

        :return: None.

        :raise TypeError: If *pnt* cannot be converted to a point.
        """
        pnt = CheckGeom.to_point(pnt)
        if not pnt:
            msg = 'Invalid point type provided.'
            raise TypeError(msg)

        self.Add(pnt)

    def add_shape(self, shape):
        """
        Add shape to the bounding box.

        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        BRepBndLib.Add_(shape.object, self, True)

    def is_pnt_out(self, pnt):
        """
        Check to see if the point is outside the bounding box.

        :param point_like pnt: The point.

        :return: *True* if outside, *False* if not.
        :rtype: bool

        :raise TypeError: If *pnt* cannot be converted to a point.
        """
        pnt = CheckGeom.to_point(pnt)
        if not pnt:
            msg = 'Invalid point type provided.'
            raise TypeError(msg)

        return self.IsOut(pnt)

    def is_line_out(self, line):
        """
        Check to see if the line intersects the box.

        :param afem.geometry.entities.Line line: The line.

        :return: *True* if outside, *False* if it intersects.
        :rtype: bool

        :raise TypeError: If *line* is not a line.
        """

        if not CheckGeom.is_line(line):
            msg = 'Methods requires a Line instance.'
            raise TypeError(msg)

        return self.IsOut(line.object)

    def is_pln_out(self, pln):
        """
        Check to see if the plane intersects the box.

        :param afem.geometry.entities.Plane pln: The plane.

        :return: *True* if outside, *False* if it intersects.
        :rtype: bool

        :raise TypeError: If *pln* is not a plane.
        """

        if not CheckGeom.is_plane(pln):
            msg = 'Methods requires a Plane instance.'
            raise TypeError(msg)

        return self.IsOut(pln.object)

    def is_box_out(self, bbox):
        """
        Check to see if the bounding box intersects this one.

        :param afem.topology.entities.BBox bbox: The other box.

        :return: *True* if outside, *False* if it intersects or is inside.
        :rtype: bool

        :raise TypeError: If *bbox* cannot be converted to a bounding box.
        """
        if not isinstance(bbox, Bnd_Box):
            msg = 'Methods requires a BBox instance.'
            raise TypeError(msg)

        return self.IsOut(bbox)

    def distance(self, bbox):
        """
        Calculate distance to other box.

        :param afem.geometry.entities.BBox bbox: The other box.

        :return: Distance to other box.
        :rtype: float

        :raise TypeError: If *bbox* cannot be converted to a bounding box.
        """
        if not isinstance(bbox, Bnd_Box):
            msg = 'Methods requires a BBox instance.'
            raise TypeError(msg)

        return self.Distance(bbox)
