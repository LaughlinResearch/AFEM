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
from datetime import datetime

from OCCT.BOPAlgo import BOPAlgo_MakerVolume, BOPAlgo_Options
from OCCT.BRepAlgoAPI import (BRepAlgoAPI_Common, BRepAlgoAPI_Cut,
                              BRepAlgoAPI_Fuse, BRepAlgoAPI_Section,
                              BRepAlgoAPI_Splitter)
from OCCT.BRepFeat import BRepFeat_MakeCylindricalHole, BRepFeat_SplitShape
from OCCT.Message import Message_Gravity
from OCCT.TopTools import TopTools_SequenceOfShape
from OCCT.TopoDS import TopoDS_Face

from afem.config import logger
from afem.geometry.entities import Surface
from afem.occ.utils import to_topods_list
from afem.topology.entities import Shape, Face, Solid, Compound
from afem.topology.explore import ExploreWire
from afem.topology.modify import RebuildShapeByTool

__all__ = ["BopCore", "BopAlgo", "FuseShapes", "CutShapes", "CommonShapes",
           "IntersectShapes", "SplitShapes", "VolumesFromShapes",
           "CutCylindricalHole", "LocalSplit", "SplitShapeByEdges",
           "SplitWire", "TrimOpenWire"]

# Turn on parallel Boolean execution by default
BOPAlgo_Options.SetParallelMode_(True)

# Message gravities
_gravities = [Message_Gravity.Message_Trace, Message_Gravity.Message_Info,
              Message_Gravity.Message_Warning, Message_Gravity.Message_Alarm,
              Message_Gravity.Message_Fail]


class BopCore(object):
    """
    Core class for Boolean operations and enabling attributes and methods for
    rebuilding shapes.
    """

    def __init__(self):
        self._bop = None

    def build(self):
        """
        Build the results.

        :return: None.
        """
        if isinstance(self._bop, BOPAlgo_MakerVolume):
            self._bop.Perform()
        else:
            self._bop.Build()

    @property
    def is_done(self):
        """
        :return: *True* if operation is done, *False* if not.
        :rtype: bool
        """
        if isinstance(self._bop, (BOPAlgo_MakerVolume,
                                  BRepFeat_MakeCylindricalHole)):
            return not self._bop.HasErrors()
        return self._bop.IsDone()

    @property
    def shape(self):
        """
        :return: The resulting shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._bop.Shape())

    def modified(self, shape):
        """
        Return a list of shapes modified from the given shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: List of modified shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._bop.Modified(shape.object))

    def generated(self, shape):
        """
        Return a list of shapes generated from the given shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: List of generated shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._bop.Generated(shape.object))

    def is_deleted(self, shape):
        """
        Check to see if shape is deleted.

        :param afem.topology.entities.Shape shape: The shape.

        :return: *True* if deleted, *False* if not.
        :rtype: bool
        """
        return self._bop.IsDeleted(shape.object)


class BopAlgo(BopCore):
    """
    Base class for Boolean operations.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or None
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or None
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.
    :param bop: The OpenCASCADE class for the Boolean operation.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1, shape2, fuzzy_val, nondestructive, bop):
        super(BopAlgo, self).__init__()

        self._bop = bop()

        if fuzzy_val is not None:
            self._bop.SetFuzzyValue(fuzzy_val)

        if nondestructive:
            self._bop.SetNonDestructive(True)
        else:
            self._bop.SetNonDestructive(False)

        if isinstance(shape1, Shape) and isinstance(shape2, Shape):
            self.set_args([shape1])
            self.set_tools([shape2])
            self.build()

    @staticmethod
    def set_parallel_mode(flag):
        """
        Global option to set the Boolean operations for parallel execution.

        :param bool flag: Option for parallel execution. *True* turns parallel
            execution on, *False* turns it off.

        :return: None.
        """
        BOPAlgo_Options.SetParallelMode_(flag)

    def debug(self, path='.'):
        """
        Export files for debugging Boolean operations.

        :param path:

        :return:
        """
        # Generate a suffix using timestamp
        now = datetime.now()
        timestamp = str(now.timestamp())

        # Operation name for prefix
        op = str(self.__class__.__name__)

        # Info file
        fn = ''.join([path, '/', op, '.info.', timestamp, '.txt'])
        info = open(fn, 'w')
        info.write('Date (M-D-Y): {}-{}-{}\n'.format(now.month, now.day,
                                                     now.year))
        info.write('Operation: {}\n'.format(op))
        info.write('Parallel: {}\n'.format(self._bop.RunParallel()))
        info.write('Fuzzy value: {}\n'.format(self._bop.FuzzyValue()))
        info.write('Nondestructive: {}\n'.format(self._bop.NonDestructive()))

        # Errors and warnings report
        msg_report = self._bop.GetReport()
        for gravity in _gravities:
            msg_list = msg_report.GetAlerts(gravity)
            if msg_list.Size() == 0:
                continue
            info.write('Messages:\n')
            for msg in msg_list:
                info.write('\t{}\n'.format(msg.GetMessageKey()))
        info.close()

        # Avoid circular imports
        from afem.exchange.brep import write_brep
        from afem.topology.create import CompoundByShapes

        # Arguments
        args = self.arguments
        if args:
            shape1 = CompoundByShapes(args).compound
            fn = ''.join([path, '/', op, '.shape1.', timestamp, '.brep'])
            write_brep(shape1, fn)

        # Tools
        tools = self.tools
        if tools:
            shape2 = CompoundByShapes(tools).compound
            fn = ''.join([path, '/', op, '.shape2.', timestamp, '.brep'])
            write_brep(shape2, fn)

    @property
    def arguments(self):
        """
        :return: The arguments.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._bop.Arguments())

    @property
    def tools(self):
        """
        :return: The tools.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._bop.Tools())

    def set_args(self, shapes):
        """
        Set the arguments.

        :param list(afem.topology.entities.Shape) shapes: The arguments.

        :return: None.
        """
        if isinstance(self._bop, BOPAlgo_MakerVolume):
            for shape in shapes:
                self._bop.AddArgument(shape.object)
            return None
        args = to_topods_list(shapes)
        self._bop.SetArguments(args)

    def set_tools(self, shapes):
        """
        Set the tools.

        :param list(afem.topology.entities.Shape) shapes: The tools.

        :return: None.
        """
        if isinstance(self._bop, BOPAlgo_MakerVolume):
            n = self._bop.__class__.__name__
            msg = ('Setting tools not available for {}. '
                   'Doing nothing.'.format(n))
            logger.warning(msg)
            return None

        tools = to_topods_list(shapes)
        self._bop.SetTools(tools)

    @property
    def vertices(self):
        """
        :return: The vertices of the resulting shape.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self.shape.vertices

    @property
    def edges(self):
        """
        :return: The edges of the resulting shape.
        :rtype: list(afem.topology.entities.Edge)
        """
        return self.shape.edges

    def refine_edges(self):
        """
        Fuse C1 edges.

        :return: None.
        """
        if isinstance(self._bop, (BRepAlgoAPI_Splitter, BOPAlgo_MakerVolume,
                                  BRepFeat_MakeCylindricalHole)):
            n = self._bop.__class__.__name__
            msg = ('Refining edges not available for {}. '
                   'Doing nothing.'.format(n))
            logger.warning(msg)
        else:
            self._bop.RefineEdges()

    @property
    def fuse_edges(self):
        """
        :return: The result flag of edge refining.
        :rtype: bool
        """
        if isinstance(self._bop, (BRepAlgoAPI_Splitter, BOPAlgo_MakerVolume,
                                  BRepFeat_MakeCylindricalHole)):
            return False
        else:
            return self._bop.FuseEdges()

    @property
    def section_edges(self):
        """
        :return: A list of section edges as a result of intersection between
            the shapes.
        :rtype: list(afem.topology.entities.Edge)
        """
        if isinstance(self._bop, (BRepAlgoAPI_Splitter, BOPAlgo_MakerVolume,
                                  BRepFeat_MakeCylindricalHole)):
            n = self._bop.__class__.__name__
            msg = ('Getting section edges not available for {}. '
                   'Returning an empty list.'.format(n))
            logger.warn(msg)
            return []
        else:
            return Shape.from_topods_list(self._bop.SectionEdges())

    @property
    def has_modified(self):
        """
        :return: *True* if there is at least one modified shape.
        :rtype: bool
        """
        return self._bop.HasModified()

    @property
    def has_generated(self):
        """
        :return: *True* if there is at least one generated shape.
        :rtype: bool
        """
        return self._bop.HasGenerated()

    @property
    def has_deleted(self):
        """
        :return: *True* if there is at least one deleted shape.
        :rtype: bool
        """
        return self._bop.HasDeleted()


class FuseShapes(BopAlgo):
    """
    Boolean fuse operation.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or None
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or None
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1=None, shape2=None, fuzzy_val=None,
                 nondestructive=False):
        super(FuseShapes, self).__init__(shape1, shape2, fuzzy_val,
                                         nondestructive, BRepAlgoAPI_Fuse)


class CutShapes(BopAlgo):
    """
    Boolean cut operation.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or None
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or None
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1=None, shape2=None, fuzzy_val=None,
                 nondestructive=False):
        super(CutShapes, self).__init__(shape1, shape2, fuzzy_val,
                                        nondestructive, BRepAlgoAPI_Cut)


class CommonShapes(BopAlgo):
    """
    Boolean common operation.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or None
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or None
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1=None, shape2=None, fuzzy_val=None,
                 nondestructive=False):
        super(CommonShapes, self).__init__(shape1, shape2, fuzzy_val,
                                           nondestructive, BRepAlgoAPI_Common)


class IntersectShapes(BopAlgo):
    """
    Boolean intersect operation.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or
        afem.geometry.entities.Surface
    :param bool compute_pcurve1: Option to compute p-curves on shape 1.
    :param bool compute_pcurve2: Option to compute p-curves on shape 2.
    :param bool approximate: Option to approximate intersection curves.
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1=None, shape2=None, compute_pcurve1=False,
                 compute_pcurve2=False, approximate=False, fuzzy_val=None,
                 nondestructive=False):
        super(IntersectShapes, self).__init__(None, None, fuzzy_val,
                                              nondestructive,
                                              BRepAlgoAPI_Section)

        self._bop.ComputePCurveOn1(compute_pcurve1)
        self._bop.ComputePCurveOn2(compute_pcurve2)
        self._bop.Approximation(approximate)

        build1, build2 = False, False
        if isinstance(shape1, (Shape, Surface)):
            self._bop.Init1(shape1.object)
            build1 = True

        if isinstance(shape2, (Shape, Surface)):
            self._bop.Init2(shape2.object)
            build2 = True

        if build1 and build2:
            self._bop.Build()

    def has_ancestor_face1(self, edge):
        """
        Get the ancestor face on the intersection edge on the first shape
        if available.

        :param afem.topology.entities.Edge edge: The edge.

        :return: *True* and the face if available, *False* and *None* if not.
        :rtype: tuple(bool, afem.topology.entities.Face or None)
        """
        f = TopoDS_Face()
        if self._bop.HasAncestorFaceOn1(edge.object, f):
            return True, Face(f)
        return False, None

    def has_ancestor_face2(self, edge):
        """
        Get the ancestor face on the intersection edge on the second shape
        if available.

        :param afem.topology.entities.Edge edge: The edge.

        :return: *True* and the face if available, *False* and *None* if not.
        :rtype: tuple(bool, afem.topology.entities.Face or None)
        """
        f = TopoDS_Face()
        if self._bop.HasAncestorFaceOn2(edge.object, f):
            return True, Face(f)
        return False, None


class SplitShapes(BopAlgo):
    """
    Split arbitrary shapes. This is a wrapper for the SALOME
    GEOMAlgo_Splitter tool.

    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or None
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or None
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.

    .. note::

        If *shape1* or *shape2* is *None* then the user is expected to manually
        set the arguments and tools and build the result.
    """

    def __init__(self, shape1=None, shape2=None, fuzzy_val=None,
                 nondestructive=False):
        super(SplitShapes, self).__init__(shape1, shape2, fuzzy_val,
                                          nondestructive, BRepAlgoAPI_Splitter)


class VolumesFromShapes(BopAlgo):
    """
    Build solids from a list of shapes.

    :param list(afem.topology.entities.Shape) shapes: The shapes.
    :param bool intersect: Option to intersect the shapes before building
        solids.
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.
    """

    def __init__(self, shapes, intersect=False, fuzzy_val=None,
                 nondestructive=False):
        super(VolumesFromShapes, self).__init__(None, None, fuzzy_val,
                                                nondestructive,
                                                BOPAlgo_MakerVolume)

        self.set_args(shapes)

        if intersect:
            self._bop.SetIntersect(True)
        else:
            self._bop.SetIntersect(False)

        self.build()
        self._solids = self.shape.solids

    @property
    def box(self):
        """
        :return: The bounding box of all provided shapes.
        :rtype: afem.topology.entities.Solid
        """
        return Solid(self._bop.Box())

    @property
    def nsolids(self):
        """
        :return: The number of solids in the shape.
        :rtype: int
        """
        return len(self.solids)

    @property
    def solids(self):
        """
        :return: The list of solids.
        :rtype: list(afem.topology.entities.Solid)
        """
        return self._solids


class CutCylindricalHole(BopAlgo):
    """
    Cut a cylindrical hole on a shape.

    :param afem.topology.entities.Shape shape: The shape.
    :param float radius: The radius of the hole.
    :param afem.geometry.entities.Axis1: The axis for the hole.
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.
    """

    def __init__(self, shape, radius, ax1, fuzzy_val=None,
                 nondestructive=False):
        super(CutCylindricalHole, self).__init__(None, None, fuzzy_val,
                                                 nondestructive,
                                                 BRepFeat_MakeCylindricalHole)

        self._bop.Init(shape.object, ax1)
        self._bop.Perform(radius)


class LocalSplit(BopCore):
    """
    Perform a local split of a shape in the context of a basis shape. This tool
    only splits faces.

    :param afem.topology.entities.Shape shape: The local shape.
    :param tool: The tool to split with.
    :type tool: afem.topology.entities.Shape or afem.geometry.entities.Surface
    :param afem.topology.entities.Shape basis_shape: The basis shape that the
        local shape is part of.
    :param bool approximate: Option to approximate intersection curves.
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool nondestructive: Option to not modify the input shapes.
    """

    def __init__(self, shape, tool, basis_shape, approximate=False,
                 fuzzy_val=None, nondestructive=False):
        super(LocalSplit, self).__init__()

        # Intersect
        section = IntersectShapes(shape, tool, True, False, approximate,
                                  fuzzy_val, nondestructive)
        sec_edges = section.edges

        # Split
        self._bop = BRepFeat_SplitShape(basis_shape.object)
        for e in sec_edges:
            status, f = section.has_ancestor_face1(e)
            if status:
                self._bop.Add(e.object, f.object)
        self.build()


class SplitShapeByEdges(BopCore):
    """
    Split a shape using edges.

    :param afem.topology.entities.Shape shape: The basis shape.
    :param edges: The edges to split the shape with. If provided, then the
        results will be built during initialization. If none are provided then
        the user is expected to add edges and build manually.
    :type edges: collections.Sequence(afem.topology.entities.Edge) or None
    :param bool check_interior: Option to check internal intersections.
    """

    def __init__(self, shape, edges=None, check_interior=True):
        super(SplitShapeByEdges, self).__init__()

        self._bop = BRepFeat_SplitShape(shape.object)

        if not check_interior:
            self._bop.SetCheckInterior(False)

        if edges is not None:
            edge_seq = TopTools_SequenceOfShape()
            for e in edges:
                edge_seq.Append(e.object)
            self._bop.Add(edge_seq)
            self.build()

    def add_edges(self, shapes):
        """
        Add splittings edges or wires for the initial shape.

        :param collections.Sequence(afem.topology.entities.Shape) shapes: The
            splitting edges or wires.

        :return: *True* if added, *False* if not.
        :rtype: bool
        """
        seq = TopTools_SequenceOfShape()
        for s in shapes:
            seq.Append(s.object)
        return self._bop.Add(seq)

    def add_wire_on_face(self, w, f):
        """
        Add the wire on the face.

        :param afem.topology.entities.Wire w: The wire.
        :param afem.topology.entities.Face f: The face.

        :return: None.
        """
        self._bop.Add(w.object, f.object)

    def add_edge_on_face(self, e, f):
        """
        Add the edge on the face.

        :param afem.topology.entities.Edge e: The edge.
        :param afem.topology.entities.Face f: The face.

        :return: None
        """
        self._bop.Add(e.object, f.object)

    def add_edges_on_face(self, e, f):
        """

        :param collections.Sequence(afem.topology.entities.Edge) e: The edges.
        :param afem.topology.entities.Face f: The face.

        :return: None
        """
        # Avoid circular imports
        from afem.topology.create import CompoundByShapes

        cmp = CompoundByShapes(e).compound
        self._bop.Add(cmp.object, f.object)

    def add_edge_on_edge(self, e1, e2):
        """
        Add the edge on an existing edge.

        :param afem.topology.entities.Edge e1: The edge.
        :param afem.topology.entities.Edge e2: The existing edge.

        :return: None.
        """
        self._bop.Add(e1.object, e2.object)


class SplitWire(object):
    """
    Split a wire with a shape.

    :param afem.topology.entities.Wire wire: The wire.
    :param afem.topology.entities.Shape splitter: The splitter shape.

    :raise RuntimeError: If the splitting algorithm fails.
    """

    def __init__(self, wire, splitter):
        bop = SplitShapes(wire, splitter)
        if not bop.is_done:
            raise RuntimeError('Failed to split wire.')

        # Replace edges in wire
        rebuild = RebuildShapeByTool(wire, bop)
        self._wire = rebuild.new_shape

    @property
    def wire(self):
        """
        :return: The split wire.
        :rtype: afem.topology.entities.Wire
        """
        return self._wire


class TrimOpenWire(object):
    """
    Trim an open wire between one or two shapes.

    :param afem.topology.entities.Wire wire: The wire.
    :param shape1: The first shape.
    :type shape1: afem.topology.entities.Shape or
        afem.geometry.entities.Geometry
    :param shape2: The second shape.
    :type shape2: afem.topology.entities.Shape or
        afem.geometry.entities.Geometry

    :raise TypeError: If a wire is not provided or it is closed.
    :raise RuntimeError: If zero or more than two split locations are found.
        The split shapes must result in one or two split locations. That is,
        they should intersect the wire at only one location.
    """

    def __init__(self, wire, shape1, shape2=None):
        if wire.closed:
            raise TypeError('Closed wires are not supported.')

        shape1 = Shape.to_shape(shape1)
        shape2 = Shape.to_shape(shape2)

        # Split wire with shapes
        other_shape = Compound.by_shapes([shape1, shape2])
        split = SplitShapes(wire, other_shape)
        split_wire = split.shape.wires[0]

        # Get new vertices
        old_verts = ExploreWire(wire).ordered_vertices
        wire_exp = ExploreWire(split_wire)
        all_verts = wire_exp.ordered_vertices
        new_verts = [v for v in all_verts if v not in old_verts]

        # Find index of new vertices and use that to extract edges
        n = len(new_verts)
        if n == 2:
            i1 = all_verts.index(new_verts[0])
            i2 = all_verts.index(new_verts[1])
            ordered_edges = wire_exp.edges
            first_edges = ordered_edges[:i1]
            trimmed_edges = ordered_edges[i1:i2]
            last_edges = ordered_edges[i2:]
        elif n == 1:
            i1 = all_verts.index(new_verts[0])
            ordered_edges = wire_exp.edges
            first_edges = ordered_edges[:i1]
            trimmed_edges = []
            last_edges = ordered_edges[i1:]
        else:
            msg = 'Only one or two split locations are supported.'
            raise RuntimeError(msg)

        # Avoid circular imports
        from afem.topology.create import WireByEdges

        # Collect data and build trimmed wires
        self._first_wire = None
        self._trimmed_wire = None
        self._last_wire = None

        self._split_wire = split_wire
        if len(first_edges) > 0:
            self._first_wire = WireByEdges(*first_edges).wire
        if len(trimmed_edges) > 0:
            self._trimmed_wire = WireByEdges(*trimmed_edges).wire
        if len(last_edges) > 0:
            self._last_wire = WireByEdges(*last_edges).wire
        self._new_verts = new_verts
        self._verts = all_verts

    @property
    def split_wire(self):
        """
        :return: The wire after splitting.
        :rtype: afem.topology.entities.Wire
        """
        return self._split_wire

    @property
    def first_wire(self):
        """
        :return: The first trimmed segment.
        :rtype: afem.topology.entities.Wire
        """
        return self._first_wire

    @property
    def last_wire(self):
        """
        :return: The last trimmed segment.
        :rtype: afem.topology.entities.Wire
        """
        return self._last_wire

    @property
    def trimmed_wire(self):
        """
        :return: The interior trimmed segment.
        :rtype: afem.topology.entities.Wire
        """
        return self._trimmed_wire

    @property
    def new_vertices(self):
        """
        :return: New vertices after splitting.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self._new_verts

    @property
    def all_vertices(self):
        """
        :return: All ordered vertices after splitting the original wire but
            before trimming.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self._verts
