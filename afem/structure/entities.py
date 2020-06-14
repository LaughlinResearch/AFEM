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
from numpy import mean

from afem.config import logger
from afem.core.entities import ShapeHolder
from afem.geometry.check import CheckGeom
from afem.geometry.create import (PlaneByNormal, PointsAlongCurveByNumber)
from afem.geometry.entities import Axis1
from afem.structure.group import GroupAPI
from afem.structure.utils import shape_of_entity
from afem.topology.bop import (CutCylindricalHole, CutShapes, FuseShapes,
                               IntersectShapes, LocalSplit, SplitShapes)
from afem.topology.check import CheckShape, ClassifyPointInSolid
from afem.topology.create import (CompoundByShapes, HalfspaceBySurface,
                                  PointAlongShape, WiresByShape, FaceByPlane,
                                  SolidByDrag)
from afem.topology.distance import DistanceShapeToShape
from afem.topology.entities import Shape, Edge, Wire, Face, Shell, Compound
from afem.topology.fix import FixShape
from afem.topology.modify import (RebuildShapeByTool,
                                  RebuildShapeWithShapes, RebuildShapesByTool,
                                  SewShape, UnifyShape)
from afem.topology.props import LengthOfShapes, LinearProps, SurfaceProps

__all__ = ["Part", "CurvePart", "Beam1D", "SurfacePart", "WingPart", "Spar",
           "Rib", "FuselagePart", "Bulkhead", "Floor", "Frame", "Skin",
           "Stiffener1D", "Stiffener2D", "Stringer", "Beam2D"]


class Part(ShapeHolder):
    """
    Base class for all parts.

    :param str name: The name.
    :param afem.topology.entities.Shape shape: The shape.
    :param cref: The reference curve. If it is not a :class:`.TrimmedCurve`,
        then it will be converted to one.
    :type cref: afem.geometry.entities.Curve or None
    :param sref: The reference surface.
    :type sref: afem.geometry.entities.Surface or None
    :param group: The group to add the part to. If not provided the part will
        be added to the active group.
    :type group: str or afem.structure.group.Group or None
    """
    _indx = 1

    def __init__(self, name, shape, cref=None, sref=None, group=None):
        types = (Shape,)
        if isinstance(self, CurvePart):
            types = (Edge, Wire, Compound)
        elif isinstance(self, SurfacePart):
            types = (Face, Shell, Compound)
        super(Part, self).__init__(name, shape, cref, sref, types)

        # Unique ID
        self._id = Part._indx
        Part._indx += 1

        # Add to group
        GroupAPI.add_parts(group, self)

        # Log
        msg = ' '.join(['Creating part:', name])
        logger.info(msg)

        # Groups for meshing
        self._node_group = None
        self._edge_group = None
        self._face_group = None

    @property
    def id(self):
        """
        :return: The unique part ID.
        :rtype: int
        """
        return self._id

    @property
    def node_group(self):
        """
        :return: The mesh node group.
        :rtype: afem.smesh.entities.MeshGroup

        :raise AttributeError: If group doesn't exist.
        """
        if self._node_group is None:
            raise AttributeError('Node group does not exist.')
        return self._node_group

    @property
    def edge_group(self):
        """
        :return: The mesh edge group.
        :rtype: afem.smesh.entities.MeshGroup

        :raise AttributeError: If group doesn't exist.
        """
        if self._edge_group is None:
            raise AttributeError('Edge group does not exist.')
        return self._edge_group

    @property
    def face_group(self):
        """
        :return: The mesh face group.
        :rtype: afem.smesh.entities.MeshGroup

        :raise AttributeError: If group doesn't exist.
        """
        if self._face_group is None:
            raise AttributeError('Face group does not exist.')
        return self._face_group

    def distance(self, other):
        """
        Find the minimum distance between the part and other shape.

        :param other: Other part or shape.
        :type other: afem.topology.entities.Shape or
            afem.structure.entities.Part

        :return: The minimum distance.
        :rtype: float
        """
        other = shape_of_entity(other)
        return DistanceShapeToShape(self._shape, other).dmin

    def check(self, raise_error=True):
        """
        Check the shape of the part.

        :param bool raise_error: Option to raise an error if the shape is
            not valid.

        :return: *True* if shape is valid, *False* if not.
        :rtype: bool

        :raise RuntimeError: If the check fails and *raise_error* is ``True``.
        """
        check = CheckShape(self._shape).is_valid

        if not raise_error:
            return check

        if check:
            return True
        msg = ' '.join(['The shape of the part is not valid. Name:',
                        self.name])
        raise RuntimeError(msg)

    def fix(self, precision=None, min_tol=None, max_tol=None, context=None,
            include_subgroup=True):
        """
        Attempt to fix the shape of the part using :class:`.FixShape`.

        :param float precision: Basic precision value.
        :param float min_tol: Minimum tolerance.
        :param float max_tol: Maximum tolerance.
        :param context: The context shape or group.
        :type context: afem.topology.entities.Shape or
            afem.structure.entities.Group or str
        :param bool include_subgroup: Option to recursively include parts
            from any subgroups.

        :return: None.
        """
        if context is not None:
            if not isinstance(context, Shape):
                context = GroupAPI.get_shape(context, include_subgroup)

        new_shape = FixShape(self._shape, precision, min_tol, max_tol,
                             context).shape
        self.set_shape(new_shape)

    def cut(self, cutter):
        """
        Cut the part shape and rebuild this part.

        :param cutter: The cutter. If geometry is provided it will be
            converted to a shape before the Boolean operation.
        :type cutter: afem.topology.entities.Shape or
            afem.structure.entities.Part or afem.geometry.entities.Geometry

        :return: *True* if shape was cut, *False* if not.
        :rtype: bool
        """
        cutter = shape_of_entity(cutter)
        cut = CutShapes(self._shape, cutter)
        if not cut.is_done:
            return False

        self.rebuild(cut)

        return True

    def split(self, splitter, rebuild_both=True):
        """
        Split the part shape and rebuild this part. Optionally rebuild the
        splitter if it is a part. This method should handle splitting with
        parts and shapes or different types (i.e., splitting a face with an
        edge).

        :param splitter: The splitter.
        :type splitter: afem.topology.entities.Shape or
            afem.structure.entities.Part
        :param bool rebuild_both: Option to rebuild both if *splitter* is a
            part.

        :return: *True* if split, *False* if not.
        :rtype: bool

        :raise TypeError: If this part is or the splitter not a curve or
            surface part.
        """
        other_part = None
        if isinstance(splitter, Part):
            other_part = splitter
        splitter = shape_of_entity(splitter)

        split = SplitShapes()
        if rebuild_both:
            split.set_args([self._shape, splitter])
        else:
            split.set_args([self._shape])
            split.set_tools([splitter])
        split.build()
        if not split.is_done:
            return False

        parts = [self]
        if rebuild_both and other_part is not None:
            parts.append(other_part)
        for part in parts:
            part.rebuild(split)
        return True

    def rebuild(self, tool):
        """
        Rebuild the part shape with a supported tool.

        :param afem.topology.bop.BopCore tool: The tool.

        :return: *True* if modified, *False* if not.
        :rtype: bool

        :raise TypeError: If this part is not a curve or surface part.
        """
        if isinstance(self, (CurvePart, SurfacePart)):
            rebuild = RebuildShapeByTool(self._shape, tool)
        else:
            msg = 'Invalid part type in rebuild operation.'
            raise TypeError(msg)

        new_shape = rebuild.new_shape
        self.set_shape(new_shape)
        return True

    def discard_by_solid(self, solid, tol=None):
        """
        Discard shapes of the part using a solid. Any shapes of the part that
        have centroids inside the solid will be removed. Edges are checked
        for curve parts and faces are checked for surface parts.

        :param afem.topology.entities.Solid solid: The solid.
        :param float tol: The tolerance. If not provided then the part
            tolerance will be used.

        :return: *True* if shapes were discarded, *False* if not.
        :rtype: bool

        :raise TypeError: If this part is not a curve or surface part.
        """
        if isinstance(self, CurvePart):
            shapes = self.shape.edges
        elif isinstance(self, SurfacePart):
            shapes = self.shape.faces
        else:
            msg = 'Invalid part type in discard operation.'
            raise TypeError(msg)

        if tol is None:
            tol = self.shape.tol_avg

        rebuild = RebuildShapeWithShapes(self._shape)
        classifer = ClassifyPointInSolid(solid, tol=tol)

        modified = False
        for shape in shapes:
            if isinstance(self, CurvePart):
                cg = LinearProps(shape).cg
            else:
                cg = SurfaceProps(shape).cg

            classifer.perform(cg, tol)
            if classifer.is_in:
                rebuild.remove(shape)
                modified = True

        if not modified:
            return False

        new_shape = rebuild.apply()
        self.set_shape(new_shape)
        return True

    def discard_by_dmax(self, entity, dmax):
        """
        Discard shapes of the part using a shape and a distance. If the
        distance between a shape of the part and the given shape is greater
        than *dmax*, then the shape is removed. Edges are checked
        for curve parts and faces are checked for surface parts.

        :param entity: The shape.
        :type entity: afem.topology.entities.Shape or
            afem.geometry.entities.Geometry
        :param float dmax: The maximum distance.

        :return: *True* if shapes were discarded, *False* if not.
        :rtype: bool

        :raise TypeError: If this part is not a curve or surface part.
        """
        entity = Shape.to_shape(entity)

        if isinstance(self, CurvePart):
            shapes = self.shape.edges
        elif isinstance(self, SurfacePart):
            shapes = self.shape.faces
        else:
            msg = 'Invalid part type in discard operation.'
            raise TypeError(msg)

        rebuild = RebuildShapeWithShapes(self._shape)

        modified = False
        for part_shape in shapes:
            dmin = DistanceShapeToShape(entity, part_shape).dmin
            if dmin > dmax:
                rebuild.remove(part_shape)
                modified = True

        if not modified:
            return False

        new_shape = rebuild.apply()
        self.set_shape(new_shape)
        return True

    def discard_by_dmin(self, entity, dmin):
        """
        Discard shapes of the part using a shape and a distance. If the
        distance between a shape of the part and the given shape is less
        than *dmin*, then the shape is removed. Edges are checked
        for curve parts and faces are checked for surface parts.

        :param entity: The shape.
        :type entity: afem.topology.entities.Shape or
            afem.geometry.entities.Geometry
        :param float dmin: The minimum distance.

        :return: *True* if shapes were discarded, *False* if not.
        :rtype: bool

        :raise TypeError: If this part is not a curve or surface part.
        """
        entity = Shape.to_shape(entity)

        if isinstance(self, CurvePart):
            shapes = self.shape.edges
        elif isinstance(self, SurfacePart):
            shapes = self.shape.faces
        else:
            msg = 'Invalid part type in discard operation.'
            raise TypeError(msg)

        rebuild = RebuildShapeWithShapes(self._shape)

        modified = False
        for part_shape in shapes:
            dmin_ = DistanceShapeToShape(entity, part_shape).dmin
            if dmin > dmin_:
                rebuild.remove(part_shape)
                modified = True

        if not modified:
            return False

        new_shape = rebuild.apply()
        self.set_shape(new_shape)
        return True

    def discard_by_cref(self, size=None):
        """
        Discard shapes of the part by using the reference curve. An infinite
        solid is created at each end of the reference curve using the curve
        tangent. Any shape that has a centroid in these solids is removed.
        For a curve part edges are discarded, for a SurfacePart faces are
        discarded.

        :param float size: Option to define a finite solid box which might be
            more robust than an infinite solid.

        :return: *True* if shapes were discard, *False* if not.
        :rtype: bool
        """
        # Create vectors at each end of the reference curve pointing "out" of
        # the part
        u1, u2 = self._cref.u1, self._cref.u2
        v1 = self._cref.deriv(u1, 1)
        v2 = self._cref.deriv(u2, 1)
        # Reverse v1 so it's "out" of the part
        v1.reverse()

        # Create planes at each end
        p1 = self._cref.eval(u1)
        p2 = self._cref.eval(u2)
        pln1 = PlaneByNormal(p1, v1).plane
        pln2 = PlaneByNormal(p2, v2).plane

        # Translate points to define half space
        if size is None:
            pref1 = p1 + 100. * v1.ijk
            pref2 = p2 + 100. * v2.ijk
            hs1 = HalfspaceBySurface(pln1, pref1).solid
            hs2 = HalfspaceBySurface(pln2, pref2).solid
        else:
            w, h = 2 * [size / 2.]
            f1 = FaceByPlane(pln1, -w, w, -h, h).face
            f2 = FaceByPlane(pln2, -w, w, -h, h).face
            v1.normalize()
            v2.normalize()
            v1.scale(size)
            v2.scale(size)
            hs1 = SolidByDrag(f1, v1).solid
            hs2 = SolidByDrag(f2, v2).solid

        # Discard by solid
        status1 = self.discard_by_solid(hs1)
        status2 = self.discard_by_solid(hs2)
        return status1 or status2

    def shared_vertices(self, other, as_compound=False):
        """
        Get vertices shared between the two parts.

        :param other: The other part or shape.
        :type other: afem.structure.entities.Part or
            afem.topology.entities.Shape
        :param bool as_compound: Option to return the shared vertices in a
            compound.

        :return: Shared vertices.
        :rtype: list(afem.topology.entities.Vertex) or
            afem.topology.entities.Compound
        """
        other = shape_of_entity(other)
        verts = self._shape.shared_vertices(other)
        if not as_compound:
            return verts
        return CompoundByShapes(verts).compound

    def shared_edges(self, other, as_compound=False):
        """
        Get edges shared between the two parts.

        :param other: The other part or shape.
        :type other: afem.structure.entities.Part or
            afem.topology.entities.Shape
        :param bool as_compound: Option to return the shared edges in a
            compound.

        :return: Shared edges.
        :rtype: list(afem.topology.entities.Edge) or
            afem.topology.entities.Compound
        """
        other = shape_of_entity(other)
        edges = self._shape.shared_edges(other)
        if not as_compound:
            return edges
        return CompoundByShapes(edges).compound

    def init_meshing(self, mesh):
        """
        Initialize the part for meshing. This includes creating node, edge, and
        face groups.

        :param afem.smesh.entities.Mesh mesh: The top-level mesh that will
            contain the groups.

        :return: None.
        """
        name = ' '.join([self.name, 'nodes'])
        self._node_group = mesh.create_group(name, mesh.NODE, self.shape)

        name = ' '.join([self.name, 'edges'])
        self._edge_group = mesh.create_group(name, mesh.EDGE, self.shape)

        name = ' '.join([self.name, 'faces'])
        self._face_group = mesh.create_group(name, mesh.FACE, self.shape)

    @classmethod
    def reset(cls):
        """
        Reset the part index counter back to 1 and the mesh to None.

        :return: None.
        """
        cls._indx = 1


class CurvePart(Part):
    """
    Base class for curve parts.
    """

    @property
    def length(self):
        """
        :return: The length of all the edges of the part.
        :rtype: float
        """
        return LinearProps(self._shape).length


class Beam1D(CurvePart):
    """
    Beam 1-D.
    """
    pass


class SurfacePart(Part):
    """
    Base class for surface parts.
    """

    @property
    def length(self):
        """
        :return: The length of the reference curve if available. Otherwise
            it returns the length of all edges of the part.
        :rtype: float
        """
        try:
            return self.cref.length
        except AttributeError:
            return LinearProps(self._shape).length

    @property
    def area(self):
        """
        :return: The area of all faces of the part.
        :rtype: float
        """
        return SurfaceProps(self._shape).area

    def fuse(self, *other_parts):
        """
        Fuse with other surface parts and rebuild both.

        :param afem.structure.entities.SurfacePart other_parts: The other
            part(s).

        :return: *True* if fused, *False* if not.
        :rtype: bool
        """
        # Putting the other parts in a compound avoids fusing them to each
        # other
        other_shapes = [part.shape for part in other_parts]
        other_compound = CompoundByShapes(other_shapes).compound

        fuse = FuseShapes(self._shape, other_compound)
        if not fuse.is_done:
            return False

        # Rebuild the part shapes
        parts = [self] + list(other_parts)
        shapes = [part.shape for part in parts]
        rebuild = RebuildShapesByTool(shapes, fuse)
        for part in parts:
            new_shape = rebuild.new_shape(part.shape)
            part.set_shape(new_shape)

        return True

    def sew(self, *other_parts):
        """
        Sew with other parts and rebuild all parts.

        :param afem.structure.entities.SurfacePart other_parts: The other
            part(s).

        :return: *True* if sewed, *False* if not.
        :rtype: bool
        """
        parts = [self] + list(other_parts)
        shapes = [self._shape] + [part.shape for part in other_parts]

        tol = float(mean([shape.tol_avg for shape in shapes], dtype=float))
        max_tol = max([shape.tol_max for shape in shapes])

        sew = SewShape(tol=tol, max_tol=max_tol, cut_free_edges=True,
                       non_manifold=True)
        for part in parts:
            sew.add(part.shape)
        sew.perform()

        for part in parts:
            if not sew.is_modified(part.shape):
                continue
            mod_shape = sew.modified(part.shape)
            part.set_shape(mod_shape)
        return True

    def merge(self, other, unify=False):
        """
        Merge other surface part or shape with this one.

        :param other: The other part or shape.
        :type other: afem.structure.entities.SurfacePart or
            afem.topology.entities.Shape
        :param bool unify: Option to attempt to unify same domains.

        :return: *True* if merged, *False* if not.
        :rtype: bool
        """
        # Fuse the parts
        fuse = FuseShapes(self._shape, other)
        if not fuse.is_done:
            return False

        # Reset the shape
        self.set_shape(fuse.shape)

        # Unify if possible
        if not unify:
            return True
        return self.unify()

    def unify(self, edges=True, faces=True, bsplines=False):
        """
        Attempt to unify the same domains of the part shape.

        :param bool edges: Option to unify all possible edges.
        :param bool faces: Option to unify all possible faces.
        :param bool bsplines: Option to concatenate the curves of edges if they
            are C1 continuous.

        :return: *True* if unified, *False* if not.
        :rtype: bool
        """
        unify = UnifyShape(self._shape, edges, faces, bsplines)
        new_shape = unify.shape
        self.set_shape(new_shape)

    def split_local(self, subshape, tool):
        """
        Locally split the faces of he sub-shape in the context of the part
        shape.

        :param afem.topology.entities.Shape subshape: The sub-shape.
        :param tool: The tool to split the sub-shape with.
        :type tool: afem.topology.entities.Shape or
            afem.geometry.entities.Surface

        :return: *True* if split, *False* if not.
        :rtype: bool
        """
        bop = LocalSplit(subshape, tool, self._shape)
        if not bop.is_done:
            return False

        self.rebuild(bop)
        return True

    def cut_hole(self, d, ds, u0=None, is_rel=False):
        """
        Cut a hole in the part and update its shape (experimental).

        :param float d: Diameter of hole.
        :param float ds: The distance.
        :param float u0: The parameter. If not provided the first parameter
            of the reference curve will be used.
        :param bool is_rel: Option specifying if the distance is absolute or
            a relative to the length of the reference curve. If relative, then
            *ds* is multiplied by the curve length to get the absolute value.

        :return: The status of the Boolean operation that will cut the hole.
            *True* if the operation was performed, *False* if not.
        :rtype: bool
        """
        # Intersect the shape with a plane to use for the hole height location
        pln = self.plane_from_parameter(ds, u0, is_rel)
        bop = IntersectShapes(self._shape, pln)
        wires = WiresByShape(bop.shape).wires
        los = LengthOfShapes(wires)
        max_length = los.max_length
        wire = los.longest_shape
        if not isinstance(wire, Wire):
            return False
        mid_length = max_length / 2.
        p = PointAlongShape(wire, mid_length).point
        u, v = self.sref.invert(p)
        v = self.sref.norm(u, v)
        v = CheckGeom.to_direction(v)
        ax1 = Axis1(p, v)

        # Cut the hole
        r = d / 2.
        bop = CutCylindricalHole(self._shape, r, ax1)
        self.set_shape(bop.shape)

        return bop.is_done

    def cut_holes(self, n, d):
        """
        Cut holes along the reference curve at evenly spaced intervals
        (experimental).

        :param int n: The number of holes.
        :param float d: The diameter.

        :return: None.
        """
        pac = PointsAlongCurveByNumber(self.cref, n + 2)
        for u in pac.parameters[1:-1]:
            self.cut_hole(d, 0., u)


class WingPart(SurfacePart):
    """
    Base class for wing parts.
    """
    pass


class Spar(WingPart):
    """
    Wing spar.
    """
    pass


class Rib(WingPart):
    """
    Wing rib.
    """
    pass


class FuselagePart(SurfacePart):
    """
    Base class for fuselage parts.
    """
    pass


class Bulkhead(FuselagePart):
    """
    Bulkhead.
    """
    pass


class Floor(FuselagePart):
    """
    Floor.
    """
    pass


class Frame(FuselagePart):
    """
    Frame.
    """
    pass


class Skin(SurfacePart):
    """
    Skin.
    """
    pass


class Stiffener1D(CurvePart):
    """
    1-D stiffener for surface parts.
    """
    pass


class Stiffener2D(SurfacePart):
    """
    2-D stiffener for surface parts.
    """
    pass


class Stringer(SurfacePart):
    """
    Stringer.
    """
    pass


class Beam2D(SurfacePart):
    """
    Beam 2-D.
    """
    pass
