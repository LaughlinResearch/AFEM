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
from OCCT.BRepBuilderAPI import BRepBuilderAPI_Sewing
from OCCT.BRepTools import BRepTools_Modifier
from OCCT.ShapeBuild import ShapeBuild_ReShape
from OCCT.ShapeCustom import ShapeCustom_BSplineRestriction
from OCCT.ShapeUpgrade import (ShapeUpgrade_ShapeDivideClosed,
                               ShapeUpgrade_ShapeDivideContinuity,
                               ShapeUpgrade_UnifySameDomain)
from OCCT.TopTools import (TopTools_DataMapOfShapeShape,
                           TopTools_IndexedMapOfShape)

from afem.geometry.entities import Geometry
from afem.topology.entities import Shape, Edge, Compound

__all__ = ["DivideClosedShape", "DivideContinuityShape", "DivideC0Shape",
           "UnifyShape", "SewShape", "RebuildShapeWithShapes",
           "RebuildShapeByTool", "RebuildShapesByTool",
           "ShapeBSplineRestriction"]


class DivideClosedShape(object):
    """
    Divide all closed faces in a shape.

    :param afem.topology.entities.Shape shape: The shape.
    """

    def __init__(self, shape):
        tool = ShapeUpgrade_ShapeDivideClosed(shape.object)
        tool.Perform()
        self._shape = Shape.wrap(tool.Result())

    @property
    def shape(self):
        """
        :return: The divided shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape


class DivideContinuityShape(object):
    """
    Divide a shape for a given continuity and tolerance.

    :param afem.topology.entities.Shape shape: The shape.
    :param float tol: The tolerance.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity: The continuity to divide.
    """

    def __init__(self, shape, tol=0.001, continuity=Geometry.C1):
        tool = ShapeUpgrade_ShapeDivideContinuity(shape.object)
        tool.SetTolerance(tol)
        tool.SetBoundaryCriterion(continuity)
        tool.SetPCurveCriterion(continuity)
        tool.SetSurfaceCriterion(continuity)
        tool.Perform()
        self._shape = Shape.wrap(tool.Result())

    @property
    def shape(self):
        """
        :return: The divided shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape


class DivideC0Shape(DivideContinuityShape):
    """
    Divide a shape at all C0 boundaries to form a C1 shape.

    :param afem.topology.entities.Shape shape: The shape.
    :param float tol: The tolerance.
    """

    def __init__(self, shape, tol=0.001):
        super(DivideC0Shape, self).__init__(shape, tol, Geometry.C1)


class UnifyShape(object):
    """
    Unify edges and faces of a shape that lie on the same geometry.

    :param afem.topology.entities.Shape shape: The shape.
    :param bool edges: Option to unify all possible edges.
    :param bool faces: Option to unify all possible faces.
    :param bool bsplines: Option to concatenate the curves of edges if they
        are C1 continuous.
    """

    def __init__(self, shape, edges=True, faces=True, bsplines=False):
        tool = ShapeUpgrade_UnifySameDomain(shape.object, edges, faces,
                                            bsplines)
        tool.Build()
        self._shape = Shape.wrap(tool.Shape())
        self._history = tool.History()

    @property
    def shape(self):
        """
        :return: The unified shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape

    def modified(self, shape):
        """
        Return a list of shapes modified from the given shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: List of modified shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._history.Modified(shape.object))

    def generated(self, shape):
        """
        Return a list of shapes generated from the given shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: List of generated shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return Shape.from_topods_list(self._history.Generated(shape.object))

    def is_deleted(self, shape):
        """
        Check to see if shape is deleted.

        :param afem.topology.entities.Wire shape: The shape.

        :return: *True* if deleted, *False* if not.
        :rtype: bool
        """
        return self._history.IsRemoved(shape.object)


class SewShape(object):
    """
    Sew the shape.

    :param afem.topology.entities.Shape shape: The context shape to sew.
    :param float tol: Sewing tolerance. If *None* is provided then the
        average tolerance of the shape will be used. If no shape is
        provided, then a default value of 1.0e-7 is used.
    :param float min_tol: Minimum tolerance.
    :param float max_tol: Maximum tolerance.
    :param bool cut_free_edges: Option for cutting of free edges.
    :param bool non_manifold: Option for non-manifold processing.

    .. note::

        If *shape* is *None* then the user is expected to manually load the
        shape and perform the operation.
    """

    def __init__(self, shape=None, tol=None, min_tol=None, max_tol=None,
                 cut_free_edges=False, non_manifold=False):
        if tol is None:
            if shape is None:
                tol = 1.0e-7
            else:
                tol = shape.tol_max

        self._tool = BRepBuilderAPI_Sewing(tol, True, True, cut_free_edges,
                                           non_manifold)

        if min_tol is not None:
            self._tool.SetMinTolerance(min_tol)
        if max_tol is not None:
            self._tool.SetMaxTolerance(max_tol)

        if shape is not None:
            self._tool.Load(shape.object)
            self._tool.Perform()

    def load(self, shape):
        """
        Load the context shape to sew.

        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        self._tool.Load(shape.object)

    def add(self, shape):
        """
        Add a shape to be sewed or controlled.

        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        self._tool.Add(shape.object)

    def perform(self):
        """
        Perform the sewing operation.

        :return: None.
        """
        self._tool.Perform()

    @property
    def sewed_shape(self):
        """
        :return: The sewed shape. May be a null shape if nothing is
            constructed.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.SewedShape())

    @property
    def n_free_edges(self):
        """
        :return: Number of free edges.
        :rtype: int
        """
        return self._tool.NbFreeEdges()

    @property
    def free_edges(self):
        """
        :return: Free edges.
        :rtype: list(afem.topology.entities.Edge)
        """
        edges = []
        for i in range(1, self.n_free_edges + 1):
            e = Edge(self._tool.FreeEdge(i))
            edges.append(e)
        return edges

    @property
    def n_multiple_edges(self):
        """
        :return: Number of edges connected to more than two faces.
        :rtype: int
        """
        return self._tool.NbMultipleEdges()

    @property
    def multiple_edges(self):
        """
        :return: Multiple edges.
        :rtype: list(afem.topology.entities.Edge)
        """
        edges = []
        for i in range(1, self.n_free_edges + 1):
            e = Edge(self._tool.MultipleEdge(i))
            edges.append(e)
        return edges

    @property
    def n_manifold_edges(self):
        """
        :return: Number of manifold edges.
        :rtype: int
        """
        return self._tool.NbContigousEdges()

    @property
    def manifold_edges(self):
        """
        :return: Manifold edges.
        :rtype: list(afem.topology.entities.Edge)
        """
        edges = []
        for i in range(1, self.n_free_edges + 1):
            e = Edge(self._tool.ContigousEdge(i))
            edges.append(e)
        return edges

    def is_modified(self, shape):
        """
        Check to see if input shape has been modified.

        :param afem.topology.entities.Shape shape: The shape.

        :return: *True* if modified, *False* if not.
        :rtype: bool
        """
        self._tool.IsModified(shape.object)

    def modified(self, shape):
        """
        Get a modified shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: The modified shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.Modified(shape.object))

    def is_modified_subshape(self, subshape):
        """
        Check to see if input sub-shape has been modified.

        :param afem.topology.entities.Shape subshape: The sub-shape.

        :return: *True* if modified, *False* if not.
        :rtype: bool
        """
        self._tool.IsModifiedSubShape(subshape.object)

    def modified_subshape(self, subshape):
        """
        Get a modified sub-shape.

        :param afem.topology.entities.Shape subshape: The sub-shape.

        :return: The modified sub-shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.ModifiedSubShape(subshape.object))


class RebuildShapeWithShapes(object):
    """
    Rebuild a shape by requesting substitutions on a shape.

    :param afem.topology.entities.Shape old_shape: The old shape.
    """

    def __init__(self, old_shape):
        self._tool = ShapeBuild_ReShape()
        self._old_shape = old_shape

    def remove(self, old_shape):
        """
        Request to remove the old shape.

        :param afem.topology.entities.Shape old_shape: The old shape. This is
            usually a sub-shape of the original old shape.

        :return: None.
        """
        self._tool.Remove(old_shape.object)

    def replace(self, old_shape, new_shapes):
        """
        Request to replace the old shape with a list of new shapes.

        :param afem.topology.entities.Shape old_shape: The old shape. This is
            usually a sub-shape of the original old shape.
        :param list(afem.topology.entities.Shape) new_shapes: The new shapes.

        :return: None.
        """

        compound = Compound.by_shapes(new_shapes)
        self._tool.Replace(old_shape.object, compound.object)

    def apply(self):
        """
        Apply the substitutions to the original old shape and return a new
        shape.

        :return: The new shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.Apply(self._old_shape.object))


class RebuildShapeByTool(object):
    """
    Rebuild a shape using a supported tool.

    :param afem.topology.entities.Shape old_shape: The old shape.
    :param tool: The tool.
    :type tool: afem.topology.bop.BopCore

    :raise ValueError: If there are no sub-shapes to substitute.

    .. note::

         This tool will first try to make substitutions on the faces of the
         shape. If no faces exist, it will try the edges. If no edges exist it
         will try the vertices.
    """

    def __init__(self, old_shape, tool):
        rebuild = RebuildShapeWithShapes(old_shape)

        # Old shapes
        old_shapes = old_shape.faces
        if not old_shapes:
            old_shapes = old_shape.edges
        if not old_shapes:
            old_shapes = old_shape.vertices
        if not old_shapes:
            raise ValueError('No sub-shapes to substitute.')

        # Delete and replace
        for shape in old_shapes:
            # Deleted
            if tool.is_deleted(shape):
                rebuild.remove(shape)
                continue

            # Modified
            mod_shapes = [s for s in tool.modified(shape)
                          if not s.is_same(shape)]
            if mod_shapes:
                rebuild.replace(shape, mod_shapes)

        self._new_shape = rebuild.apply()

    @property
    def new_shape(self):
        """
        :return: The new shape after substitutions.
        :rtype: afem.topology.entities.Shape
        """
        return self._new_shape


class RebuildShapesByTool(object):
    """
    Rebuild multiple shapes using a supported tool. This method is intended
    to address the case where modified shapes from an old shape are not
    duplicated in adjacent shapes, like what would happen if rebuilding a
    single shape at a time without any context. If a modified shape has
    already been replaced in an old shape and is encountered again,
    it is not substituted in the later shape. This method will first try to
    make substitutions on the faces of the shape. If not faces exist it will
    try the edges. If no edges exist it will try the vertices.

    :param collections.Sequence(afem.topology.entities.Shape) old_shapes: The
        old shapes.
    :param tool: The tool.
    :type tool: afem.topology.bop.BopCore
    """

    def __init__(self, old_shapes, tool):
        reshape = ShapeBuild_ReShape()

        self._new_shapes = TopTools_DataMapOfShapeShape()
        index_map = TopTools_IndexedMapOfShape()

        for old_shape in old_shapes:
            # Old shapes
            shapes = old_shape.faces
            if not shapes:
                shapes = old_shape.edges
            if not shapes:
                shapes = old_shape.vertices
            if not shapes:
                continue

            # Delete and replace
            for shape in shapes:
                # Deleted
                if tool.is_deleted(shape):
                    reshape.Remove(shape.object)
                    continue

                # Modified considering shapes already used
                mod_shapes = tool.modified(shape)
                replace_shapes = []
                for mod_shape in mod_shapes:
                    if index_map.Contains(mod_shape.object):
                        continue
                    replace_shapes.append(mod_shape)
                    index_map.Add(mod_shape.object)

                if replace_shapes:
                    new_shape = Compound.by_shapes(replace_shapes)
                    reshape.Replace(shape.object, new_shape.object)

            new_shape = Shape.wrap(reshape.Apply(old_shape.object))
            self._new_shapes.Bind(old_shape.object, new_shape.object)

    def new_shape(self, old_shape):
        """
        Get the new shape from the old shape.

        :param afem.topology.entities.Shape old_shape: The old shape provided
            in the initial inputs.

        :return: The new shape after substitutions.
        :rtype: afem.topology.entities.Shape

        :raises RuntimeError: If the old shape is not a key in the final
            results.
        """
        return Shape.wrap(self._new_shapes.Find(old_shape.object))


class ShapeBSplineRestriction(object):
    """
    Re-approximate shape surfaces with B-splines.

    :param afem.topology.entities.Shape shape: The shape.
    :param bool is_mutable: Flag for mutable input.
    :param bool approx_srf: Flag to approximate surface.
    :param bool approx_crv3d: Flag to approximate 3-d curves.
    :param bool approx_crv2d: Flag to approximate 2-d curves.
    :param float tol3d: Tolerance for 3-d approximations.
    :param float tol2d: Tolerance for 2-d approximations.
    :param int dmax: Maximum allowed degree for approximation.
    :param int nmax: Maximum allowed number of segments for approximation.
    :param bool degree: If *True*, the approximation is made with degree
        limited by *dmax* but at the expense of *nmax*. If *False*, the
        approximation is made with number of spans limited by *nmax* but at the
        expense of *dmax*.
    :param bool rational: If *True*, the approximation for rational B-Spline
        and Bezier are converted to polynomial.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity3d: Desired continuity for 3-d
        curve and surface approximation.
    :param OCCT.GeomAbs.GeomAbs_Shape continuity2d: Desired continuity for 2-d
        curve and surface approximation.
    """

    def __init__(self, shape, is_mutable=False, approx_srf=True,
                 approx_crv3d=True, approx_crv2d=True, tol3d=0.01,
                 tol2d=1.0e-6, dmax=9, nmax=10000, degree=True, rational=False,
                 continuity3d=Geometry.C1, continuity2d=Geometry.C2):
        self._the_mod = ShapeCustom_BSplineRestriction(approx_srf,
                                                       approx_crv3d,
                                                       approx_crv2d, tol3d,
                                                       tol2d, continuity3d,
                                                       continuity2d,
                                                       dmax, nmax, degree,
                                                       rational)

        self._modifier = BRepTools_Modifier(is_mutable)
        self._modifier.Init(shape.object)
        self._modifier.Perform(self._the_mod)

    @property
    def is_done(self):
        """
        :return: *True* if modification was successful.
        :rtype: bool
        """
        return self._modifier.IsDone()

    @property
    def error_curve2d(self):
        """
        :return: Error for 2-d curve approximation.
        :rtype: float
        """
        return self._the_mod.Curve2dError()

    @property
    def error_curve3d(self):
        """
        :return: Error for 3-d curve approximation.
        :rtype: float
        """
        return self._the_mod.Curve3dError()

    @property
    def error_surface(self):
        """
        :return: Error for surface approximation.
        :rtype: float
        """
        return self._the_mod.SurfaceError()

    @property
    def nspan(self):
        """
        :return: Number of spans for approximation.
        :rtype: int
        """
        return self._the_mod.NbOfSpan()

    def modified_shape(self, shape):
        """
        Return the modified shape corresponding to the given shape.

        :param afem.topology.entities.Shape shape: A shape.

        :return: The modified shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._modifier.ModifiedShape(shape))
