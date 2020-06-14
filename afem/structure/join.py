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

from afem.structure.entities import SurfacePart
from afem.topology.bop import FuseShapes, IntersectShapes, SplitShapes
from afem.topology.create import CompoundByShapes, EdgeByCurve
from afem.topology.entities import Shape
from afem.topology.modify import RebuildShapesByTool, SewShape
from afem.config import logger

__all__ = ["FuseSurfaceParts", "FuseSurfacePartsByCref", "CutParts",
           "SewSurfaceParts", "SplitParts", "FuseGroups"]


class FuseSurfaceParts(object):
    """
    Fuse together multiple surface parts and rebuild their shapes.

    :param parts: The surface parts.
    :type parts: collections.Sequence(afem.structure.entities.SurfacePart)
    :param parts: The other surface parts.
    :type tools: collections.Sequence(afem.structure.entities.SurfacePart)
    :param float fuzzy_val: Fuzzy tolerance value.
    """

    def __init__(self, parts, tools, fuzzy_val=None):
        bop = FuseShapes(fuzzy_val=fuzzy_val)

        parts = list(parts)
        other_parts = list(tools)
        args = [part.shape for part in parts]
        bop.set_args(args)
        tools = [part.shape for part in tools]
        bop.set_tools(tools)
        bop.build()

        rebuild = RebuildShapesByTool(args + tools, bop)
        for part in parts + other_parts:
            new_shape = rebuild.new_shape(part.shape)
            part.set_shape(new_shape)

        self._is_done = bop.is_done
        self._fused_shape = bop.shape

    @property
    def is_done(self):
        """
        :return: *True* if operation is done, *False* if not.
        :rtype: bool
        """
        return self._is_done

    @property
    def shape(self):
        """
        :return: The fused shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._fused_shape


class FuseSurfacePartsByCref(object):
    """
    Attempt to automatically fuse together adjacent parts based on the
    possible intersection of their reference curve. The part shapes are
    rebuilt in place.

    :param parts: The surface parts.
    :type parts: collections.Sequence(afem.structure.entities.SurfacePart)
    :param float tol: The tolerance to use for checking possible
        intersections of the reference curves. Default is the maximum
        tolerance of the part shape.

    :raises TypeError: If a given part is not a surface part.
    """

    def __init__(self, parts, tol=None):
        self._is_done = False

        for part in parts:
            if not isinstance(part, SurfacePart):
                msg = 'Part is not a surface part.'
                raise TypeError(msg)

        # Test all combinations of parts for intersection of reference curve
        join_parts = []
        main_parts = []
        nparts = len(parts)
        for i in range(0, nparts - 1):
            main = parts[i]
            other_parts = []
            for j in range(i + 1, nparts):
                other = parts[j]
                if not main.has_cref or not other.has_cref:
                    continue
                if tol is None:
                    tol1 = main.shape.tol_max
                    tol2 = other.shape.tol_max
                    _tol = max(tol1, tol2)
                else:
                    _tol = tol
                e1 = EdgeByCurve(main.cref).edge
                e2 = EdgeByCurve(other.cref).edge
                bop = IntersectShapes(e1, e2, fuzzy_val=_tol)
                if not bop.vertices:
                    continue
                # Store potential join
                msg = 'Found joint between {} and {}.'.format(main.name,
                                                              other.name)
                logger.info(msg)
                other_parts.append(other)
            if other_parts:
                main_parts.append(main)
                join_parts.append(other_parts)

        # Join the parts
        for main, other_parts in zip(main_parts, join_parts):
            main.fuse(*other_parts)
            self._is_done = True

    @property
    def is_done(self):
        """
        :return: *True* if at least one fuse was performed, *False* if not.
        :rtype: bool
        """
        return self._is_done


class CutParts(object):
    """
    Cut each part with a shape and rebuild the part shape.

    :param parts: The parts to cut.
    :type parts: collections.Sequence(afem.structure.entities.Part)
    :param shape: The shape to cut with.
    :type shape: afem.topology.entities.Shape or afem.geometry.entities.Surface
    """

    def __init__(self, parts, shape):
        parts = list(parts)

        shape2 = Shape.to_shape(shape)

        # Loop through each since since that seems to be more robust
        self._status = {}
        for part in parts:
            status = part.cut(shape2)
            self._status[part] = status

        # shapes = [part.shape for part in parts]
        # shape1 = CompoundByShapes(shapes).compound
        #
        # bop = CutShapes(shape1, shape2)
        # self._shape = bop.shape
        #
        # rebuild = RebuildShapesByTool(shapes, bop)
        # for part in parts:
        #     new_shape = rebuild.new_shape(part.shape)
        #     part.set_shape(new_shape)

    def was_cut(self, part):
        """
        Check the status of the cut operation.

        :param afem.structure.entities.Part part: The part to check.

        :return: *True* if part was cut, *False* if not.
        :rtype:
        """
        return self._status[part]


class SewSurfaceParts(object):
    """
    Sew edges of the surface parts and rebuild their shapes.

    :param parts: The parts to sew.
    :type parts: collections.Sequence(afem.structure.entities.SurfacePart)
    :param float tol: The tolerance. If not provided then the average
        tolerance from all part shapes will be used.
    :param float max_tol: Maximum tolerance. If not provided then the maximum
        tolerance from all part shapes will be used.
    """

    def __init__(self, parts, tol=None, max_tol=None):
        parts = list(parts)
        shapes = [part.shape for part in parts]

        if tol is None:
            tol = mean([shape.tol_avg for shape in shapes], dtype=float)

        if max_tol is None:
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


class SplitParts(object):
    """
    Split part shapes and rebuild in place.

    :param parts: The parts that will be split and rebuilt.
    :type parts: collections.Sequence(afem.structure.entities.Part)
    :param tools: The parts or shapes used to split the parts but are not
        modified.
    :type tools: collection.Sequence(afem.structure.entities.Part)
    :param float fuzzy_val: Fuzzy tolerance value.
    """

    def __init__(self, parts, tools=None, fuzzy_val=None):
        bop = SplitShapes(fuzzy_val=fuzzy_val)

        args = [part.shape for part in parts]
        bop.set_args(args)

        if tools is not None:
            tools = [part.shape for part in tools]
            bop.set_tools(tools)

        bop.build()

        rebuild = RebuildShapesByTool(args, bop)
        for part in parts:
            new_shape = rebuild.new_shape(part.shape)
            part.set_shape(new_shape)

        self._is_done = bop.is_done
        self._split_shape = bop.shape

    @property
    def is_done(self):
        """
        :return: *True* if operation is done, *False* if not.
        :rtype: bool
        """
        return self._is_done

    @property
    def shape(self):
        """
        :return: The split shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._split_shape


class FuseGroups(object):
    """
    Fuse groups and rebuild the part shapes. This tool puts all the part
    shapes into compounds before the Boolean operation.

    :param groups: The groups.
    :type groups: collections.Sequence(afem.structure.group.Group)
    :param float fuzzy_val: Fuzzy tolerance value.
    :param bool include_subgroup: Option to recursively include parts
            from all subgroups.

    :raise ValueError: If less than two groups are provided.
    """

    def __init__(self, groups, fuzzy_val=None, include_subgroup=True):
        if len(groups) < 2:
            raise ValueError('Not enough groups to fuse. Need at least '
                             'two.')

        bop = FuseShapes(fuzzy_val=fuzzy_val)

        groups = list(groups)
        parts1 = groups[0].get_parts(include_subgroup)
        shapes1 = [part.shape for part in parts1]
        shape1 = CompoundByShapes(shapes1).compound
        bop.set_args([shape1])

        tools = []
        other_parts = []
        for group in groups[1:]:
            parts = group.get_parts(include_subgroup)
            other_parts += parts
            shapes = [part.shape for part in parts]
            shape = CompoundByShapes(shapes).compound
            tools.append(shape)
        bop.set_tools(tools)

        bop.build()

        all_parts = parts1 + other_parts
        all_shapes = [part.shape for part in all_parts]
        rebuild = RebuildShapesByTool(all_shapes, bop)
        for part in all_parts:
            new_shape = rebuild.new_shape(part.shape)
            part.set_shape(new_shape)

        self._bop = bop

    @property
    def is_done(self):
        """
        :return: *True* if operation is done, *False* if not.
        :rtype: bool
        """
        return self._bop.is_done

    @property
    def shape(self):
        """
        :return: The fused shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._bop.shape
