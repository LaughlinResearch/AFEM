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
from afem.structure.group import Group, GroupAPI
from afem.topology.fix import FixShape

__all__ = ["FixGroup"]


class FixGroup(object):
    """
    Attempt to fix the shapes of each part in an group using
    :class:`.FixShape`. Subgroups are included by default.

    :param group: The group. If ``None`` then the active group is used.
    :type group: str or afem.structure.group.Group or None
    :param float precision: Basic precision value.
    :param float min_tol: Minimum allowed tolerance.
    :param float max_tol: Maximum allowed tolerance.

    :raise TypeError: If an :class:`.Group` instance is not found.
    """

    def __init__(self, group=None, precision=None, min_tol=None, max_tol=None):
        group = GroupAPI.get_group(group)
        if not isinstance(group, Group):
            raise TypeError('Could not find group.')

        parts = group.get_parts()
        compound = group.get_shape()

        fix = FixShape(compound, precision, min_tol, max_tol)

        for part in parts:
            new_shape = fix.apply(part.shape)
            part.set_shape(new_shape)

    @staticmethod
    def limit_tolerance(group=None, tol=1.0e-7):
        """
        Limit tolerances for the group shapes.

        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None
        :param float tol: Target tolerance.

        :return: *True* if at least one tolerance of a sub-shape was modified.
        :rtype: bool

        :raise TypeError: If an :class:`.Group` instance is not found.
        """
        group = GroupAPI.get_group(group)
        if not isinstance(group, Group):
            raise TypeError('Could not find group.')

        shape = group.get_shape()
        return FixShape.limit_tolerance(shape, tol)

    @staticmethod
    def set_tolerance(group=None, tol=1.0e7):
        """
        Enforce tolerance on the given group.

        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None
        :param float tol: The tolerance.

        :return: None.

        :raise TypeError: If an :class:`.Group` instance is not found.
        """
        group = GroupAPI.get_group(group)
        if not isinstance(group, Group):
            raise TypeError('Could not find group.')

        shape = group.get_shape()
        return FixShape.set_tolerance(shape, tol)
