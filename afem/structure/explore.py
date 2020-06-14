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
from afem.topology.create import CompoundByShapes

__all__ = ["ExploreParts"]


class ExploreParts(object):
    """
    Tool to explore parts.
    """

    @staticmethod
    def shared_edges(parts, others, as_compound=False):
        """
        Collect the shared edges between two sets of parts.

        :param collections.Sequence(afem.structure.entities.Part) parts: The
            first set of parts.
        :param collections.Sequence(afem.structure.entities.Part) others: The
            other set of parts.
        :param bool as_compound: Option to return the shared edges as a
            compound.

        :return: List of shared edges.
        :rtype: list(afem.topology.entities.Edge) or
            afem.topology.entities.Compound
        """
        shape1 = CompoundByShapes([part.shape for part in parts]).compound
        shape2 = CompoundByShapes([part.shape for part in others]).compound
        return shape1.shared_edges(shape2, as_compound)
