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
__all__ = ["DiscardByCref"]


class DiscardByCref(object):
    """
    Discard shapes of the parts by using their reference curves. An infinite
    solid is created at each end of the reference curve using the curve
    tangent. Any shape that has a centroid in these solids is removed.
    For a curve part edges are discarded, for a SurfacePart faces are
    discarded.

    :param list(afem.structure.entities.Part) parts: The parts.
    """

    def __init__(self, parts):
        self._status = {}

        for part in parts:
            if part.has_cref:
                status = part.discard_by_cref()
                self._status[part] = status
            else:
                self._status[part] = False

    def was_modified(self, part):
        """
        Check to see if part was modified.

        :param afem.structure.entities.Part part: The part.

        :return: *True* if entities were discarded from the part, *False*
            otherwise.
        :rtype: bool
        """
        try:
            return self._status[part]
        except KeyError:
            return False
