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
from afem.structure.entities import Part, SurfacePart, WingPart, Spar, Rib

__all__ = ["CheckPart"]


class CheckPart(object):
    """
    Check structural components.
    """

    @staticmethod
    def is_part(part):
        """
        Check to see if the part is a structural component.

        :param part: Part to check.

        :return: *True* if part is a structural component, *False* if not.
        :rtype: bool
        """
        return isinstance(part, Part)

    @staticmethod
    def are_parts(*parts):
        """
        Check to see if all entities are parts.

        :param parts: Parts to check.

        :return: *True* if all entities are parts, *False* if not.
        :rtype: bool
        """
        for part in parts:
            if not isinstance(part, Part):
                return False
        return True

    @staticmethod
    def is_surface_part(part):
        """
        Check to see if part is surface-based part.

        :param part:
        :return:
        """
        return isinstance(part, SurfacePart)

    @staticmethod
    def is_wing_part(part):
        """
        Check to see if the part is a wing structural component.

        :param part: Part to check.

        :return: *True* if part is a wing structural component, *False* if
            not.
        :rtype: bool
        """
        return isinstance(part, WingPart)

    @staticmethod
    def is_spar(part):
        """
        Check to see if the part is a spar.

        :param part: Part to check.

        :return: *True* if the part is a spar, *False* if not.
        :rtype: bool
        """
        return isinstance(part, Spar)

    @staticmethod
    def is_rib(part):
        """
        Check to see if the part is a rib.

        :param part: Part to check.

        :return: *True* if the part is a rib, *False* if not.
        :rtype: bool
        """
        return isinstance(part, Rib)
