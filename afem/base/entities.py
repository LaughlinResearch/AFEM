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
from OCCT.Quantity import Quantity_TOC_RGB, Quantity_Color
from numpy.random import rand

__all__ = ["Metadata", "NamedItem", "ViewableItem"]


class Metadata(dict):
    """
    Simple class for storing data as (key, value) pairs.
    """

    def set(self, key, value):
        """
        Set data.

        :param key: The key.
        :param value: The value.

        :return: None.
        """
        self[key] = value


class NamedItem(object):
    """
    Base class for types that can be give a name.

    :param str name: The name.
    """

    def __init__(self, name='Item'):
        self._name = name
        self._metadata = Metadata()

    @property
    def name(self):
        """
        :Getter: The name.
        :Setter: Set name.
        :type: str
        """
        return self._name

    @name.setter
    def name(self, name):
        self.set_name(name)

    @property
    def metadata(self):
        """
        :return: The metadata dictionary.
        :rtype: afem.base.entities.Metadata
        """
        return self._metadata

    def set_name(self, name):
        """
        Set name.

        :param str name: The name.

        :return: None.
        """
        self._name = name


class ViewableItem(object):
    """
    Base class for types that can be viewed.
    """

    def __init__(self):
        self._color = None
        self._transparency = 0.

    @property
    def displayed_shape(self):
        """
        :return: The shape to be displayed.
        :rtype: OCCT.TopoDS.TopoDS_Shape
        """
        msg = ('Viewing this type is not yet support. '
               'Implement displayed_shape property.')
        raise NotImplementedError(msg)

    @property
    def color(self):
        """
        :return: The color or *None* if not set.
        :rtype: OCCT.Quantity.Quantity_Color or None
        """
        return self._color

    @property
    def transparency(self):
        """
        :return: The transparency.
        :rtype: float
        """
        return self._transparency

    def set_color(self, r, g, b):
        """
        Set color (0. <= r, g, b <= 1.).

        :param float r: Red.
        :param float g: Green.
        :param float b: Blue.

        :return: None.
        """
        if r > 1.:
            r /= 255.
        if g > 1.:
            g /= 255.
        if b > 1.:
            b /= 255.
        self._color = Quantity_Color(r, g, b, Quantity_TOC_RGB)

    def set_transparency(self, transparency):
        """
        Set the opacity for graphics.

        :param float transparency: Level of transparency (0 to 1).

        :return: None.
        """
        if transparency < 0.:
            transparency = 0.
        elif transparency > 1.:
            transparency = 1.
        self._transparency = transparency

    def random_color(self):
        """
        Set the color randomly.

        :return: None.
        """
        r, g, b = rand(1, 3)[0]
        self._color = Quantity_Color(r, g, b, Quantity_TOC_RGB)
