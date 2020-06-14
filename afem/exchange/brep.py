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
from OCCT.BRep import BRep_Builder
from OCCT.BRepTools import BRepTools
from OCCT.TopoDS import TopoDS_Shape

from afem.topology.entities import Shape


def write_brep(shape, fn):
    """
    Write a BREP file using the shape.

    :param afem.topology.entities.Shape shape: The shape.
    :param str fn: The filename.

    :return: None.
    """
    BRepTools.Write_(shape.object, fn)


def read_brep(fn):
    """
    Read a BREP file and return the shape.

    :param str fn: The filename.

    :return: The shape.
    :rtype: afem.topology.entities.Shape
    """
    shape = TopoDS_Shape()
    builder = BRep_Builder()
    BRepTools.Read_(shape, fn, builder)

    return Shape.wrap(shape)
