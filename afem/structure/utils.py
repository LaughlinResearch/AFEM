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
from afem.core.entities import ShapeHolder
from afem.topology.entities import Shape


def order_parts_by_id(parts):
    """
    Order the list of parts by id.

    :param list(afem.structure.entities.Part) parts: The parts.

    :return: List of parts sorted by their ID.
    :rtype: list(afem.structure.entities.Part)
    """
    part_order = [(part.id, part) for part in parts]
    part_order.sort(key=lambda tup: tup[0])
    return [row[1] for row in part_order]


def shape_of_entity(entity):
    """
    Get the shape of the entity. This method is useful if method inputs can
    either be a part or a shape. If the entity is already a shape it will be
    returned. If the entity is part the shape of the part will be returned. If
    the entity is a curve or surface then it will be converted to a shape.

    :param entity: The entity.
    :type entity: afem.geometry.entities.Geometry or
        afem.topology.entities.Shape or afem.base.entities.ShapeHolder

    :return: The shape.
    :rtype: afem.topology.entities.Shape
    """
    if isinstance(entity, ShapeHolder):
        return entity.shape
    else:
        return Shape.to_shape(entity)
