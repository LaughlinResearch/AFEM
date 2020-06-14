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
from OCCT.BRepExtrema import (BRepExtrema_DistShapeShape, BRepExtrema_IsVertex,
                              BRepExtrema_IsOnEdge, BRepExtrema_IsInFace)
from OCCT.Extrema import Extrema_ExtFlag_MIN

from afem.adaptor.entities import FaceAdaptorSurface
from afem.config import logger
from afem.geometry.check import CheckGeom
from afem.geometry.entities import Point, Direction
from afem.topology.entities import Shape, Vertex

__all__ = ["DistanceShapeToShape", "DistanceShapeToShapes",
           "DistancePointToShapes"]


class DistanceShapeToShape(object):
    """
    Calculate minimum distance between two shapes. If geometry is provided
    it will be converted to a shape.

    :param shape1: The first shape or geometry.
    :type shape1: afem.topology.entities.Shape or
        afem.geometry.entities.Geometry
    :param shape2: The second or geometry.
    :type shape2: afem.topology.entities.Shape or
        afem.geometry.entities.Geometry
    """

    def __init__(self, shape1, shape2, deflection=1.0e-7):
        shape1 = Shape.to_shape(shape1)
        shape2 = Shape.to_shape(shape2)
        self._tool = BRepExtrema_DistShapeShape(shape1.object, shape2.object,
                                                deflection,
                                                Extrema_ExtFlag_MIN)

    @property
    def is_done(self):
        """
        :return: *True* if algorithm is done, *False* if not.
        :rtype: bool
        """
        return self._tool.IsDone()

    @property
    def nsol(self):
        """
        :return: The number of solutions satisfying the minimum distance.
        :rtype:
        """
        return self._tool.NbSolution()

    @property
    def dmin(self):
        """
        :return: The minimum distance.
        :rtype: float
        """
        return self._tool.Value()

    @property
    def inner_solution(self):
        """
        :return: *True* if one of the shapes is a solid and the other is
            completely or partially inside the solid.
        :rtype: bool
        """
        return self._tool.InnerSolution()

    def point_on_shape1(self, n=1):
        """
        The point for the *n-th* solution on the first shape.

        :param int n: The index.

        :return: The point.
        :rtype: afem.geometry.entities.Point
        """
        gp_pnt = self._tool.PointOnShape1(n)
        return Point(gp_pnt.X(), gp_pnt.Y(), gp_pnt.Z())

    def point_on_shape2(self, n=1):
        """
        The point for the *n-th* solution on the second shape.

        :param int n: The index.

        :return: The point.
        :rtype: afem.geometry.entities.Point
        """
        gp_pnt = self._tool.PointOnShape2(n)
        return Point(gp_pnt.X(), gp_pnt.Y(), gp_pnt.Z())

    def support_type_shape1(self, n=1):
        """
        The type of support for the *n-th* solution on the first shape.

        :param int n: The index.

        :return: The support type.
        :rtype: OCCT.BRepExtrema.BRepExtrema_SupportType
        """
        return self._tool.SupportTypeShape1(n)

    def is_vertex_shape1(self, n=1):
        """
        Check if support type is a vertex for the first shape.

        :param int n: The index.

        :return: *True* if a vertex, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape1(n) == BRepExtrema_IsVertex

    def is_on_edge_shape1(self, n=1):
        """
        Check if support type is on an edge for the first shape.

        :param int n: The index.

        :return: *True* if on an edge, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape1(n) == BRepExtrema_IsOnEdge

    def is_in_face_shape1(self, n=1):
        """
        Check if support type is in a face for the first shape.

        :param int n: The index.

        :return: *True* if in a face, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape1(n) == BRepExtrema_IsInFace

    def support_type_shape2(self, n=1):
        """
        The type of support for the *n-th* solution on the second shape.

        :param int n: The index.

        :return: The support type.
        :rtype: OCCT.BRepExtrema.BRepExtrema_SupportType
        """
        return self._tool.SupportTypeShape2(n)

    def is_vertex_shape2(self, n=1):
        """
        Check if support type is a vertex for the second shape.

        :param int n: The index.

        :return: *True* if a vertex, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape2(n) == BRepExtrema_IsVertex

    def is_on_edge_shape2(self, n=1):
        """
        Check if support type is on an edge for the second shape.

        :param int n: The index.

        :return: *True* if on an edge, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape2(n) == BRepExtrema_IsOnEdge

    def is_in_face_shape2(self, n=1):
        """
        Check if support type is in a face for the second shape.

        :param int n: The index.

        :return: *True* if in a face, *False* otherwise.
        :rtype: bool
        """
        return self.support_type_shape2(n) == BRepExtrema_IsInFace

    def support_on_shape1(self, n=1):
        """
        Get the shape where the *n-th* solution is on the first shape.

        :param int n: The index.

        :return: The support shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.SupportOnShape1(n))

    def support_on_shape2(self, n=1):
        """
        Get the shape where the *n-th* solution is on the second shape.

        :param int n: The index.

        :return: The support shape.
        :rtype: afem.topology.entities.Shape
        """
        return Shape.wrap(self._tool.SupportOnShape2(n))

    def par_on_edge_shape1(self, n=1):
        """
        Get the parameter of the *n-th* solution if it is on an edge of the
        first shape.

        :param int n: The index.

        :return: The parameter.
        :rtype: float
        """
        return self._tool.ParOnEdgeS1(n, 0.)

    def par_on_edge_shape2(self, n=1):
        """
        Get the parameter of the *n-th* solution if it is on an edge of the
        second shape.

        :param int n: The index.

        :return: The parameter.
        :rtype: float
        """
        return self._tool.ParOnEdgeS2(n, 0.)

    def par_on_face_shape1(self, n=1):
        """
        Get the parameters of the *n-th* solution if it is in a face of the
        first shape.

        :param int n: The index.

        :return: The parameters.
        :rtype: tuple(float, float)
        """
        return self._tool.ParOnFaceS1(n, 0., 0.)

    def par_on_face_shape2(self, n=1):
        """
        Get the parameters of the *n-th* solution if it is in a face of the
        second shape.

        :param int n: The index.

        :return: The parameters.
        :rtype: tuple(float, float)
        """
        return self._tool.ParOnFaceS2(n, 0., 0.)

    def normal_on_shape1(self, n=1):
        """
        Get a unit normal on the first shape where the *n-th* solution is
        located if it is in a face.

        :param int n: The index.

        :return: The unit normal.
        :rtype: afem.geometry.entities.Direction

        :raise ValueError: If the solution is not in a face.
        """
        if not self.is_in_face_shape1(n):
            raise ValueError('The solution is not in a face.')

        face = self.support_on_shape1(n)
        u, v = self.par_on_face_shape1(n)

        adp_srf = FaceAdaptorSurface.by_face(face)
        return Direction.by_vector(adp_srf.norm(u, v))

    def normal_on_shape2(self, n=1):
        """
        Get a unit normal on the second shape where the *n-th* solution is
        located if it is in a face.

        :param int n: The index.

        :return: The unit normal.
        :rtype: afem.geometry.entities.Direction

        :raise ValueError: If the solution is not in a face.
        """
        if not self.is_in_face_shape2(n):
            raise ValueError('The solution is not in a face.')

        face = self.support_on_shape2(n)
        u, v = self.par_on_face_shape2(n)

        adp_srf = FaceAdaptorSurface.by_face(face)
        return Direction.by_vector(adp_srf.norm(u, v))


class DistanceShapeToShapes(object):
    """
    Calculate the minimum distance between a shape and other shapes. Sort the
    results by distance.

    :param afem.topology.entities.Shape shape: The main shape.
    :param list(afem.topology.entities.Shape) other_shapes: The other shapes.
    """

    def __init__(self, shape, other_shapes):
        results = []
        for shape2 in other_shapes:
            dist = DistanceShapeToShape(shape, shape2)
            if dist.nsol == 0:
                logger.warning("Could not calculate distance to a shape in "
                               "DistanceShapeToShapes tool. Continuing...")
                continue
            results.append((dist.dmin, shape2))

        results.sort(key=lambda tup: tup[0])
        self._distances = [data[0] for data in results]
        self._shapes = [data[1] for data in results]

    @property
    def dmin(self):
        """
        :return: The minimum distance of all shapes.
        :rtype: float
        """
        return self._distances[0]

    @property
    def dmax(self):
        """
        :return: The maximum distance of all shapes.
        :rtype: float
        """
        return self._distances[-1]

    @property
    def sorted_distances(self):
        """
        :return: List of sorted distances.
        :rtype: list(float)
        """
        return self._distances

    @property
    def nearest_shape(self):
        """
        :return: The nearest shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shapes[0]

    @property
    def farthest_shape(self):
        """
        :return: The farthest shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shapes[-1]

    @property
    def sorted_shapes(self):
        """
        :return: List of shapes sorted by distance.
        :rtype: list(afem.topology.entities.Shape)
        """
        return self._shapes


class DistancePointToShapes(DistanceShapeToShapes):
    """
    Calculate the minimum distance between a point and other shapes. Sort the
    results by distance. This method converts the point to a vertex and then
    uses :class:`.DistanceShapeToShapes`.

    :param point_like pnt: The point.
    :param list(afem.topology.entities.Shape) other_shapes: The other shapes.

    :raise TypeError: If *pnt* cannot be converted to a point.
    """

    def __init__(self, pnt, other_shapes):
        pnt = CheckGeom.to_point(pnt)
        if not pnt:
            raise TypeError('Invalid point type provided.')

        v = Vertex.by_point(pnt)
        super(DistancePointToShapes, self).__init__(v, other_shapes)
