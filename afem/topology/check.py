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
from OCCT.BRepCheck import BRepCheck_Analyzer, BRepCheck_NoError
from OCCT.BRepClass3d import BRepClass3d_SolidClassifier
from OCCT.TopAbs import TopAbs_IN, TopAbs_ON, TopAbs_OUT, TopAbs_UNKNOWN

from afem.config import logger
from afem.geometry.check import CheckGeom
from afem.topology.entities import Face

__all__ = ["CheckShape", "ClassifyPointInSolid"]


def _invalid_subshapes(shape, check, errors):
    """
    Find invalid sub-shapes.
    """
    invalid = []
    for sub_shape in shape.shape_iter:
        result = check.Result(sub_shape.object)
        list_of_status = result.Status()
        for status in list_of_status:
            if status != BRepCheck_NoError:
                type_ = sub_shape.__class__.__name__
                error = str(status).split('.')[-1]
                msg = '\t{0}: {1}'.format(type_, error)
                errors.append(msg)
                invalid.append(sub_shape)
        invalid += _invalid_subshapes(sub_shape, check, errors)

    return invalid


class CheckShape(object):
    """
    Check shape and its sub-shapes for errors.

    :param afem.topology.entities.Shape shape: The shape.
    :param bool geom: Option to check geometry in additional to topology.
    """

    def __init__(self, shape, geom=True):
        self._check = BRepCheck_Analyzer(shape.object, geom)
        self._invalid = []
        self._errors = []
        if not self._check.IsValid():
            self._invalid = _invalid_subshapes(shape, self._check,
                                               self._errors)

    @property
    def is_valid(self):
        """
        :return: *True* if the shape and all of its sub-shapes are valid,
            *False* if not.
        :rtype: bool
        """
        return self._check.IsValid()

    @property
    def invalid_shapes(self):
        """
        :return: List of invalid shapes.
        :rtype: list(afem.topology.entities.Shape)
        """
        return self._invalid

    def print_errors(self):
        """
        Print the errors.

        :return: None.
        """
        for msg in self._errors:
            print(msg)

    def log_errors(self):
        """
        Log the errors at the "info" level.

        :return: None.
        """
        for msg in self._errors:
            logger.info(msg)

    def is_subshape_valid(self, shape):
        """
        Check if a sub-shape of the original shape is valid.

        :param afem.topology.entities.Shape shape: The sub-shape.

        :return: *True* if valid, *False* if not.
        """
        return self._check.IsValid(shape.object)


class ClassifyPointInSolid(object):
    """
    Classify a point in a solid.

    :param afem.topology.entities.Solid solid: The solid.
    :param point_like pnt: The point. If not provided the *perform()* method
        will need to be used.
    :param float tol: The tolerance.
    """

    def __init__(self, solid, pnt=None, tol=1.0e-7):
        pnt = CheckGeom.to_point(pnt)

        if not CheckGeom.is_point(pnt):
            self._tool = BRepClass3d_SolidClassifier(solid.object)
        else:
            self._tool = BRepClass3d_SolidClassifier(solid.object, pnt, tol)

    @property
    def is_in(self):
        """
        :return: *True* if point is in solid, *False* if not.
        :rtype: bool
        """
        return self._tool.State() == TopAbs_IN

    @property
    def is_out(self):
        """
        :return: *True* if point is outside the solid, *False* if not.
        :rtype: bool
        """
        return self._tool.State() == TopAbs_OUT

    @property
    def is_on(self):
        """
        :return: *True* if point is on the solid, *False* if not.
        :rtype: bool
        """
        return self._tool.State() == TopAbs_ON

    @property
    def is_unknown(self):
        """
        :return: *True* if classification is unknown, *False* if not.
        :rtype: bool
        """
        return self._tool.State() == TopAbs_UNKNOWN

    @property
    def is_on_face(self):
        """
        :return: *True* if point is on a face, *False* if not.
        :rtype: bool
        """
        return self._tool.IsOnAFace()

    def perform(self, pnt, tol=1.0e-7):
        """
        Perform the classification with the point and tolerance.

        :param point_like pnt: The point.
        :param float tol: The tolerance.

        :return: None.
        """
        pnt = CheckGeom.to_point(pnt)
        self._tool.Perform(pnt, tol)

    def face(self):
        """
        Get the face the point is on.

        :return: The face.
        :rtype: afem.topology.entities.Face
        """
        return Face(self._tool.Face())
