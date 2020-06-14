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
from math import sqrt

from OCCT.Extrema import (Extrema_ExtPC, Extrema_ExtCC, Extrema_POnCurv,
                          Extrema_ExtPS, Extrema_ExtCS, Extrema_POnSurf)
from OCCT.GeomProjLib import GeomProjLib

from afem.adaptor.entities import AdaptorCurve, AdaptorSurface
from afem.geometry.check import CheckGeom
from afem.geometry.entities import Curve, Line

__all__ = ["PointProjector", "ProjectPointToCurve",
           "ProjectPointToSurface", "CurveProjector", "ProjectCurveToPlane",
           "ProjectCurveToSurface"]


class PointProjector(object):
    """
    Base class for point projections.
    """

    def __init__(self):
        self._npts = 0
        self._results = []

    @property
    def npts(self):
        """
        :return: Number of projection results.
        :rtype: int
        """
        return len(self._results)

    @property
    def success(self):
        """
        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        return self.npts > 0

    @property
    def points(self):
        """
        :return: Projected points.
        :rtype: list(afem.geometry.entities.Point)
        """
        if self.npts <= 0:
            return []
        return [results[0] for results in self._results]

    @property
    def parameters(self):
        """
        :return: Parameters of projected points.
        :rtype: list(float)
        """
        if self.npts <= 0:
            return []
        return [results[1] for results in self._results]

    @property
    def nearest_point(self):
        """
        :return: Nearest projection result to original point.
        :rtype: afem.geometry.entities.Point
        """
        return self._results[0][0]

    @property
    def nearest_param(self):
        """
        :return: Parameter(s) of nearest point on curve(surface).
        :rtype: float or tuple(float, float)
        """
        return self._results[0][1]

    @property
    def dmin(self):
        """
        :return: Minimum distance of all projection results.
        :rtype: float
        """
        return self._results[0][2]

    def point(self, indx=1):
        """
        Return the point result by index.

        :param int indx: Index for point.

        :return: Projected point.
        :rtype: afem.geometry.entities.Point
        """
        return self._results[indx - 1][0]

    def parameter(self, indx=1):
        """
        Return the parameter result by index.

        :param int indx: Index for parameter.

        :return: Parameter of point. For a curve projection a single float *u*
            will be returned. For a surface projection a tuple containing the
            *u* and *v* parameters will be returned (u, v).
        :rtype: float or tuple(float, float)
        """
        return self._results[indx - 1][1]

    def distance(self, indx=1):
        """
        Return the projection distance by index.

        :param int indx: Index for distance.

        :return: Projection distance between original point and projection
            result.
        :rtype: float
        """
        return self._results[indx - 1][2]


class ProjectPointToCurve(PointProjector):
    """
    Project a point to a curve.

    :param point_like pnt: Point to project.
    :param crv: Curve to project to.
    :type crv: afem.adaptor.entities.AdaptorCurve or
        afem.geometry.entities.Curve or afem.topology.entities.Edge or
        afem.topology.entities.Wire
    :param array_like direction: Direction of projection. If *None* then a
        normal projection will be performed. By providing a direction the
        tool actually performs a line-curve intersection. This is generally
        not recommended but provided by request.
    :param bool update: Option to update the point's location to match the
        nearest point.
    """

    def __init__(self, pnt, crv, direction=None, update=False):
        super(ProjectPointToCurve, self).__init__()

        pnt = CheckGeom.to_point(pnt)
        direction = CheckGeom.to_direction(direction)
        adp_crv = AdaptorCurve.to_adaptor(crv)
        self._results = []

        if not direction:
            ext = Extrema_ExtPC(pnt, adp_crv.object)
            npts = ext.NbExt()
            for i in range(1, npts + 1):
                poc = ext.Point(i)
                ui = poc.Parameter()
                pi = adp_crv.eval(ui)
                di = sqrt(ext.SquareDistance(i))
                self._results.append([pi, ui, di])
        else:
            # Use minimum distance between line and curve to project point
            # along a direction.
            line = Line.by_direction(pnt, direction)
            adp_crv2 = AdaptorCurve.to_adaptor(line)
            ext = Extrema_ExtCC(adp_crv.object, adp_crv2.object)
            npts = ext.NbExt()
            for i in range(1, npts + 1):
                poc1, poc2 = Extrema_POnCurv(), Extrema_POnCurv()
                ext.Points(i, poc1, poc2)
                ui = poc1.Parameter()
                pi = adp_crv.eval(ui)
                di = sqrt(ext.SquareDistance(i))
                self._results.append([pi, ui, di])

        # Sort by distance and return.
        if self._results:
            self._results.sort(key=lambda lst: lst[2])

        if update:
            pnt.set_xyz(self.nearest_point)


class ProjectPointToSurface(PointProjector):
    """
    Project a point to a surface.

    :param point_like pnt: Point to project.
    :param srf: Surface to project to.
    :type srf: afem.adaptor.entities.AdaptorSurface or
        afem.geometry.entities.Surface or afem.topology.entities.Face
    :param array_like direction: Direction of projection. If *None* then a
        normal projection will be performed. By providing a direction the
        tool actually performs a line-surface intersection. This is generally
        not recommended but provided by request.
    :param bool update: Option to update the point's location to match the
        nearest point.
    """

    def __init__(self, pnt, srf, direction=None, update=False, tol=1.0e-7):
        super(ProjectPointToSurface, self).__init__()

        pnt = CheckGeom.to_point(pnt)
        direction = CheckGeom.to_direction(direction)
        adp_srf = AdaptorSurface.to_adaptor(srf)
        self._results = []

        if not direction:
            ext = Extrema_ExtPS(pnt, adp_srf.object, tol, tol)
            npts = ext.NbExt()
            for i in range(1, npts + 1):
                pos = ext.Point(i)
                ui, vi = pos.Parameter(0., 0.)
                pi = adp_srf.eval(ui, vi)
                di = sqrt(ext.SquareDistance(i))
                self._results.append([pi, (ui, vi), di])
        else:
            # Use minimum distance between line and surface to project point
            # along a direction.
            line = Line.by_direction(pnt, direction)
            adp_crv = AdaptorCurve.to_adaptor(line)
            ext = Extrema_ExtCS(adp_crv.object, adp_srf.object, tol, tol)
            npts = ext.NbExt()
            for i in range(1, npts + 1):
                poc, pos = Extrema_POnCurv(), Extrema_POnSurf()
                ext.Points(i, poc, pos)
                ui, vi = pos.Parameter(0., 0.)
                pi = adp_srf.eval(ui, vi)
                di = sqrt(ext.SquareDistance(i))
                self._results.append([pi, (ui, vi), di])

        # Sort by distance and return.
        if self._results:
            self._results.sort(key=lambda lst: lst[2])

        if update:
            pnt.set_xyz(self.nearest_point)


class CurveProjector(object):
    """
    Base class for curve projections.
    """

    def __init__(self):
        self._crv = None

    @property
    def success(self):
        """
        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        return self._crv is not None

    @property
    def curve(self):
        """
        :return: The projected curve.
        :rtype: afem.geometry.entities.Curve
        """
        return self._crv


class ProjectCurveToPlane(CurveProjector):
    """
    Project a curve to a plane along a direction.

    :param afem.geometry.entities.Curve crv: Curve to project.
    :param afem.geometry.entities.Plane pln: Plane to project to.
    :param array_like direction: Direction of projection. If *None* is
        provided, then the curve is projected normal to the plane.

    :raise RuntimeError: If the OCC method fails to project the curve to the
        plane.
    """

    def __init__(self, crv, pln, direction=None, keep_param=True):
        super(ProjectCurveToPlane, self).__init__()

        direction = CheckGeom.to_direction(direction)
        if not CheckGeom.is_direction(direction):
            direction = pln.object.Pln().Axis().Direction()

        # OCC projection
        hcrv = GeomProjLib.ProjectOnPlane_(crv.object, pln.object, direction,
                                           keep_param)

        self._crv = Curve(hcrv)


class ProjectCurveToSurface(CurveProjector):
    """
    Project a curve to a surface. Only normal projections are supported.

    :param afem.geometry.entities.Curve crv: Curve to project.
    :param afem.geometry.entities.Surface srf: Surface to project to.

    :raise RuntimeError: If the OCC method fails to project the curve to the
        plane.
    """

    def __init__(self, crv, srf):
        super(ProjectCurveToSurface, self).__init__()

        # OCC projection
        hcrv = GeomProjLib.Project_(crv.object, srf.object)
        self._crv = Curve(hcrv)
