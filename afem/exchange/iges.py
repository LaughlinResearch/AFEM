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
from OCCT.IFSelect import IFSelect_RetDone
from OCCT.IGESControl import IGESControl_Reader, IGESControl_Writer
from OCCT.Interface import Interface_Static

from afem.config import Settings, units_dict
from afem.topology.entities import Shape

__all__ = ["IgesWrite", "IgesRead"]


class IgesWrite(object):
    """
    Write shape to an IGES file.

    :param str units: Units to convert IGES file to.
    :param int modecr: Option for writing faces. If 0, faces will be translated
        to IGES 144 (Trimmed Surface) entities. If 1, faces will be translated
        to IGES 510 (Face) entities and the IGES face will contain BRep
        entities.
    """

    def __init__(self, units='inch', modecr=0):
        self._writer = IGESControl_Writer(units, modecr)
        try:
            units = units_dict[units]
        except KeyError:
            units = Settings.units
        Interface_Static.SetCVal_('write.step.unit', units)

    @property
    def object(self):
        """
        :return: The IGES writer object.
        :rtype: OCCT.IGESControl.IGESControl_Writer
        """
        return self._writer

    def add_shape(self, shape):
        """
        Add the shape to the exported entities.

        :param afem.topology.entities.Shape shape: The shape.

        :return: *True* if shape was transferred, *False* if not.
        :rtype: bool
        """
        return self._writer.AddShape(shape.object)

    def add_geom(self, geom):
        """
        Add the geometry to the exported entities.

        :param OCCT.Geom.Geom_Geometry geom: The geometry.

        :return: *True* if shape was transferred, *False* if not.
        :rtype: bool

        .. note::

            The input type in ``Geom_Geometry`` and not the wrapper.
        """
        return self._writer.AddGeom(geom)

    def write(self, fn='afem.igs'):
        """
        Write the IGES file.

        :param str fn: The filename.

        :return: *True* if written, *False* if not.
        :rtype: bool
        """
        return self._writer.Write(fn)


class IgesRead(object):
    """
    Read an IGES file.

    :param str fn: The file to read.
    """

    def __init__(self, fn):
        self._reader = IGESControl_Reader()

        # Read file
        status = self._reader.ReadFile(fn)
        if status != IFSelect_RetDone:
            raise RuntimeError("Error reading IGES file.")

        # Convert to desired units
        Interface_Static.SetCVal_("xstep.cascade.unit", Settings.units)

        # Transfer
        nroots = self._reader.TransferRoots()
        if nroots > 0:
            self._shape = Shape.wrap(self._reader.OneShape())

    @property
    def object(self):
        """
        :return: The IGES reader object.
        :rtype: OCCT.IGESControl.IGESControl_Reader
        """
        return self._reader

    @property
    def shape(self):
        """
        :return: The main shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape
