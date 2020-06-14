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
from OCCT.IFSelect import (IFSelect_RetError,
                           IFSelect_RetDone)
from OCCT.Interface import Interface_Static
from OCCT.STEPConstruct import STEPConstruct
from OCCT.STEPControl import (STEPControl_AsIs, STEPControl_Writer,
                              STEPControl_Reader)
from OCCT.TCollection import TCollection_HAsciiString

from afem.config import Settings, units_dict
from afem.topology.entities import Shape

__all__ = ["StepWrite", "StepRead"]


class StepWrite(object):
    """
    Write shape to a STEP file.
    
    :param str schema: Schema for STEP file ('AP203', or 'AP214').
    :param units: Units to convert STEP file to.
    :type units: str or None
    :param product_name: The name of the STEP product entry. If more than
        one product is generated during translation, then OpenCASCADE will
        automatically append a unique integer.
    :type product_name: str or None
    :param assembly_mode: Mode for writing assemblies (0, 1, or 2).
    :type assembly_mode: int or None

    .. note::
        The assembly modes are as follows:

        * 0 (off, default): Writes STEP files without assemblies.
        * 1(on): Writes all shapes in the form of STEP assemblies.
        * 2(auto): Writes shapes having a structure of (possibly nested)
          compounds in the form of STEP assemblies, single shapes are written
          without assembly structures.
    """

    def __init__(self, schema='AP203', units=None, product_name=None,
                 assembly_mode=None):
        self._writer = STEPControl_Writer()
        self._fp = self._writer.WS().TransferWriter().FinderProcess()
        Interface_Static.SetCVal_('write.step.schema', schema)

        try:
            units = units_dict[units]
        except KeyError:
            units = Settings.units
        Interface_Static.SetCVal_('write.step.unit', units)

        if product_name is not None:
            Interface_Static.SetCVal_('write.step.product.name', product_name)

        if assembly_mode is not None:
            Interface_Static.SetIVal_('write.step.assembly', assembly_mode)

    @property
    def object(self):
        """
        :return: The STEP writer object.
        :rtype: OCCT.STEPControl.STEPControl_Writer
        """
        return self._writer

    def transfer(self, *shapes):
        """
        Transfer and add the shapes to the exported entities.

        :param afem.topology.entities.Shape shapes: The shape(s).

        :return: *True* if shape was transferred, *False* if not.
        :rtype: bool
        """
        added_shape = False
        for shape in shapes:
            shape = Shape.to_shape(shape)
            if not shape:
                continue
            status = self._writer.Transfer(shape.object, STEPControl_AsIs)
            if int(status) < int(IFSelect_RetError):
                added_shape = True
        return added_shape

    def set_name(self, shape, name):
        """
        Set the name of the STEP entity for the given shape. The shape(s)
        should be transferred before naming them.

        :param afem.topology.entities.Shape shape: The shape (or sub-shape).
        :param str name: The name.

        :return: *True* if name is set, *False* otherwise.
        :rtype: bool
        """
        item = STEPConstruct.FindEntity_(self._fp, shape.object)
        if not item:
            return False

        item.SetName(TCollection_HAsciiString(name))
        return True

    def write(self, fn='afem.stp'):
        """
        Write the STEP file.

        :param str fn: The filename.

        :return: *True* if written, *False* if not.
        :rtype: bool
        """
        status = self._writer.Write(fn)
        return int(status) < int(IFSelect_RetError)


class StepRead(object):
    """
    Read a STEP file.

    :param str fn: The file to read.
    """

    def __init__(self, fn):
        self._reader = STEPControl_Reader()
        self._tr = self._reader.WS().TransferReader()

        # Read file
        status = self._reader.ReadFile(fn)
        if status != IFSelect_RetDone:
            raise RuntimeError("Error reading STEP file.")

        # Convert to desired units
        Interface_Static.SetCVal_("xstep.cascade.unit", Settings.units)

        # Transfer
        nroots = self._reader.TransferRoots()
        if nroots > 0:
            self._shape = Shape.wrap(self._reader.OneShape())

    @property
    def object(self):
        """
        :return: The STEP reader object.
        :rtype: OCCT.STEPControl.STEPControl_Reader
        """
        return self._reader

    @property
    def shape(self):
        """
        :return: The main shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape

    def name_from_shape(self, shape):
        """
        Attempt to extract the name for the STEP entity that corresponds to the
        shape.

        :param afem.topology.entities.Shape shape: The shape.

        :return: The name or None if not found.
        :rtype: str or None
        """
        item = self._tr.EntityFromShapeResult(shape.object, 1)
        if not item:
            return None
        return item.Name().ToCString()
