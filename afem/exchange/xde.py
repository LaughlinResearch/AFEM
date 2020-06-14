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
from OCCT.BinXCAFDrivers import BinXCAFDrivers
from OCCT.IFSelect import IFSelect_RetError
from OCCT.Interface import Interface_Static
from OCCT.PCDM import PCDM_StoreStatus, PCDM_ReaderStatus
from OCCT.STEPCAFControl import STEPCAFControl_Reader
from OCCT.STEPCAFControl import STEPCAFControl_Writer
from OCCT.STEPConstruct import STEPConstruct
from OCCT.TCollection import (TCollection_ExtendedString,
                              TCollection_AsciiString,
                              TCollection_HAsciiString)
from OCCT.TDF import TDF_ChildIterator, TDF_Label, TDF_LabelSequence
from OCCT.TDataStd import TDataStd_Name, TDataStd_AsciiString
from OCCT.TDocStd import TDocStd_Document
from OCCT.TNaming import TNaming_NamedShape
from OCCT.XCAFApp import XCAFApp_Application
from OCCT.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_Color
from OCCT.XmlXCAFDrivers import XmlXCAFDrivers

from afem.config import units_dict, Settings
from afem.topology.entities import Shape

__all__ = ["XdeDocument", "XdeLabel"]


class XdeDocument(object):
    """
    Wrapper class to work with Extended Data Exchange.

    :param bool binary: If *True*, the document will be saved in a binary
        format. If *False*, the document will be saved in an XML format.
    """

    def __init__(self, binary=True):
        if binary:
            self._fmt = 'BinXCAF'
            self._ext = '.xbf'
        else:
            self._fmt = 'XmlXCAF'
            self._ext = '.xml'

        # Get application
        self._app = XCAFApp_Application.GetApplication_()
        if binary:
            BinXCAFDrivers.DefineFormat_(self._app)
        else:
            XmlXCAFDrivers.DefineFormat_(self._app)

        # Initialize document
        fmt = TCollection_ExtendedString(self._fmt)
        self._doc = TDocStd_Document(fmt)
        self._app.InitDocument(self._doc)

        # Exchange data
        self._shape = None
        self._step_writer = None
        self._step_fp = None

        self._init_tool()

    def _init_tool(self):
        self._tool = XCAFDoc_DocumentTool.ShapeTool_(self._doc.Main())

    @property
    def main_label(self):
        """
        :return: The main label of the document.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._doc.Main())

    @property
    def shapes_label(self):
        """
        :return: The shapes label of the document.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(XCAFDoc_DocumentTool.ShapesLabel_(self._doc.Main()))

    def open(self, fn):
        """
        Open a document.

        :param str fn: The filename.

        :return: *True* if opened, *False* if not.
        :rtype: bool
        """
        if not fn.endswith(self._ext):
            fn += self._ext

        txt = TCollection_ExtendedString(fn)
        status, self._doc = self._app.Open(txt, self._doc)
        if status != PCDM_ReaderStatus.PCDM_RS_OK:
            return False
        self._init_tool()
        return True

    def save_as(self, fn):
        """
        Save the document.

        :param str fn: The filename.

        :return: *True* if sucessfully saved, *False* if not.
        :rtype: bool
        """
        if not fn.endswith(self._ext):
            fn += self._ext

        txt = TCollection_ExtendedString(fn)
        status = self._app.SaveAs(self._doc, txt)
        return status == PCDM_StoreStatus.PCDM_SS_OK

    def close(self):
        """
        Close the document.

        :return: None.
        """
        self._doc.Close()

    def read_step(self, fn):
        """
        Read and translate a STEP file.

        :param str fn: The filename.

        :return: The shapes label.
        :rtype: afem.exchange.xde.Label.

        :raise RuntimeError: If the file cannot be read.
        """
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        reader.SetColorMode(True)
        status = reader.Perform(fn, self._doc)
        if not status:
            raise RuntimeError("Error reading STEP file.")

        self._shape = Shape.wrap(reader.Reader().OneShape())
        label = XCAFDoc_DocumentTool.ShapesLabel_(self._doc.Main())
        return XdeLabel(label)

    def transfer_step(self, schema='AP203', units=None):
        """
        Transfer the document in preparation for STEP export.

        :param str schema: Schema for STEP file ('AP203', or 'AP214').
        :param units: Units to convert STEP file to.
        :type units: str or None

        :return: *True* if transferred, *False* otherwise.
        :rtype: bool
        """
        self._step_writer = STEPCAFControl_Writer()
        self._step_writer.SetNameMode(True)
        self._step_writer.SetColorMode(True)

        Interface_Static.SetCVal_('write.step.schema', schema)
        try:
            units = units_dict[units]
        except KeyError:
            units = Settings.units
        Interface_Static.SetCVal_('write.step.unit', units)

        self._step_writer.Transfer(self._doc)

        tw = self._step_writer.ChangeWriter().WS().TransferWriter()
        self._step_fp = tw.FinderProcess()

        return True

    def set_shape_name(self, shape, name):
        """
        Set the name of the STEP entity for the given shape. The shape(s)
        should be transferred before naming them.

        :param afem.topology.entities.Shape shape: The shape (or sub-shape).
        :param str name: The name.

        :return: *True* if name is set, *False* otherwise.
        :rtype: bool
        """
        if self._step_writer is None:
            raise RuntimeError('Document has not been transferred.')

        item = STEPConstruct.FindEntity_(self._step_fp, shape.object)
        if not item:
            return False

        item.SetName(TCollection_HAsciiString(name))
        return True

    def write_step(self, fn, schema='AP203', units=None):
        """
        Write the document to a STEP file.

        :param str fn: The filename.
        :param str schema: Schema for STEP file ('AP203', or 'AP214').
        :param units: Units to convert STEP file to.
        :type units: str or None

        :return: *True* if written successfully, *False* otherwise.
        """
        if self._step_writer is None:
            self.transfer_step(schema, units)

        status = self._step_writer.Write(fn)

        return int(status) < int(IFSelect_RetError)

    def is_top_level(self, label):
        """
        Check if label is top-level as opposed to a component of assembly or
        a sub-shape.

        :param afem.exchange.xde.XdeLabel label: The label

        :return: *True* if top-level, *False* otherwise.
        :rtype: bool
        """
        return self._tool.IsTopLevel(label.object)

    def is_sub_shape(self, label, shape):
        """
        Check if the shape is a sub-shape of the shape stored on the label.

        :param afem.exchange.xde.XdeLabel label: The label
        :param afem.topology.entities.Shape shape: The shape.

        :return: *True* if a sub-shape, *False* otherwise.
        :rtype: bool
        """
        return self._tool.IsSubShape(label.object, shape.object)

    def find_shape(self, shape, find_instance=False):
        """
        Find the label corresponding to the shape. This method searches only
        top-level shapes.

        :param afem.topology.entities.Shape shape: The shape.
        :param bool find_instance: If *False*, search for the non-located shape
            in an assembly. If *True*, search for the shape with the same
            location.

        :return: The shape label if found, *None* otherwise.
        :rtype: afem.exchange.xde.XdeLabel
        """
        label = self._tool.FindShape(shape.object, find_instance)
        if not label:
            return None
        return XdeLabel(label)

    def new_shape(self):
        """
        Create a new top-level label.

        :return: The label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._tool.NewShape())

    def set_shape(self, label, shape):
        """
        Set the shape of the top-level label.

        :param afem.exchange.xde.XdeLabel label: The label.
        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        self._tool.SetShape(label.object, shape.object)

    def add_shape(self, shape, name=None, make_assy=True):
        """
        Add a new top-level shape.

        :param afem.topology.entities.Shape shape: The shape.
        :param str name: The label name.
        :param bool make_assy: If *True*, then treat compounds as assemblies.

        :return: The shape label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        label = XdeLabel(self._tool.AddShape(shape.object, make_assy))
        if name is not None:
            label.set_name(name)
        return label

    def remove_shape(self, label, remove_completely=True):
        """
        Remove a shape.

        :param afem.exchange.xde.XdeLabel label: The label.
        :param bool remove_completely: If *True*, removes all shapes. If
            *False*, only remove the shape with the same location.

        :return: *True* if removed, or *False* if not removed because it is not
            a free or top-level shape.
        :rtype: bool
        """
        return self._tool.RemoveShape(label.object, remove_completely)

    def get_shapes(self):
        """
        Get a list containing all top-level shapes.

        :return: List of top-level shapes.
        :rtype: list(afem.exchange.xde.XdeLabel)
        """
        labels = TDF_LabelSequence()
        self._tool.GetShapes(labels)
        return [XdeLabel(label) for label in labels]

    def get_shape_by_name(self, name):
        """
        Get a shape label by its name. This only applies to top-level shapes
        and will return the first match.

        :param str name: The name.

        :return: The label or *None* if not found.
        :rtype: afem.exchange.xde.XdeLabel or None
        """
        for label in self.get_shapes():
            if label.name == name:
                return label

    def find_subshape(self, label, shape):
        """
        Find a label for the sub-shape stored on the given label.

        :param afem.exchange.xde.XdeLabel label: The label.
        :param afem.topology.entities.Shape shape: The sub-shape.

        :return: The sub-shape label if found, *None* otherwise.
        :rtype: afem.exchange.xde.XdeLabel or None
        """
        sub_label = TDF_Label()
        status, sub_label = self._tool.FindSubShape(label.object, shape.object,
                                                    sub_label)
        if not status:
            return None
        return XdeLabel(sub_label)

    def add_subshape(self, label, shape, name=None):
        """
        Add a label for a sub-shape stored on the shape of the label.

        :param afem.exchange.xde.XdeLabel label: The label.
        :param afem.topology.entities.Shape shape: The sub-shape.
        :param str name: The name of the sub-shape label.

        :return: The sub-shape label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        label = XdeLabel(self._tool.AddSubShape(label.object, shape.object))
        if name is None:
            return label
        label.set_name(name)
        return label

    def set_auto_naming(self, mode):
        """
        Set the option to auto-name shape labels. This only applies to
        top-level shapes.

        :param bool mode: The mode. If *True*, added shapes are automatically
            named based on their type (e.g., "SOLID", "SHELL", etc.).

        :return: None.
        """
        self._tool.SetAutoNaming_(mode)


class XdeLabel(object):
    """
    Wrapper class for OpenCASCADE TDF_Label.

    :param OCCT.TDF.TDF_Label: The label.
    """

    def __init__(self, label):
        self._label = label

    @property
    def object(self):
        """
        :return: The underlying object.
        :rtype: OCCT.TDF.TDF_Label
        """
        return self._label

    @property
    def tag(self):
        """
        :return: The label tag.
        :rtype: int
        """
        return self._label.Tag()

    @property
    def depth(self):
        """
        :return: The depth of this label.
        :rtype: int
        """
        return self._label.Depth()

    @property
    def father(self):
        """
        :return: The father label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._label.Father())

    @property
    def has_child(self):
        """
        :return: *True* if label has children, *False* if not.
        :rtype: bool
        """
        return self._label.HasChildren()

    @property
    def nb_children(self):
        """
        :return: The number of children.
        :rtype: int
        """
        return self._label.NbChildren()

    @property
    def is_null(self):
        """
        :return: Check if the label is null.
        :rtype: bool
        """
        return self._label.IsNull()

    @property
    def is_root(self):
        """
        :return: Check if the label is a root.
        :rtype: bool
        """
        return self._label.IsRoot()

    @property
    def root(self):
        """
        :return: The root label of this label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._label.Root())

    @property
    def name(self):
        """
        :return: The label name.
        :rtype: str or None
        """
        name = TDataStd_Name()
        status, name = self._label.FindAttribute(name.GetID_(), name)
        if status:
            return name.Get().ToExtString()
        return None

    @property
    def shape(self):
        """
        :return: The label shape.
        :rtype: afem.topology.entities.Shape or None
        """
        shape = TNaming_NamedShape()
        status, shape = self._label.FindAttribute(shape.GetID_(), shape)
        if status:
            return Shape.wrap(shape.Get())
        return None

    @property
    def string(self):
        """
        :return: The label string.
        :rtype: str or None
        """
        string = TDataStd_AsciiString()
        status, string = self._label.FindAttribute(string.GetID_(), string)
        if status:
            return string.Get().ToCString()
        return None

    @property
    def color(self):
        """
        :return: The label color.
        :rtype: OCCT.Quantity.Quantity.Color or None
        """
        color = XCAFDoc_Color()
        status4, color = self._label.FindAttribute(color.GetID_(), color)
        if status4:
            return color.GetColor()
        return None

    @property
    def children_iter(self):
        """
        :return: Yield the children labels.
        :rtype: collections.Iterable(afem.exchange.xde.XdeLabel)
        """
        iter_ = TDF_ChildIterator(self._label, False)
        while iter_.More():
            label = iter_.Value()
            iter_.Next()
            yield XdeLabel(label)

    def is_equal(self, other):
        """
        Check if this label equals the other.

        :param afem.exchange.xde.XdeLabel other: The other label.

        :return: *True* if equal, *False* if not.
        :rtype: bool
        """
        return self._label.IsEqual(other._label)

    def is_descendant(self, other):
        """
        Check if this label is a descendant of the other.

        :param afem.exchange.xde.XdeLabel other: The other label.

        :return: *True* if a descendant, *False* if not.
        :rtype: bool
        """
        return self._label.IsDescendant(other._label)

    def new_child(self):
        """
        Create a new child label of this one.

        :return: Child label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._label.NewChild())

    def find_child(self, tag):
        """
        Find a child label with ``tag``.

        :param int tag: The child label tag. If the tag doesn't exist then a
            new child will be created using that tag.

        :return: Child label.
        :rtype: afem.exchange.xde.XdeLabel
        """
        return XdeLabel(self._label.FindChild(tag, True))

    def set_name(self, name):
        """
        Set label name.

        :param str name: The name.

        :return: None.
        """
        txt = TCollection_ExtendedString(name)
        TDataStd_Name.Set_(self._label, txt)

    def set_string(self, string):
        """
        Set label name.

        :param str string: The string.

        :return: None.
        """
        txt = TCollection_AsciiString(string)
        TDataStd_AsciiString.Set_(self._label, txt)

    def set_color(self, color):
        """
        Set label color.

        :param OCCT.Quantity.Quantity_Color color: The color.

        :return: None.
        """
        XCAFDoc_Color.Set_(self._label, color)
