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
from afem.topology.create import FaceBySurface
from afem.topology.entities import Solid
from afem.topology.transform import mirror_shape

__all__ = ["Body"]


class Body(ShapeHolder):
    """
    Generic class for solid bodies and encapsulating necessary
    information when creating structural components.

    :param shape: The shape which should be a solid.
    :type shape: afem.topology.entities.Solid or afem.topology.entities.Shape
    :param str name: The name.
    """

    def __init__(self, shape, name='Body'):
        super(Body, self).__init__(name, shape, expected_types=Solid)

    def mirrored(self, pln, name=None):
        """
        Mirror this Body using the plane.

        :param afem.geometry.entities.Plane pln: The plane.
        :param str name: The name of the new Body.

        :return: Mirrored Body.
        :rtype: afem.oml.entities.Body
        """
        solid = mirror_shape(self.shape, pln)
        body = Body(solid, name)
        if self.sref is not None:
            sref = self.sref.copy()
            sref.mirror(pln)
            body.set_sref(sref)
        return body

    @staticmethod
    def save_bodies(fn, bodies, binary=True):
        """
        Save Body instances.

        :param str fn: The filename.
        :param collections.Sequence(afem.oml.entities.Body) bodies: The Body
            instances
        :param bool binary: If *True*, the document will be saved in a binary
            format. If *False*, the document will be saved in an XML format.

        :return: *True* if saved, *False* otherwise.
        :rtype: bool

        .. note::

            This method only saves the name, shape, reference surface, and
            color of the Body. Other user-defined metadata is currently not
            saved.
        """
        from afem.exchange.xde import XdeDocument
        # Create document
        doc = XdeDocument(binary)

        # Add the bodies
        for body in bodies:
            label = doc.add_shape(body.shape, body.name, False)
            label.set_string('Body')
            label.set_color(body.color)

            # Sref
            if body.sref is not None:
                face = FaceBySurface(body.sref).face
                label = doc.add_shape(face, body.name, False)
                label.set_string('SREF')

        return doc.save_as(fn)

    @staticmethod
    def load_bodies(fn):
        """
        Load saved Body instances.

        :param str fn: The filename. The extension should be either ".xbf" for
            a binary file or ".xml" for an XML file.

        :return: A dictionary where the key is the name and the value is the
            Body instance.
        :rtype: dict(str, afem.oml.entities.Body)

        :raise TypeError: If the file extension type is not supported.
        """
        from afem.exchange.xde import XdeDocument

        if fn.endswith('.xbf'):
            binary = True
        elif fn.endswith('.xml'):
            binary = False
        else:
            raise TypeError('Document type not supported.')

        # Open document
        doc = XdeDocument(binary)
        doc.open(fn)

        # Get the main label and iterate on top-level children which
        # should be parts
        label = doc.shapes_label
        body_data = []
        sref_to_body = {}
        for current in label.children_iter:
            name = current.name
            shape = current.shape
            type_ = current.string
            color = current.color

            if None in [name, shape, type_]:
                continue

            # Check for reference geometry
            if type_ == 'SREF' and shape.is_face:
                sref_to_body[name] = shape.surface
                continue

            # Add part data
            body_data.append((name, shape, color))

        # Create bodies
        label_to_bodies = {}
        for label, shape, color in body_data:
            sref = None
            if label in sref_to_body:
                sref = sref_to_body[label]
            body = Body(shape, label)
            if sref is not None:
                body.set_sref(sref)
            if color is not None:
                r, g, b = color.Red(), color.Green(), color.Blue()
                body.set_color(r, g, b)
            label_to_bodies[label] = body

        return label_to_bodies
