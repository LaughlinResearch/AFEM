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
from afem.base.entities import NamedItem
from afem.exchange.xde import XdeDocument
from afem.structure.utils import order_parts_by_id
from afem.topology.create import CompoundByShapes, EdgeByCurve, FaceBySurface

__all__ = ["Group", "GroupAPI"]


class Group(NamedItem):
    """
    Group of parts.

    :param str name: The name.
    :param parent: The parent group, if any.
    :type parent: afem.structure.group.Group or None
    """

    def __init__(self, name, parent=None):
        super(Group, self).__init__(name)
        self._parent = parent
        self._children = set()
        self._parts = set()
        if isinstance(self._parent, Group):
            self._parent._children.add(self)

    @property
    def parent(self):
        """
        :return: The parent group, if any.
        :rtype: afem.structure.group.Group or None
        """
        return self._parent

    @property
    def parts(self):
        """
        :return: List of all parts.
        :rtype: list(afem.structure.entities.Part)
        """
        return list(self._parts)

    def activate(self):
        """
        Activate this group.

        :return: None
        """
        GroupAPI._active = self

    def add_parts(self, *parts):
        """
        Add parts to the group.

        :param afem.structure.entities.Part parts: Part(s) to add.

        :return: None.
        """
        part_set = set(parts)
        self._parts.update(part_set)

    def get_part(self, name):
        """
        Get a part in the group by name.

        :param str name: Part name.

        :return: The part.
        :rtype: afem.structure.entities.Part

        :raise KeyError: If the part is not found.
        """
        for part in self.parts:
            if part.name == name:
                return part
        raise KeyError('Part with given name could not be found in the '
                       'group.')

    def get_parts(self, include_subgroup=True, rtype=None, order=False):
        """
        Get all the parts from the group and its subgroups.

        :param bool include_subgroup: Option to recursively include parts
            from any subgroups.
        :param rtype: Option to return only parts of a certain type. Provide a
            class to check if the part is of the given type using
            *isinstance()*.
        :param bool order: Option to order parts by their ID.

        :return: List of parts.
        :rtype: list(afem.structure.entities.Part)
        """
        parts = []
        for part in self.parts:
            if rtype is None:
                parts.append(part)
            else:
                if isinstance(part, rtype):
                    parts.append(part)

        if include_subgroup:
            for group in self._children:
                parts += group.get_parts(True, rtype)

        if not order:
            return parts
        return order_parts_by_id(parts)

    def remove_part(self, name):
        """
        Remove a part from the group if present.

        :param str name: Part name.

        :return: None.
        """
        part = self.get_part(name)
        self._parts.discard(part)

    def get_shape(self, include_subgroup=True):
        """
        Get a shape derived from all parts in the group. This puts all the
        parts into a single compound which could be used as the master shape
        for the meshing process.

        :param bool include_subgroup: Option to recursively include parts
            from any subgroups.

        :return: The part shapes as a compound.
        :rtype: afem.topology.entities.Compound
        """
        parts = self.get_parts(include_subgroup)
        shapes = [part.shape for part in parts]
        return CompoundByShapes(shapes).compound

    def create_subgroup(self, name, active=True):
        """
        Create a new sub-group of this one.

        :param str name: The group name.
        :param bool active: Option to make this new group the active one.

        :return: The new sub-group.
        :rtype: afem.structure.group.Group
        """
        return GroupAPI.create_group(name, self, active)

    @staticmethod
    def parts_to_compound(parts):
        """
        Convert the list of parts into a single compound using each of their
        shapes.

        :param collections.Sequence(afem.structure.entities.Part) parts: The
            parts.

        :return: The compound.
        :rtype: afem.topology.entities.Compound
        """
        return CompoundByShapes([part.shape for part in parts]).compound


class GroupAPI(object):
    """
    Group API. This stores all created groups so data can be accessed
    from one place. There is always a master model named '_master' that is
    created at initialization. This will be the parent of all groups
    that are not provided a parent when created. No groups should be
    named '_master'.
    """
    _master = Group('_master', None)
    _all = {'_master': _master}
    _active = _master

    @classmethod
    def reset(cls):
        """
        Reset master group and data structure and reset Part index back to 1.
        This should delete all groups unless they are referenced somewhere
        else.

        :return: None.
        """
        cls._master = Group('_master', None)
        cls._all = {'_master': cls._master}
        cls._active = cls._master

        from afem.structure.entities import Part

        Part.reset()

    @classmethod
    def get_master(cls):
        """
        Get the master group.

        :return: The master group.
        :rtype: afem.structure.group.Group
        """
        return cls._master

    @classmethod
    def get_active(cls):
        """
        Get the active group.

        :return: Active group.
        :rtype: afem.structure.group.Group
        """
        group = cls._active
        if not isinstance(group, Group):
            return cls._master
        return group

    @classmethod
    def get_group(cls, group=None):
        """
        Get an group. If a string is provided then the group name
        will be used to get the group. If a group instance is
        provided then it is simply returned. If ``None`` is provided then
        the active group is returned.

        :param group: Group to get.
        :type group: str or afem.structure.group.Group or None

        :return: The group.
        :rtype: afem.structure.group.Group
        """
        if isinstance(group, Group):
            return group

        try:
            return cls._all[group]
        except KeyError:
            return cls.get_active()

    @classmethod
    def make_active(cls, group):
        """
        Activate the group. This method uses the *GroupAPI.get_group()*
        method to get the group to activate.

        :param group: Group to activate.
        :type group: str or afem.structure.group.Group or None

        :return: None.
        """
        group = cls.get_group(group)
        group.activate()

    @classmethod
    def create_group(cls, name, parent=None, active=True, *parts):
        """
        Create an group.

        :param str name: The name.
        :param parent: The parent group. If ``None`` then the master
            group is used.
        :type parent: str or afem.structure.group.Group or None
        :param bool active: Option to make the new group the active one.

        :param afem.structure.entities.Part parts: The part(s) to add to the
            group, if any.

        :return: New group.
        :rtype: afem.structure.group.Group
        """
        if name in cls._all:
            return None
        if parent is None:
            parent = cls._master
        else:
            parent = cls.get_group(parent)
        group = Group(name, parent)
        if active:
            cls._active = group
        group.add_parts(*parts)
        cls._all[name] = group
        return group

    @classmethod
    def add_parts(cls, group, *parts):
        """
        Add parts to the group.

        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None
        :param afem.structure.entities.Part parts: The part(s) to add.

        :return: None.
        """
        group = cls.get_group(group)
        group.add_parts(*parts)

    @classmethod
    def get_part(cls, name, group=None):
        """
        Get a part from the group using its name.
        
        :param str name: The part name.
        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None
         
        :return: The part.
        :rtype: afem.structure.entities.Part

        :raise KeyError: If the part is not found.
        """
        group = cls.get_group(group)
        return group.get_part(name)

    @classmethod
    def get_parts(cls, group=None, include_subgroup=True, rtype=None,
                  order=False):
        """
        Get parts from group.

        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None
        :param bool include_subgroup: Option to recursively include parts
            from any subgroups.
        :param rtype: Option to return only parts of a certain type. Provide a
            class to check if the part is of the given type using
            *isinstance()*.
        :param bool order: Option to order parts by their ID.

        :return: The parts.
        :rtype: list(afem.structure.entities.Part)
        """
        group = cls.get_group(group)
        return group.get_parts(include_subgroup, rtype, order)

    @classmethod
    def remove_part(cls, name, group=None):
        """
        Remove a part from the group if present.

        :param str name: Part name.
        :param group: The group. If ``None`` then the active group is
            used.
        :type group: str or afem.structure.group.Group or None

        :return: None.
        """
        group = cls.get_group(group)
        group.remove_part(name)

    @classmethod
    def get_shape(cls, group='_master', include_subgroup=True):
        """
        Get a shape derived from all parts in the group. This puts all the
        parts into a single compound which could be used as the master shape
        for the meshing process.

        :param group: The group. If ``None`` then the active group is
            used. By default the master model is used.
        :type group: str or afem.structure.group.Group or None
        :param bool include_subgroup: Option to recursively include parts
            from any subgroups.

        :return: The parts as a compound.
        :rtype: afem.topology.entities.Compound
        """
        group = cls.get_group(group)
        return group.get_shape(include_subgroup)

    @classmethod
    def save_model(cls, fn, binary=True):
        """
        Save the model.

        :param str fn: The filename.
        :param bool binary: If *True*, the document will be saved in a binary
            format. If *False*, the document will be saved in an XML format.

        :return: *True* if saved, *False* otherwise.
        :rtype: bool
        """
        group = cls.get_master()

        # Create document and application
        doc = XdeDocument(binary)

        # Store parts as top-level shapes
        # TODO Support group hierarchy
        parts = group.get_parts()
        for part in parts:
            name = doc.add_shape(part.shape, part.name, False)
            name.set_string(part.type_name)
            name.set_color(part.color)

            # Reference curve
            if part.has_cref:
                edge = EdgeByCurve(part.cref).edge
                name = doc.add_shape(edge, part.name, False)
                name.set_string('CREF')

            # Reference surface
            if part.has_sref:
                face = FaceBySurface(part.sref).face
                name = doc.add_shape(face, part.name, False)
                name.set_string('SREF')

        return doc.save_as(fn)

    @classmethod
    def load_model(cls, fn, group=None):
        """
        Load a model.

        :param str fn: The filename. The extension should be either ".xbf" for
            a binary file or ".xml" for an XML file.
        :param afem.structure.group.Group group: The group to load the parts
            into. If *None* then the active group is used.

        :return: *True* if loaded, *False* otherwise.
        :rtype: bool

        :raise TypeError: If the file extension type is not supported.
        """
        from afem.structure.create import CreatePartByName

        if fn.endswith('.xbf'):
            binary = True
        elif fn.endswith('.xml'):
            binary = False
        else:
            raise TypeError('Document type not supported.')

        group = cls.get_group(group)

        # Open document
        doc = XdeDocument(binary)
        doc.open(fn)

        # Get the main name and iterate on top-level children which
        # should be parts
        name = doc.shapes_label
        part_data = []
        cref_to_part = {}
        sref_to_part = {}
        for current in name.children_iter:
            name = current.name
            shape = current.shape
            type_ = current.string
            color = current.color

            if None in [name, shape, type_]:
                continue

            # Check for reference geometry
            if type_ == 'CREF':
                cref_to_part[name] = shape.curve
                continue

            if type_ == 'SREF':
                sref_to_part[name] = shape.surface
                continue

            # Add part data
            part_data.append((type_, name, shape, color))

        # Create parts
        # TODO Support group hierarchy
        for type_, name, shape, color in part_data:
            cref, sref = None, None
            if name in cref_to_part:
                cref = cref_to_part[name]
            if name in sref_to_part:
                sref = sref_to_part[name]

            part = CreatePartByName(type_, name=name, shape=shape,
                                    group=group, cref=cref, sref=sref).part
            if color is not None:
                r, g, b = color.Red(), color.Green(), color.Blue()
                part.set_color(r, g, b)

        return True
