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
from afem.exchange import nastran
from afem.smesh.entities import MeshGen, MeshGroup, Mesh
from afem.smesh.hypotheses import (Regular1D, NetgenAlgo2D,
                                   NetgenSimple2D, LocalLength1D,
                                   NumberOfSegments1D, MaxLength1D,
                                   QuadrangleAlgo2D, QuadrangleHypo2D)
from afem.structure.group import GroupAPI

__all__ = ["MeshVehicle"]


class MeshVehicle(object):
    """
    Tool for assisting in vehicle-level mesh generation. The "master" group
    will be used to define the top-level shape for meshing, which should
    include all the parts and therefore shapes to be meshed. Default mesh
    controls are applied to the top-level shape based on the target element
    size.

    :param float target_size: Default global element size.
    :param bool allow_quads: Option to generate quad-dominated mesh.
    """

    def __init__(self, target_size=1., allow_quads=True):
        group = GroupAPI.get_master()
        self._shape = group.get_shape()
        self._gen = MeshGen()
        self._mesh = self._gen.create_mesh(self._shape)

        # Initialize each part for meshing
        for part in group.get_parts():
            part.init_meshing(self._mesh)

        # Define global mesh control based on target size
        hyp1d = LocalLength1D(self._gen, target_size)
        alg1d = Regular1D(self._gen)
        hyp2d = NetgenSimple2D(self._gen, target_size, allow_quads=allow_quads)
        alg2d = NetgenAlgo2D(self._gen)
        self.add_controls([hyp1d, alg1d, hyp2d, alg2d])

    @property
    def shape(self):
        """
        :return: The top-level shape.
        :rtype: afem.topology.entities.Shape
        """
        return self._shape

    @property
    def gen(self):
        """
        :return: The mesh generator.
        :rtype: afem.smesh.entities.MeshGen
        """
        return self._gen

    @property
    def mesh(self):
        """
        :return: The top-level mesh.
        :rtype: afem.smesh.entities.Mesh
        """
        return self._mesh

    def add_control(self, control, shape=None):
        """
        Add a mesh control.

        :param afem.smesh.hypotheses.Hypothesis control: The control.
        :param shape: The shape the control applies to. This can be a
            sub-shape of the master shape. If not provided then the master
            shape is used.
        :type shape: afem.topology.entities.Shape

        :return: Status of adding hypothesis.
        :rtype: OCCT.SMESH.SMESH_Hypothesis.Hypothesis_Status
        """
        if shape is None:
            shape = self.shape

        return self._mesh.add_hypothesis(control, shape)

    def add_controls(self, controls, shape=None):
        """
        Add mesh controls to the shape.

        :param controls: The controls.
        :type controls:
            collections.Sequence(afem.smesh.hypotheses.Hypothesis)
        :param shape: The shape the control applies to. This can be a
            sub-shape of the master shape. If not provided then the master
            shape is used.
        :type shape: afem.topology.entities.Shape

        :return: Dictionary where the key is the control and the value is
            the control status.
        :rtype: dict

        :raise ValueError: If no shape is available to apply the control to.
        """
        status_dict = {}
        for hyp in controls:
            status = self.add_control(hyp, shape)
            status_dict[hyp] = status
        return status_dict

    def set_number_segments_1d(self, nseg, shape):
        """
        Set the number of edge segments for the shape.

        :param int nseg: The number of segments.
        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        alg = Regular1D(self.gen)
        hyp = NumberOfSegments1D(self.gen, nseg)
        self.add_controls([alg, hyp], shape)

    def set_local_length_1d(self, local_length, shape):
        """
        Set the local length of edge segments for the shape.

        :param float local_length: The local length.
        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        alg = Regular1D(self.gen)
        hyp = LocalLength1D(self.gen, local_length)
        self.add_controls([alg, hyp], shape)

    def set_max_length_1d(self, max_length, shape):
        """
        Set the max length of edge segments for the shape.

        :param float max_length: The max length.
        :param afem.topology.entities.Shape shape: The shape.

        :return: None.
        """
        alg = Regular1D(self.gen)
        hyp = MaxLength1D(self.gen, max_length)
        self.add_controls([alg, hyp], shape)

    def set_quadrangle_2d(self, shape):
        """
        Set the mesh control to use structured quadrangle mesh for the shape.

        :param afem.topology.entities.Shape shape: The shape. The algorithm is
            applied to each face of the shape only if it is applicable.

        :return: None.
        """
        alg = QuadrangleAlgo2D(self.gen)
        hyp = QuadrangleHypo2D(self.gen)
        for face in shape.faces:
            if alg.is_applicable(face):
                self.add_controls([alg, hyp], face)

    def compute(self):
        """
        Compute the mesh.

        :return: *True* if successful, *False* if not.
        :rtype: bool
        """
        return self._gen.compute(self._mesh, self._shape)

    def create_node_group(self, shape, name='node_group'):
        """
        Create a mesh node group from the shape.

        :param afem.topology.entities.Shape shape: The shape.
        :param str name: The group name.

        :return: The node group.
        :rtype: afem.smesh.entities.MeshGroup
        """
        return MeshGroup(self.mesh, name, Mesh.NODE, shape)

    def create_edge_group(self, shape, name='edge_group'):
        """
        Create a mesh edge group from the shape.

        :param afem.topology.entities.Shape shape: The shape.
        :param str name: The group name.

        :return: The edge group.
        :rtype: afem.smesh.entities.MeshGroup
        """
        return MeshGroup(self.mesh, name, Mesh.EDGE, shape)

    def create_face_group(self, shape, name='node_group'):
        """
        Create a mesh face group from the shape.

        :param afem.topology.entities.Shape shape: The shape.
        :param str name: The group name.

        :return: The face group.
        :rtype: afem.smesh.entities.MeshGroup
        """
        return MeshGroup(self.mesh, name, Mesh.FACE, shape)

    def export_nastran(self, fn):
        """
        Export the mesh to a Nastran bulk data file.

        :param str fn: The filename.

        :return: None.
        """
        nastran.export_bdf(self.mesh, fn)
