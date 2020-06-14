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
from OCCT.BRepTools import BRepTools_WireExplorer
from OCCT.ShapeAnalysis import ShapeAnalysis_FreeBounds

from afem.topology.entities import Vertex, Edge, Compound

__all__ = ["ExploreWire", "ExploreFreeEdges"]


class ExploreWire(object):
    """
    Explore the edges of a wire.

    :param afem.topology.entities.Wire wire: The wire.
    :param afem.topology.entities.Face face: The face.
    """

    def __init__(self, wire, face=None):
        if face is None:
            explorer = BRepTools_WireExplorer(wire.object)
        else:
            explorer = BRepTools_WireExplorer(wire.object, face.object)

        edges = []
        current_verts = []
        while explorer.More():
            ei = Edge(explorer.Current())
            vi = Vertex(explorer.CurrentVertex())
            edges.append(ei)
            current_verts.append(vi)
            explorer.Next()

        # CurrentVertex doesn't get the last vertex. Try to get it.
        ordered_verts = list(current_verts)
        if edges:
            vi = Vertex(explorer.CurrentVertex())
            ordered_verts.append(vi)

        self._edges = edges
        self._current_verts = current_verts
        self._ordered_verts = ordered_verts

    @property
    def nedges(self):
        """
        :return: Number of edges.
        :rtype: int
        """
        return len(self._edges)

    @property
    def edges(self):
        """
        :return: The ordered edges.
        :rtype: list(afem.topology.entities.Edge)
        """
        return self._edges

    @property
    def current_vertices(self):
        """
        :return: The result of the BRepTools_WireExplorer::CurrentVertex
            method. As the explorer traverses the edges, this stores the vertex
            connecting the current edge to the previous one. This will not be a
            complete list of ordered vertices.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self._current_verts

    @property
    def ordered_vertices(self):
        """
        :return: Attempt to provide the ordered vertices of a wire. If the wire
            is closed the first and last vertices will be the same.
        :rtype: list(afem.topology.entities.Vertex)
        """
        return self._ordered_verts


class ExploreFreeEdges(object):
    """
    Explore the free bounds of a shape.

    :param afem.topology.entities.Shape shape: The shape.
    """

    def __init__(self, shape):
        tool = ShapeAnalysis_FreeBounds(shape.object)
        closed_wires = Compound(tool.GetClosedWires())
        open_wires = Compound(tool.GetOpenWires())
        self._closed_wires = closed_wires.wires
        self._open_wires = open_wires.wires
        self._edges = closed_wires.edges + open_wires.edges

    @property
    def closed_wires(self):
        """
        :return: Closed wires of free edges.
        :rtype: list(afem.topology.entities.Wire)
        """
        return self._closed_wires

    @property
    def open_wires(self):
        """
        :return: Open wires of free edges.
        :rtype: list(afem.topology.entities.Wire)
        """
        return self._open_wires

    @property
    def free_edges(self):
        """
        :return: All free edges of the shape.
        :rtype: list(afem.topology.entities.Edge)
        """
        return self._edges
