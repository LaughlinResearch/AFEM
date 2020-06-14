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
import unittest

import afem.topology.transform
from afem.exchange import brep
from afem.geometry import *
from afem.graphics import Viewer
from afem.topology import *


def show_shapes(*shapes):
    gui = Viewer()
    gui.add(*shapes)
    gui.start()


class TestTopologyCreate(unittest.TestCase):
    """
    Test cases for afem.topology.create.
    """

    def test_vertex_by_point(self):
        builder = VertexByPoint((1., 2., 3.))
        v = builder.vertex
        self.assertIsInstance(v, Vertex)
        self.assertAlmostEqual(v.point.x, 1.)
        self.assertAlmostEqual(v.point.y, 2.)
        self.assertAlmostEqual(v.point.z, 3.)

    def test_edge_by_points(self):
        builder = EdgeByPoints((0., 0., 0.), (10., 0., 0.))
        e = builder.edge
        v1 = builder.vertex1
        v2 = builder.vertex2
        self.assertIsInstance(e, Edge)
        self.assertIsInstance(v1, Vertex)
        self.assertIsInstance(v2, Vertex)
        self.assertAlmostEqual(v1.point.x, 0.)
        self.assertAlmostEqual(v1.point.y, 0.)
        self.assertAlmostEqual(v1.point.z, 0.)
        self.assertAlmostEqual(v2.point.x, 10.)
        self.assertAlmostEqual(v2.point.y, 0.)
        self.assertAlmostEqual(v2.point.z, 0.)
        self.assertAlmostEqual(e.length, 10.)

    def test_edge_by_curve(self):
        c = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        builder = EdgeByCurve(c)
        e = builder.edge
        v1 = builder.vertex1
        v2 = builder.vertex2
        self.assertIsInstance(e, Edge)
        self.assertIsInstance(v1, Vertex)
        self.assertIsInstance(v2, Vertex)
        self.assertAlmostEqual(v1.point.x, 0.)
        self.assertAlmostEqual(v1.point.y, 0.)
        self.assertAlmostEqual(v1.point.z, 0.)
        self.assertAlmostEqual(v2.point.x, 10.)
        self.assertAlmostEqual(v2.point.y, 0.)
        self.assertAlmostEqual(v2.point.z, 0.)
        self.assertAlmostEqual(e.length, 10.)

    def test_edge_by_drag(self):
        vertex = VertexByPoint((0., 0., 0.)).vertex
        builder = EdgeByDrag(vertex, (1., 0., 0.))
        e = builder.edge
        v1 = builder.first_vertex
        v2 = builder.last_vertex
        self.assertIsInstance(e, Edge)
        self.assertIsInstance(v1, Vertex)
        self.assertIsInstance(v2, Vertex)
        self.assertAlmostEqual(v1.point.x, 0.)
        self.assertAlmostEqual(v1.point.y, 0.)
        self.assertAlmostEqual(v1.point.z, 0.)
        self.assertAlmostEqual(v2.point.x, 1.)
        self.assertAlmostEqual(v2.point.y, 0.)
        self.assertAlmostEqual(v2.point.z, 0.)
        self.assertAlmostEqual(e.length, 1.)

    def test_edge_by_wire_concat(self):
        c1 = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        c2 = NurbsCurveByPoints([(10., 0., 0.), (11., 1., 0.)]).curve
        e1 = EdgeByCurve(c1).edge
        e2 = EdgeByCurve(c2).edge
        w = WireByEdges(e1, e2).wire
        edge = EdgeByWireConcat(w).edge
        self.assertIsInstance(edge, Edge)
        self.assertAlmostEqual(edge.length, 11.41421, places=5)

    def test_wire_by_edges(self):
        c1 = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        c2 = NurbsCurveByPoints([(10., 0., 0.), (11., 1., 0.)]).curve
        e1 = EdgeByCurve(c1).edge
        e2 = EdgeByCurve(c2).edge
        builder = WireByEdges(e1, e2)
        w = builder.wire
        self.assertIsInstance(w, Wire)
        self.assertAlmostEqual(w.length, 11.41421, places=5)

    def test_wire_by_connected_edges(self):
        c1 = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        c2 = NurbsCurveByPoints([(10., 0., 0.), (11., 1., 0.)]).curve
        e1 = EdgeByCurve(c1).edge
        e2 = EdgeByCurve(c2).edge
        builder = WiresByConnectedEdges([e1, e2])
        self.assertEqual(builder.nwires, 1)
        w = builder.wires[0]
        self.assertIsInstance(w, Wire)
        self.assertAlmostEqual(w.length, 11.41421, places=5)

    def test_wire_by_points(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        builder = WireByPoints([p1, p2, p3, p4], True)
        wire = builder.wire
        self.assertIsInstance(wire, Wire)
        self.assertTrue(wire.closed)
        self.assertAlmostEqual(wire.length, 4.)

    def test_wire_by_concat(self):
        c1 = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        c2 = NurbsCurveByPoints([(10., 0., 0.), (11., 1., 0.)]).curve
        e1 = EdgeByCurve(c1).edge
        e2 = EdgeByCurve(c2).edge
        w = WireByEdges(e1, e2).wire
        wire = WireByConcat(w).wire
        self.assertIsInstance(wire, Wire)
        self.assertAlmostEqual(wire.length, 11.41421, places=5)

    def test_face_by_surface(self):
        pln = PlaneByNormal().plane
        f = FaceBySurface(pln).face
        self.assertIsInstance(f, Face)

    def test_face_by_plane(self):
        pln = PlaneByNormal().plane
        f = FaceByPlane(pln, -1., 1., -1., 1.).face
        self.assertIsInstance(f, Face)

    def test_face_by_planar_wire(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        builder = FaceByPlanarWire(wire)
        face = builder.face
        self.assertIsInstance(face, Face)
        self.assertAlmostEqual(face.area, 1.)

    def test_face_by_drag(self):
        vertex = VertexByPoint((0., 0., 0.)).vertex
        e = EdgeByDrag(vertex, (1., 0., 0.)).edge
        builder = FaceByDrag(e, (0., 1., 0.))
        f = builder.face
        self.assertIsInstance(f, Face)
        self.assertAlmostEqual(f.area, 1.)

    def test_shell_by_surface(self):
        pln = PlaneByNormal().plane
        shell = ShellBySurface(pln).shell
        self.assertIsInstance(shell, Shell)

    def test_shell_by_faces(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        face = FaceByPlanarWire(wire).face
        shell = ShellByFaces([face]).shell
        self.assertIsInstance(shell, Shell)
        self.assertAlmostEqual(shell.area, 1.)

    def test_shell_by_drag(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        builder = ShellByDrag(wire, (0., 0., 1.))
        shell = builder.shell
        self.assertIsInstance(shell, Shell)
        self.assertAlmostEqual(shell.area, 4.)

    def test_shell_by_sewing(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        w1 = WireByPoints([p1, p2, p3, p4], True).wire
        f1 = FaceByPlanarWire(w1).face
        p1 = (0., 0., 0.)
        p2 = (0., 0., 1.)
        p3 = (0., 1., 1.)
        p4 = (0., 1., 0.)
        w2 = WireByPoints([p1, p2, p3, p4], True).wire
        f2 = FaceByPlanarWire(w2).face
        builder = ShellBySewing([f1, f2])
        self.assertEqual(builder.nshells, 1)
        self.assertEqual(len(builder.shell.faces), 2)

        # Non-manifold case
        p1 = (0., 0., 0.)
        p2 = (-1., 0., 0.)
        p3 = (-1., 1., 0.)
        p4 = (0., 1., 0.)
        w3 = WireByPoints([p1, p2, p3, p4], True).wire
        f3 = FaceByPlanarWire(w3).face
        builder = ShellBySewing([f1, f2, f3], non_manifold=True)
        self.assertEqual(builder.nshells, 1)
        self.assertEqual(len(builder.shell.faces), 3)

    def test_solid_by_plane(self):
        pln = PlaneByNormal().plane
        box = SolidByPlane(pln, 1., 1., 1.).solid
        self.assertIsInstance(box, Solid)
        self.assertAlmostEqual(box.volume, 1.)

    def test_solid_by_drag(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        face = FaceByPlanarWire(wire).face
        builder = SolidByDrag(face, (0., 0., 1.))
        box = builder.solid
        self.assertIsInstance(box, Solid)
        self.assertAlmostEqual(box.volume, 1.)

    def test_box_by_size(self):
        builder = BoxBySize(10., 10., 10.)
        self.assertIsInstance(builder.solid, Solid)
        self.assertIsInstance(builder.shell, Shell)
        self.assertIsInstance(builder.bottom_face, Face)
        self.assertIsInstance(builder.back_face, Face)
        self.assertIsInstance(builder.front_face, Face)
        self.assertIsInstance(builder.left_face, Face)
        self.assertIsInstance(builder.right_face, Face)
        self.assertIsInstance(builder.top_face, Face)

    def test_box_by_2_points(self):
        builder = BoxBy2Points((0, 0, 0), (10, 10, 10))
        self.assertIsInstance(builder.solid, Solid)
        self.assertIsInstance(builder.shell, Shell)
        self.assertIsInstance(builder.bottom_face, Face)
        self.assertIsInstance(builder.back_face, Face)
        self.assertIsInstance(builder.front_face, Face)
        self.assertIsInstance(builder.left_face, Face)
        self.assertIsInstance(builder.right_face, Face)
        self.assertIsInstance(builder.top_face, Face)

    def test_cylinder_by_axis(self):
        builder = CylinderByAxis(1, 10)
        self.assertIsInstance(builder.face, Face)
        self.assertIsInstance(builder.shell, Shell)
        self.assertIsInstance(builder.solid, Solid)

    def test_sphere_by_3_points(self):
        p1 = Point(0, 0, 0)
        p2 = Point(1, 0, 0)
        p3 = Point(0.5, 0.5, 0)
        builder = SphereBy3Points(p1, p2, p3)
        self.assertIsInstance(builder.face, Face)
        self.assertIsInstance(builder.shell, Shell)
        self.assertIsInstance(builder.solid, Solid)

    def test_halfspace_by_shape(self):
        pln = PlaneByNormal().plane
        f = FaceByPlane(pln, -1., 1., -1., 1.).face
        builder = HalfspaceByShape(f, (0., 0., 1.))
        hs = builder.solid
        self.assertIsInstance(hs, Solid)

    def test_points_along_shape(self):
        e = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        tool = PointAlongShape(e, 5.)
        p = tool.point
        self.assertAlmostEqual(p.x, 5.)
        self.assertAlmostEqual(p.y, 0.)
        self.assertAlmostEqual(p.z, 0.)
        self.assertAlmostEqual(tool.parameter, 5.)

    def test_points_along_shape_by_number(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        builder = PointsAlongShapeByNumber(wire, 10)
        self.assertEqual(builder.npts, 10)
        self.assertAlmostEqual(builder.spacing, 0.44444, places=5)

    def test_points_along_shape_by_distance(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        builder = PointsAlongShapeByDistance(wire, 1.)
        self.assertEqual(builder.npts, 5)
        self.assertAlmostEqual(builder.spacing, 1., places=5)

    def test_plane_by_edges(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4]).wire
        builder = PlaneByEdges(wire)
        self.assertTrue(builder.found)
        pln = builder.plane
        self.assertIsInstance(pln, Plane)


class TestTopologyBop(unittest.TestCase):
    """
    Test cases for afem.topology.bop.
    """

    def test_fuse_shapes(self):
        e1 = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        e2 = EdgeByPoints((5., 1., 0.), (5., -1., 0.)).edge
        fuse = FuseShapes(e1, e2)
        self.assertTrue(fuse.is_done)
        # Setting arguments and tools
        fuse = FuseShapes()
        fuse.set_args([e1])
        fuse.set_tools([e2])
        fuse.build()
        self.assertTrue(fuse.is_done)

    def test_cut_shapes(self):
        e1 = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        e2 = EdgeByPoints((5., 0., 0.), (6., 0., 0.)).edge
        cut = CutShapes(e1, e2)
        self.assertTrue(cut.is_done)
        # Setting arguments and tools
        cut = CutShapes()
        cut.set_args([e1])
        cut.set_tools([e2])
        cut.build()
        self.assertTrue(cut.is_done)

    def test_common_shapes(self):
        e1 = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        e2 = EdgeByPoints((5., 0., 0.), (6., 0., 0.)).edge
        common = CommonShapes(e1, e2)
        self.assertTrue(common.is_done)
        # Setting arguments and tools
        common = CommonShapes()
        common.set_args([e1])
        common.set_tools([e2])
        common.build()
        self.assertTrue(common.is_done)

    def test_intersect_shapes(self):
        e1 = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        e2 = EdgeByPoints((5., 1., 0.), (5., -1., 0.)).edge
        section = IntersectShapes(e1, e2)
        self.assertTrue(section.is_done)
        # Setting arguments and tools
        section = IntersectShapes()
        section.set_args([e1])
        section.set_tools([e2])
        section.build()
        self.assertTrue(section.is_done)

    def test_split_shapes(self):
        e1 = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        e2 = EdgeByPoints((5., 1., 0.), (5., -1., 0.)).edge
        split = SplitShapes(e1, e2)
        self.assertTrue(split.is_done)
        # Setting arguments and tools
        split = SplitShapes()
        split.set_args([e1])
        split.set_tools([e2])
        split.build()
        self.assertTrue(split.is_done)

    @unittest.expectedFailure
    def test_cut_cylindrical_hole_through_face(self):
        """
        This feature seems to have stopped working on faces as of OCCT 7.4.0.
        Will submit to OpenCASCADE.
        """
        pln = PlaneByAxes().plane
        face = FaceByPlane(pln, -2., 2., -2., 2.).face
        p = Point(0., 0., 0.)
        d = Direction(0., 1., 0.)
        ax1 = Axis1(p, d)
        cut = CutCylindricalHole(face, 1., ax1)
        self.assertTrue(cut.is_done)

    def test_cut_cylindrical_hole_through_shell(self):
        shell = BoxBySize(10., 10., 10.).shell
        p = Point(5., 5., 5.)
        d = Direction(0., 1., 0.)
        ax1 = Axis1(p, d)
        cut = CutCylindricalHole(shell, 1., ax1)
        self.assertTrue(cut.is_done)
        self.assertEqual(cut.shape.num_faces, 6)

    def test_cut_cylindrical_hole_through_solid(self):
        solid = BoxBySize(10., 10., 10.).solid
        p = Point(5., 5., 5.)
        d = Direction(0., 1., 0.)
        ax1 = Axis1(p, d)
        cut = CutCylindricalHole(solid, 1., ax1)
        self.assertTrue(cut.is_done)
        self.assertEqual(cut.shape.num_faces, 7)

    def test_local_split(self):
        pln = PlaneByAxes().plane
        builder = SolidByPlane(pln, 5., 5., 5.)
        box = builder.solid
        tool = PlaneByAxes(axes='xy').plane
        face = box.faces[0]
        split = LocalSplit(face, tool, box)
        self.assertTrue(split.is_done)

    @unittest.expectedFailure
    def test_section_fail_08292017(self):
        """
        Intersection between a planar face and an edge. Should result in a
        vertex but finds nothing. Appears to be OCCT bug. Same behavior in
        SALOME 8.2.0.
        """
        shape1 = brep.read_brep('./test_io/section_fail_08292017_shape1.brep')
        shape2 = brep.read_brep('./test_io/section_fail_08292017_shape2.brep')

        section = IntersectShapes(shape1, shape2)
        self.assertTrue(section.is_done)

        verts = section.vertices
        self.assertEqual(len(verts), 1)

    def test_section_fail_0912017(self):
        """
        Intersection between wing solid and planar face near a wing cross
        section.
        """
        shape1 = brep.read_brep('./test_io/section_fail_09122017_shape1.brep')
        shape2 = brep.read_brep('./test_io/section_fail_09122017_shape2.brep')

        section = IntersectShapes(shape1, shape2)
        self.assertTrue(section.is_done)

        shape = section.shape
        self.assertTrue(CheckShape(shape).is_valid)

        self.assertEqual(len(section.edges), 6)

        builder = WiresByShape(shape)
        self.assertEqual(builder.nwires, 1)
        wire = builder.wires[0]
        self.assertTrue(wire.closed)

    def test_fuse_fail_09142017(self):
        """
        In OCE 0.18 these shapes fail to fuse. In SALOME 8.2.0, they will fuse
        with the "Remove extra edges" set to OFF.
        """
        shape1 = brep.read_brep('./test_io/fuse_fail_09142017_shape1.brep')
        shape2 = brep.read_brep('./test_io/fuse_fail_09142017_shape2.brep')

        fuse = FuseShapes(shape1, shape2)
        self.assertTrue(fuse.is_done)

        shape = fuse.shape
        self.assertTrue(CheckShape(shape).is_valid)
        faces = shape.faces
        self.assertEqual(len(faces), 4)

        section_edges = fuse.section_edges
        self.assertEqual(len(section_edges), 1)

    def test_cut_fail_10232017(self):
        shape1 = brep.read_brep('./test_io/cut_fail_10232017_shape1.brep')
        shape2 = brep.read_brep('./test_io/cut_fail_10232017_shape2.brep')

        cut = CutShapes(shape1, shape2)
        self.assertTrue(cut.is_done)

        shape = cut.shape
        self.assertTrue(CheckShape(shape).is_valid)

    @unittest.expectedFailure
    def test_common_fail_02282018(self):
        """
        Common operation between a wing solid and a basis shell. The result
        is empty but should not be.
        """
        shape1 = brep.read_brep('./test_io/common_fail_02282018_shape1.brep')
        shape2 = brep.read_brep('./test_io/common_fail_02282018_shape2.brep')

        common = CommonShapes(shape1, shape2)
        self.assertTrue(common.is_done)

        shape = common.shape
        self.assertTrue(CheckShape(shape).is_valid)

        faces = shape.faces
        self.assertGreater(len(faces), 0)

    def test_section_fail_03012018(self):
        """
        Intersection between xz-plane and a face. The intersection should be
        a single edge, but it's sometimes two edges with a gap in between or
        an edge and a single vertex.
        """
        shape1 = brep.read_brep('./test_io/section_fail_03012018_shape1.brep')
        shape2 = brep.read_brep('./test_io/section_fail_03012018_shape2.brep')

        section = IntersectShapes(shape1, shape2)
        self.assertTrue(section.is_done)

        shape = section.shape
        self.assertTrue(CheckShape(shape).is_valid)

        self.assertEqual(len(section.edges), 1)
        self.assertEqual(len(section.vertices), 2)


class TestTopologyDistance(unittest.TestCase):
    """
    Test cases for afem.topoloy.distance.
    """

    def test_distance_shape_to_shape(self):
        v1 = VertexByPoint((0., 0., 0.)).vertex
        v2 = VertexByPoint((10., 0., 0.)).vertex
        tool = DistanceShapeToShape(v1, v2)
        self.assertEqual(tool.nsol, 1)
        self.assertAlmostEqual(tool.dmin, 10.)

    def test_distance_shape_to_shapes(self):
        v1 = VertexByPoint((0., 0., 0.)).vertex
        v2 = VertexByPoint((5., 0., 0.)).vertex
        v3 = VertexByPoint((10., 0., 0.)).vertex
        tool = DistanceShapeToShapes(v1, [v3, v2])
        self.assertAlmostEqual(tool.dmin, 5.)
        self.assertAlmostEqual(tool.dmax, 10.)
        self.assertAlmostEqual(tool.sorted_distances[0], 5.)
        self.assertAlmostEqual(tool.sorted_distances[1], 10.)


class TestTopologyExplore(unittest.TestCase):
    """
    Test cases for afem.topoloy.explore.
    """

    def test_explore_wire(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        wire = WireByPoints([p1, p2, p3, p4], True).wire
        explorer = ExploreWire(wire)
        self.assertEqual(explorer.nedges, 4)


class TestTopologyModify(unittest.TestCase):
    """
    Test cases for afem.topoloy.modify.
    """

    def test_sew_shape(self):
        p1 = (0., 0., 0.)
        p2 = (1., 0., 0.)
        p3 = (1., 1., 0.)
        p4 = (0., 1., 0.)
        w1 = WireByPoints([p1, p2, p3, p4], True).wire
        f1 = FaceByPlanarWire(w1).face
        p1 = (0., 0., 0.)
        p2 = (0., 0., 1.)
        p3 = (0., 1., 1.)
        p4 = (0., 1., 0.)
        w2 = WireByPoints([p1, p2, p3, p4], True).wire
        f2 = FaceByPlanarWire(w2).face
        p1 = (0., 0.5, 0.)
        p2 = (-1., 0.5, 0.)
        p3 = (-1., 1.5, 0.)
        p4 = (0., 1.5, 0.)
        w3 = WireByPoints([p1, p2, p3, p4], True).wire
        f3 = FaceByPlanarWire(w3).face
        # Build an unconnected shell
        shell = ShellByFaces([f1, f2, f3]).shell
        tool = SewShape(shell, non_manifold=True)
        shape = tool.sewed_shape
        self.assertEqual(len(shape.faces), 3)

    def test_rebuild_shape_by_tool(self):
        pln1 = PlaneByAxes(axes='xy').plane
        box1 = SolidByPlane(pln1, 10., 10., 10.).solid
        pln2 = PlaneByAxes((1., 1., 1.), 'xy').plane
        box2 = SolidByPlane(pln2, 5., 15., 5.).solid
        cut = CutShapes(box1, box2)
        self.assertTrue(cut.is_done)
        rebuild = RebuildShapeByTool(box1, cut)
        new_shape = rebuild.new_shape
        self.assertTrue(box1.is_solid)
        shape = FixShape(new_shape).shape
        self.assertTrue(shape.is_shell)

    def test_unify_shape(self):
        builder = BoxBySize(10, 10, 10)
        box = builder.solid
        pln = PlaneByAxes((5, 5, 5), 'xz').plane
        shape = LocalSplit(builder.front_face, pln, box).shape
        self.assertEqual(shape.num_faces, 7)
        unify = UnifyShape(shape)
        self.assertEqual(unify.shape.num_faces, 6)


class TestTopologyOffset(unittest.TestCase):
    """
    Test cases for afem.topology.offset.
    """

    def test_project_shape(self):
        pln = PlaneByAxes().plane
        face = FaceByPlane(pln, -5., 5., -5., 5.).face
        edge = EdgeByPoints((0., 1., 15.), (0., 1., -15.)).edge
        proj = ProjectShape(face, [edge])
        self.assertTrue(proj.is_done)
        self.assertEqual(proj.nedges, 1)

    def test_loft_shape(self):
        pnts1 = [(0., 0., 0.), (5., 0., 5.), (10., 0., 0.)]
        wire1 = WireByPoints(pnts1).wire
        pnts2 = [(0., 10., 0.), (5., 10., -5.), (10., 10., 0.)]
        wire2 = WireByPoints(pnts2).wire
        loft = LoftShape([wire1, wire2])
        self.assertTrue(loft.is_done)


class TestTopologyProps(unittest.TestCase):
    """
    Test cases for afem.topology.props.
    """

    def test_linear_props(self):
        e = EdgeByPoints((0., 0., 0.), (1., 0., 0.)).edge
        prop = LinearProps(e)
        self.assertAlmostEqual(prop.length, 1.)
        p = prop.cg
        self.assertAlmostEqual(p.x, 0.5)
        self.assertAlmostEqual(p.y, 0.)
        self.assertAlmostEqual(p.z, 0.)

    def test_surface_props(self):
        e = EdgeByPoints((0., 0., 0.), (1., 0., 0.)).edge
        f = FaceByDrag(e, (0., 1., 0.)).face
        prop = SurfaceProps(f)
        self.assertAlmostEqual(prop.area, 1.)
        p = prop.cg
        self.assertAlmostEqual(p.x, 0.5)
        self.assertAlmostEqual(p.y, 0.5)
        self.assertAlmostEqual(p.z, 0.)

    def test_volume_props(self):
        e = EdgeByPoints((0., 0., 0.), (1., 0., 0.)).edge
        f = FaceByDrag(e, (0., 1., 0.)).face
        solid = SolidByDrag(f, (0., 0., 1.)).solid
        prop = VolumeProps(solid)
        self.assertAlmostEqual(prop.volume, 1.)
        p = prop.cg
        self.assertAlmostEqual(p.x, 0.5)
        self.assertAlmostEqual(p.y, 0.5)
        self.assertAlmostEqual(p.z, 0.5)


class TestTopologyTransform(unittest.TestCase):
    """
    Test cases for afem.topology.transform.
    """

    def test_translate(self):
        box = BoxBySize().solid
        vec = VectorByXYZ().vector

        new_box = afem.topology.transform.translate_shape(box, vec)

        prop = VolumeProps(new_box)
        self.assertAlmostEqual(1.5, prop.cg.x)


if __name__ == '__main__':
    unittest.main()
