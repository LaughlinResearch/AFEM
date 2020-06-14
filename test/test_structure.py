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

from afem.exchange import brep
from afem.geometry import *
from afem.oml import *
from afem.structure import *
from afem.topology import *


class TestStructureEntities(unittest.TestCase):
    """
    Test cases for afem.structure.entities.
    """

    @classmethod
    def setUpClass(cls):
        shape = brep.read_brep('./test_io/rhs_wing.brep')
        cls.wing = Body(shape, 'wing')
        face = brep.read_brep('./test_io/rhs_wing_sref.brep')
        sref = face.surface
        cls.wing.set_sref(sref)
        shape = brep.read_brep('./test_io/fuselage.brep')
        cls.fuselage = Body(shape, 'fuselage')

        cls.fspar = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5,
                                     cls.wing).part
        cls.rspar = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5,
                                     cls.wing).part
        cls.part1 = RibByPoints('rib1', cls.fspar.cref.p1, cls.rspar.cref.p1,
                                cls.wing).part
        cls.part2 = RibByPoints('rib2', cls.fspar.cref.p2, cls.rspar.cref.p2,
                                cls.wing).part

        cls.null_rib = RibByParameters('null rib', 0.15, 0.5, 0.65, 0.5,
                                       cls.wing).part
        cls.null_rib.shape.nullify()

    @classmethod
    def tearDownClass(cls):
        GroupAPI.reset()

    def test_part_name(self):
        self.assertEqual('fspar', self.fspar.name)

    def test_part_id(self):
        self.assertEqual(1, self.fspar.id)
        self.assertEqual(2, self.rspar.id)
        self.assertEqual(3, self.part1.id)
        self.assertEqual(4, self.part2.id)

    def test_part_shape(self):
        self.assertIsInstance(self.fspar.shape, Compound)
        p = self.wing.sref.eval(0.5, 0.5)
        pln = PlaneByAxes(p, 'xz').plane
        f = FaceByPlane(pln, -100, 100, -100, 100).face
        self.assertIsNone(self.null_rib.set_shape(f))

    def test_part_is_null(self):
        self.assertFalse(self.fspar.shape.is_null)
        self.assertTrue(self.null_rib.shape.is_null)

    def test_part_tol(self):
        self.assertAlmostEqual(self.fspar.shape.tol_avg, 1.831889856924312e-5,
                               places=7)
        self.assertAlmostEqual(self.fspar.shape.tol_max, 7.575160986669027e-5,
                               places=7)

    def test_part_cref(self):
        self.assertIsInstance(self.fspar.cref, TrimmedCurve)

    def test_part_sref(self):
        self.assertIsInstance(self.fspar.sref, Plane)

    def test_part_plane(self):
        self.assertIsInstance(self.fspar.sref, Plane)

    def test_part_p1(self):
        p1 = self.fspar.cref.p1
        self.assertAlmostEqual(p1.x, 969.324, places=3)
        self.assertAlmostEqual(p1.y, 196.839, places=3)
        self.assertAlmostEqual(p1.z, -59.085, places=3)

    def test_part_p2(self):
        p2 = self.fspar.cref.p2
        self.assertAlmostEqual(p2.x, 1251.391, places=3)
        self.assertAlmostEqual(p2.y, 652.999, places=3)
        self.assertAlmostEqual(p2.z, 6.559, places=3)

    def test_part_edges(self):
        self.assertEqual(self.fspar.shape.num_edges, 9)
        for e in self.fspar.shape.edges:
            self.assertIsInstance(e, Edge)
        self.assertIsInstance(self.fspar.edge_compound, Compound)

    def test_part_faces(self):
        self.assertEqual(self.fspar.shape.num_faces, 1)
        for f in self.fspar.shape.faces:
            self.assertIsInstance(f, Face)
        self.assertIsInstance(self.fspar.face_compound, Compound)


class TestStructureCreate(unittest.TestCase):
    """
    Test cases for afem.structure.create.
    """

    @classmethod
    def setUpClass(cls):
        shape = brep.read_brep('./test_io/rhs_wing.brep')
        cls.wing = Body(shape, 'wing')
        face = brep.read_brep('./test_io/rhs_wing_sref.brep')
        sref = face.surface
        cls.wing.set_sref(sref)
        shape = brep.read_brep('./test_io/fuselage.brep')
        cls.fuselage = Body(shape, 'fuselage')

    def tearDown(self):
        GroupAPI.reset()

    def test_curve_part_by_shape(self):
        e = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        builder = CurvePartByShape('part', e)
        part = builder.part
        self.assertIsInstance(part, CurvePart)

    def test_beam_by_shape(self):
        e = EdgeByPoints((0., 0., 0.), (10., 0., 0.)).edge
        builder = Beam1DByShape('beam', e)
        part = builder.part
        self.assertIsInstance(part, Beam1D)

    def test_beam_by_curve(self):
        c = NurbsCurveByPoints([(0., 0., 0.), (10., 0., 0.)]).curve
        builder = Beam1DByCurve('beam', c)
        part = builder.part
        self.assertIsInstance(part, Beam1D)

    def test_beam_by_points(self):
        builder = Beam1DByPoints('part', (0., 0., 0.), (10., 0., 0.))
        part = builder.part
        self.assertIsInstance(part, Beam1D)

    def test_spar_by_parameters(self):
        builder = SparByParameters('spar', 0.5, 0., 0.5, 0.75, self.wing)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spar_by_points(self):
        p1 = self.wing.sref.eval(0.5, 0.)
        p2 = self.wing.sref.eval(0.5, 0.75)
        builder = SparByPoints('spar', p1, p2, self.wing)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spar_by_ends(self):
        p1 = (0.5, 0.)
        p2 = self.wing.sref.eval(0.5, 0.75)
        builder = SparByEnds('spar', p1, p2, self.wing)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spar_by_surface(self):
        p0 = self.wing.sref.eval(0.5, 0.)
        pln = PlaneByAxes(p0, 'yz').plane
        builder = SparByShape('spar', pln, self.wing)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spar_by_shape(self):
        p0 = self.wing.sref.eval(0.5, 0.)
        pln = PlaneByAxes(p0, 'yz').plane
        f = FaceBySurface(pln).face
        builder = SparByShape('spar', f, self.wing)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spar_between_shapes(self):
        p = self.wing.sref.eval(0.5, 0.)
        pln1 = PlaneByAxes(p, 'xz').plane
        shape1 = FaceBySurface(pln1).face
        p = self.wing.sref.eval(0.5, 0.5)
        pln2 = PlaneByAxes(p, 'xz').plane
        shape2 = FaceBySurface(pln2).face
        p = self.wing.sref.eval(0.5, 0.)
        pln3 = PlaneByNormal(p, (0.5, -0.2, 0.)).plane
        basis_shape = FaceBySurface(pln3).face
        builder = SparBetweenShapes('spar', shape1, shape2, self.wing,
                                    basis_shape)
        spar = builder.part
        self.assertIsInstance(spar, Spar)

    def test_spars_between_planes_by_number(self):
        builder = RibByParameters('rib1', 0.15, 0.15, 0.65, 0.15, self.wing)
        rib1 = builder.part
        builder = RibByParameters('rib2', 0.15, 0.25, 0.65, 0.25, self.wing)
        rib2 = builder.part
        pln1 = PlaneByAxes(rib2.cref.p1, 'yz').plane
        pln2 = PlaneByAxes(rib2.cref.p2, 'yz').plane
        builder = SparsBetweenPlanesByNumber('spar', pln1, pln2, 5, rib1,
                                             rib2, self.wing)
        self.assertEqual(builder.nparts, 5)
        self.assertEqual(builder.next_index, 6)
        for spar in builder.parts:
            self.assertIsInstance(spar, Spar)

    def test_spars_between_planes_by_distance(self):
        builder = RibByParameters('rib1', 0.15, 0.15, 0.65, 0.15, self.wing)
        rib1 = builder.part
        builder = RibByParameters('rib2', 0.15, 0.25, 0.65, 0.25, self.wing)
        rib2 = builder.part
        pln1 = PlaneByAxes(rib2.cref.p1, 'yz').plane
        pln2 = PlaneByAxes(rib2.cref.p2, 'yz').plane
        builder = SparsBetweenPlanesByDistance('spar', pln1, pln2, 36., rib1,
                                               rib2, self.wing)
        self.assertEqual(builder.nparts, 5)
        self.assertEqual(builder.next_index, 6)
        self.assertAlmostEqual(builder.spacing, 32.203, delta=0.001)
        for spar in builder.parts:
            self.assertIsInstance(spar, Spar)

    def test_spars_along_curve_by_number(self):
        builder = RibByParameters('rib1', 0.15, 0.15, 0.65, 0.15, self.wing)
        rib1 = builder.part
        builder = RibByParameters('rib2', 0.15, 0.25, 0.65, 0.25, self.wing)
        rib2 = builder.part
        builder = SparsAlongCurveByNumber('spar', rib2.cref, 5, rib1, rib2,
                                          self.wing)
        self.assertEqual(builder.nparts, 5)
        self.assertEqual(builder.next_index, 6)
        for spar in builder.parts:
            self.assertIsInstance(spar, Spar)

    def test_spars_along_curve_by_distance(self):
        builder = RibByParameters('rib1', 0.15, 0.15, 0.65, 0.15, self.wing)
        rib1 = builder.part
        builder = RibByParameters('rib2', 0.15, 0.25, 0.65, 0.25, self.wing)
        rib2 = builder.part
        builder = SparsAlongCurveByDistance('spar', rib2.cref, 36., rib1, rib2,
                                            self.wing)
        self.assertEqual(builder.nparts, 7)
        self.assertEqual(builder.next_index, 8)
        self.assertAlmostEqual(builder.spacing, 32.247, delta=0.001)
        for spar in builder.parts:
            self.assertIsInstance(spar, Spar)

    def test_rib_by_parameters(self):
        builder = RibByParameters('rib', 0.15, 0.15, 0.65, 0.15, self.wing)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def test_rib_by_points(self):
        p1 = self.wing.sref.eval(0.15, 0.15)
        p2 = self.wing.sref.eval(0.65, 0.15)
        builder = RibByPoints('rib', p1, p2, self.wing)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def test_rib_by_surface(self):
        p0 = self.wing.sref.eval(0.5, 0.5)
        pln = PlaneByAxes(p0, 'xz').plane
        builder = RibByShape('rib', pln, self.wing)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def test_rib_by_shape(self):
        p0 = self.wing.sref.eval(0.5, 0.25)
        pln = PlaneByAxes(p0, 'xz').plane
        f = FaceBySurface(pln).face
        builder = RibByShape('rib', f, self.wing)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def test_rib_between_shapes(self):
        p = self.wing.sref.eval(0.15, 0.5)
        pln1 = PlaneByAxes(p, 'yz').plane
        shape1 = FaceBySurface(pln1).face
        p = self.wing.sref.eval(0.65, 0.5)
        pln2 = PlaneByAxes(p, 'yz').plane
        shape2 = FaceBySurface(pln2).face
        p = self.wing.sref.eval(0.5, 0.5)
        pln3 = PlaneByAxes(p, 'xz').plane
        basis_shape = FaceBySurface(pln3).face
        builder = RibBetweenShapes('rib', shape1, shape2, self.wing,
                                   basis_shape)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def rib_by_orientation(self):
        p = self.wing.sref.eval(0.15, 0.15)
        builder = RibByOrientation('rib', p, self.wing, gamma=15.)
        rib = builder.part
        self.assertIsInstance(rib, Rib)

    def test_ribs_between_planes_by_number(self):
        builder = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5, self.wing)
        fspar = builder.part
        builder = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5, self.wing)
        rspar = builder.part
        pln1 = PlaneByAxes(fspar.cref.p1, 'xz').plane
        pln2 = PlaneByAxes(fspar.cref.p2, 'xz').plane
        builder = RibsBetweenPlanesByNumber('rib', pln1, pln2, 5, fspar,
                                            rspar, self.wing)
        self.assertEqual(builder.nparts, 5)
        self.assertEqual(builder.next_index, 6)
        for rib in builder.parts:
            self.assertIsInstance(rib, Rib)

    def test_ribs_between_planes_by_distance(self):
        builder = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5, self.wing)
        fspar = builder.part
        builder = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5, self.wing)
        rspar = builder.part
        pln1 = PlaneByAxes(fspar.cref.p1, 'xz').plane
        pln2 = PlaneByAxes(fspar.cref.p2, 'xz').plane
        builder = RibsBetweenPlanesByDistance('rib', pln1, pln2, 36., fspar,
                                              rspar, self.wing)
        self.assertEqual(builder.nparts, 12)
        self.assertAlmostEqual(builder.spacing, 35.089, delta=0.001)
        self.assertEqual(builder.next_index, 13)
        for rib in builder.parts:
            self.assertIsInstance(rib, Rib)

    def test_ribs_along_curve_by_number(self):
        builder = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5, self.wing)
        fspar = builder.part
        builder = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5, self.wing)
        rspar = builder.part
        root = RibByPoints('root', fspar.cref.p1, rspar.cref.p1,
                           self.wing).part
        tip = RibByPoints('tip', fspar.cref.p2, rspar.cref.p2, self.wing).part
        p1 = root.cref.eval(0.5 * (root.cref.u1 + root.cref.u2))
        p2 = tip.cref.eval(0.5 * (tip.cref.u1 + tip.cref.u2))
        curve = NurbsCurveByPoints([p1, p2]).curve
        builder = RibsAlongCurveByNumber('rib', curve, 5, fspar, rspar,
                                         self.wing)
        self.assertEqual(builder.nparts, 5)
        self.assertEqual(builder.next_index, 6)
        for rib in builder.parts:
            self.assertIsInstance(rib, Rib)

    def test_ribs_along_curve_by_distance(self):
        builder = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5, self.wing)
        fspar = builder.part
        builder = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5, self.wing)
        rspar = builder.part
        root = RibByPoints('root', fspar.cref.p1, rspar.cref.p1,
                           self.wing).part
        tip = RibByPoints('tip', fspar.cref.p2, rspar.cref.p2, self.wing).part
        p1 = root.cref.eval(0.5 * (root.cref.u1 + root.cref.u2))
        p2 = tip.cref.eval(0.5 * (tip.cref.u1 + tip.cref.u2))
        curve = NurbsCurveByPoints([p1, p2]).curve
        builder = RibsAlongCurveByDistance('rib', curve, 36., fspar, rspar,
                                           self.wing)
        self.assertEqual(builder.nparts, 16)
        self.assertEqual(builder.next_index, 17)
        self.assertAlmostEqual(builder.spacing, 34.216, delta=0.001)
        for rib in builder.parts:
            self.assertIsInstance(rib, Rib)

    def test_ribs_along_curve_and_surface_by_distance(self):
        builder = SparByParameters('fspar', 0.15, 0.15, 0.15, 0.5, self.wing)
        fspar = builder.part
        builder = SparByParameters('rspar', 0.65, 0.15, 0.65, 0.5, self.wing)
        rspar = builder.part
        builder = RibsAlongCurveAndSurfaceByDistance('rib', rspar.cref,
                                                     self.wing.sref, 36.,
                                                     fspar,
                                                     rspar, self.wing)
        self.assertEqual(builder.nparts, 15)
        self.assertEqual(builder.next_index, 16)
        self.assertAlmostEqual(builder.spacing, 35.107, delta=0.001)
        for rib in builder.parts:
            self.assertIsInstance(rib, Rib)

    def test_bulkhead_by_surface(self):
        pln = PlaneByAxes((600., 0., 0.), 'yz').plane
        builder = BulkheadByShape('bulkhead', pln, self.fuselage)
        bh = builder.part
        self.assertIsInstance(bh, Bulkhead)

    def test_floor_by_surface(self):
        pln = PlaneByAxes((0., 0., 0.), 'xy').plane
        builder = FloorByShape('floor', pln, self.fuselage)
        floor = builder.part
        self.assertIsInstance(floor, Floor)

    def test_frame_by_plane(self):
        pln = PlaneByAxes((600., 0., 0.), 'yz').plane
        builder = FrameByPlane('frame', pln, self.fuselage, 3.)
        frame = builder.part
        self.assertIsInstance(frame, Frame)

    def test_frames_by_planes(self):
        pln1 = PlaneByAxes((600., 0., 0.), 'yz').plane
        pln2 = PlaneByAxes((605., 0., 0.), 'yz').plane
        pln3 = PlaneByAxes((610., 0., 0.), 'yz').plane
        builder = FramesByPlanes('frame', [pln1, pln2, pln3], self.fuselage,
                                 3.)
        self.assertEqual(builder.nparts, 3)
        self.assertEqual(builder.next_index, 4)
        for frame in builder.parts:
            self.assertIsInstance(frame, Frame)

    def test_frames_between_planes_by_number(self):
        pln1 = PlaneByAxes((600., 0., 0.), 'yz').plane
        pln2 = PlaneByAxes((800., 0., 0.), 'yz').plane
        builder = FramesBetweenPlanesByNumber('frame', pln1, pln2, 10,
                                              self.fuselage, 3.)
        self.assertEqual(builder.nparts, 10)
        self.assertEqual(builder.next_index, 11)
        self.assertAlmostEqual(builder.spacing, 18.182, delta=0.001)

    def test_frames_between_planes_by_distance(self):
        pln1 = PlaneByAxes((600., 0., 0.), 'yz').plane
        pln2 = PlaneByAxes((800., 0., 0.), 'yz').plane
        builder = FramesBetweenPlanesByDistance('frame', pln1, pln2, 24.,
                                                self.fuselage, 3.)
        self.assertEqual(builder.nparts, 8)
        self.assertEqual(builder.next_index, 9)
        self.assertAlmostEqual(builder.spacing, 22.222, delta=0.001)

    def test_skin_by_solid(self):
        skin = SkinBySolid('skin', self.wing.shape).part
        self.assertIsInstance(skin, Skin)

    def test_skin_by_body(self):
        skin = SkinByBody('skin', self.fuselage).part
        self.assertIsInstance(skin, Skin)


if __name__ == '__main__':
    unittest.main()
