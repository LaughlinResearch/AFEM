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

from afem.config import Settings
from afem.exchange import ImportVSP
from afem.graphics import Viewer

Settings.log_to_console()


def show_shapes(*shapes):
    gui = Viewer()
    gui.add(*shapes)
    gui.start()


class TestImportVSP(unittest.TestCase):
    """
    Test cases for afem.exchange.vsp.
    """

    def test_import_777_200LR(self):
        fn = './test_io/777-200LR.stp'
        vsp_import = ImportVSP(fn)
        self.assertFalse(vsp_import.has_invalid)
        self.assertEqual(vsp_import.num_bodies, 7)

    def test_import_thick_TE1(self):
        """
        Import and prepare a multi-section wing with a thick trailing edge
        along the entire span.
        """
        fn = './test_io/vsp_wing_thick_TE1_v3.5.0.stp'
        vsp_import = ImportVSP(fn)
        self.assertFalse(vsp_import.has_invalid)
        self.assertEqual(vsp_import.num_bodies, 1)

    def test_import_thick_TE2(self):
        """
        Import and prepare a multi-section wing with a thick trailing edge
        along where the middle airfoil has no thick trailing edge.
        """
        fn = './test_io/vsp_wing_thick_TE2_v3.5.0.stp'
        vsp_import = ImportVSP(fn)
        self.assertFalse(vsp_import.has_invalid)
        self.assertEqual(vsp_import.num_bodies, 1)


if __name__ == '__main__':
    unittest.main()
