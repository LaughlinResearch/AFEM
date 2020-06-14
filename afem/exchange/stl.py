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
from OCCT.StlAPI import StlAPI_Writer

__all__ = ["StlWrite"]


class StlWrite(StlAPI_Writer):
    """
    Export shape to STL file.
    """

    def __init__(self):
        super(StlWrite, self).__init__()

    def write(self, shape, fn):
        """
        Converts shape to STL format and writes to a file.
        
        :param afem.topology.entities.Shape shape: The shape.
        :param str fn: The filename.
         
        :return: None.
        """
        self.Write(shape.object, fn)
