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
from OCCT.TColStd import (TColStd_Array1OfInteger, TColStd_Array1OfReal,
                          TColStd_Array2OfReal, TColStd_HSequenceOfReal)
from OCCT.TColgp import (TColgp_Array1OfPnt, TColgp_Array1OfPnt2d,
                         TColgp_Array2OfPnt, TColgp_HArray1OfPnt,
                         TColgp_HArray1OfPnt2d)
from OCCT.TopoDS import TopoDS_ListOfShape
from OCCT.gp import gp_Pnt, gp_Pnt2d
from numpy import array as np_array, zeros

from afem.misc.utils import is_array_like


def to_gp_pnt(p):
    """
    Convert the point_like entity to a gp_Pnt.
    """
    if isinstance(p, gp_Pnt):
        return p
    if is_array_like(p) and len(p) == 3:
        return gp_Pnt(*p)
    return None


def to_gp_pnt2d(p):
    """
    Convert the point_like entity to a gp_Pnt2d.
    """
    if isinstance(p, gp_Pnt2d):
        return p
    if is_array_like(p) and len(p) == 2:
        return gp_Pnt2d(*p)
    return None


def to_tcolgp_array1_pnt(pnts):
    """
    Convert the 1-D array of point_like entities to OCC data.

    :param array_like pnts: Array of points to convert.

    :return: OCC array of points.
    :rtype: TColgp_Array1OfPnt
    """
    gp_pnts = []
    for gp in pnts:
        gp = to_gp_pnt(gp)
        if not gp:
            continue
        gp_pnts.append(gp)

    n = len(gp_pnts)
    array = TColgp_Array1OfPnt(1, n)
    for i, gp in enumerate(gp_pnts, 1):
        array.SetValue(i, gp)

    return array


def to_tcolgp_array1_pnt2d(pnts):
    """
    Convert the 1-D array of point_like entities to OCC data.

    :param array_like pnts: Array of points to convert.

    :return: OCC array of points.
    :rtype: TColgp_Array1OfPnt2d
    """
    gp_pnts = []
    for gp in pnts:
        gp = to_gp_pnt2d(gp)
        if not gp:
            continue
        gp_pnts.append(gp)

    n = len(gp_pnts)
    array = TColgp_Array1OfPnt2d(1, n)
    for i, gp in enumerate(gp_pnts, 1):
        array.SetValue(i, gp)

    return array


def to_tcolgp_harray1_pnt(pnts):
    """
    Convert the 1-D array of point_like entities to OCC data.

    :param array_like pnts: Array of points to convert.

    :return: OCC array of points.
    :rtype: TColgp_HArray1OfPnt
    """
    gp_pnts = []
    for gp in pnts:
        gp = to_gp_pnt(gp)
        if not gp:
            continue
        gp_pnts.append(gp)

    n = len(gp_pnts)
    harray = TColgp_HArray1OfPnt(1, n)
    for i, gp in enumerate(gp_pnts, 1):
        harray.SetValue(i, gp)

    return harray


def to_tcolgp_harray1_pnt2d(pnts):
    """
    Convert the 1-D array of point2d_like entities to OCC data.

    :param point2d_like pnts: Array of 2-D points to convert.

    :return: OCC array of points.
    :rtype: TColgp_HArray1OfPnt2d
    """
    gp_pnts = []
    for gp in pnts:
        gp = to_gp_pnt2d(gp)
        if not gp:
            continue
        gp_pnts.append(gp)

    n = len(gp_pnts)
    harray = TColgp_HArray1OfPnt2d(1, n)
    for i, gp in enumerate(gp_pnts, 1):
        harray.SetValue(i, gp)

    return harray


def to_tcolstd_array1_real(array):
    """
    Convert the 1-D array of floats to OCC data.

    :param array_like array: Array of floats.

    :return: OCC array of floats.
    :rtype: TColStd_Array1OfReal
    """
    flts = [float(x) for x in array]
    n = len(flts)
    array = TColStd_Array1OfReal(1, n)
    for i, x in enumerate(flts, 1):
        array.SetValue(i, x)

    return array


def to_tcolstd_array1_integer(array):
    """
    Convert the 1-D array of integers to OCC data.

    :param array_like array: Array of integers.

    :return: OCC array of integers.
    :rtype: TColStd_Array1OfInteger
    """
    ints = [int(x) for x in array]
    n = len(ints)
    array = TColStd_Array1OfInteger(1, n)
    for i, x in enumerate(ints, 1):
        array.SetValue(i, x)

    return array


def to_tcolgp_array2_pnt(pnts):
    """
    Convert the 2-D array of point_like entities to OCC data.

    :param array_like pnts: Array of points to convert.

    :return: OCC array of points.
    :rtype: TColgp_Array2OfPnt
    """
    pnts = np_array(pnts, dtype=float)
    n, m = pnts.shape[0:2]
    gp_pnts = []
    for i in range(0, n):
        row = []
        for j in range(0, m):
            gp = to_gp_pnt(pnts[i, j])
            row.append(gp)
        gp_pnts.append(row)

    array = TColgp_Array2OfPnt(1, n, 1, m)
    for i, row in enumerate(gp_pnts, 1):
        for j, gp in enumerate(row, 1):
            array.SetValue(i, j, gp)

    return array


def to_tcolstd_hseq_real(array):
    """
    Convert the sequence of floats to OCC data.

    :param array_like array: The array.

    :return: The OCC data.
    :rtype: OCCT.TColStd.TColStd_HSequenceOfReal
    """
    hseq = TColStd_HSequenceOfReal()
    for x in array:
        hseq.Append(x)
    return hseq


def to_tcolstd_array2_real(array):
    """
    Convert the 2-D array of floats to OCC data.

    :param array_like array: Array of floats.

    :return: OCC array of floats.
    :rtype: TColStd_Array2OfReal
    """
    flts = np_array(array, dtype=float)
    n, m = flts.shape
    array = TColStd_Array2OfReal(1, n, 1, m)
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            x = float(flts[i - 1, j - 1])
            array.SetValue(i, j, x)

    return array


def to_np_from_tcolstd_array1_real(tcol_array):
    """
    Convert OCC data to NumPy array.

    :param tcol_array: OCC array of floats.
    :type tcol_array: TColStd_Array1OfReal

    :return: NumPy array of floats.
    :rtype: ndarray
    """
    n = tcol_array.Length()
    array = zeros(n, dtype=float)
    for i in range(n):
        x = tcol_array.Value(i + 1)
        array[i] = x
    return array


def to_np_from_tcolstd_array1_integer(tcol_array):
    """
    Convert OCC data to NumPy array.

    :param tcol_array: OCC array of integers.
    :type tcol_array: TColStd_Array1OfInteger

    :return: NumPy array of integers.
    :rtype: ndarray
    """
    n = tcol_array.Length()
    array = zeros(n, dtype=int)
    for i in range(n):
        x = tcol_array.Value(i + 1)
        array[i] = x
    return array


def to_np_from_tcolgp_array1_pnt(tcol_array):
    """
    Convert OCC data to NumPy array.

    :param tcol_array: OCC array of points.
    :type tcol_array: TColgp_Array1OfPnt

    :return: NumPy array of points.
    :rtype: ndarray
    """
    n = tcol_array.Length()
    array = zeros((n, 3), dtype=float)
    for i in range(n):
        p = tcol_array.Value(i + 1)
        array[i, :] = p.X(), p.Y(), p.Z()
    return array


def to_np_from_tcolgp_array2_pnt(tcol_array):
    """
    Convert OCC data to NumPy array.

    :param tcol_array: OCC array of points.
    :type tcol_array: TColgp_Array2OfPnt

    :return: NumPy array of points.
    :rtype: ndarray
    """
    n, m = tcol_array.ColLength(), tcol_array.RowLength()
    array = zeros((n, m, 3), dtype=float)
    for i in range(n):
        for j in range(m):
            p = tcol_array.Value(i + 1, j + 1)
            array[i, j, :] = p.X(), p.Y(), p.Z()
    return array


def to_np_from_tcolstd_array2_real(tcol_array):
    """
    Convert OCC data to NumPy array.

    :param tcol_array: OCC array of floats.
    :type tcol_array: TColStd_Array2OfReal

    :return: NumPy array of floats.
    :rtype: ndarray
    """
    n, m = tcol_array.ColLength(), tcol_array.RowLength()
    array = zeros((n, m), dtype=float)
    for i in range(n):
        for j in range(m):
            x = tcol_array.Value(i + 1, j + 1)
            array[i, j] = x
    return array


def to_topods_list(shapes):
    """
    Create TopoDS_ListOfShape from shapes.

    :param list(afem.topology.entities.Shape) shapes: The list.

    :return: The list.
    :rtype: OCCT.TopoDS.TopoDS_ListOfShape
    """
    topods_list = TopoDS_ListOfShape()
    for s in shapes:
        topods_list.Append(s.object)
    return topods_list
