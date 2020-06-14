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
import json

from OCCT.BRepBuilderAPI import (BRepBuilderAPI_MakeFace,
                                 BRepBuilderAPI_MakeWire)
from OCCT.GCPnts import GCPnts_QuasiUniformDeflection
from OCCT.ShapeFix import ShapeFix_Wire
from OCCT.ShapeUpgrade import ShapeUpgrade_SplitSurface

from afem.adaptor.entities import AdaptorCurve
from afem.config import logger
from afem.exchange.step import StepRead
from afem.exchange.xde import XdeDocument
from afem.geometry import utils as geom_utils
from afem.geometry.create import (PointFromParameter, NurbsSurfaceByInterp,
                                  NurbsCurveByPoints, NurbsCurveByApprox)
from afem.geometry.entities import Geometry
from afem.occ import utils as occ_utils
from afem.oml.entities import Body
from afem.topology.check import CheckShape
from afem.topology.entities import Compound, Face, Wire, Solid, Shell, Edge
from afem.topology.fix import FixShape
from afem.topology.modify import (DivideClosedShape, SewShape, UnifyShape,
                                  ShapeBSplineRestriction)
from afem.topology.props import SurfaceProps, LinearProps

__all__ = ["ImportVSP"]


class ImportVSP(object):
    """
    Tool for importing, pre-processing, and translating OpenVSP models.

    :param str fn: The full path to a file to import. If provided, the file
        extension will be used to guess which import method to call. For
        example, if the file ended with 'stp' or 'step', then the
        *import_step()* method is called during initialization.
    :param bool divide_closed: Option to divide closed faces.
    :param bool bspline_restrict: Option to attempt to refit the underlying
        OpenVSP surfaces with surfaces of higher continuity. This may take a
        long time to run and no guarantee it will be successful. At the moment
        this is only used on known wing components. This method is
        experimental.
    :param bool reloft: For wings that are not split, this option will
        extract isocurves at each spanwise cross section, tessellate the
        curve using *tol*, and then approximate the section using a C1
        continuous curve. These curves are then used to generate a new wing
        surface. This method is experimental.
    :param float tol: Tolerance for approximation if *bspline_restrict* or
        *reloft* is *True*.

    :raise TypeError: If a file is provided but the extension is not recognized
        or supported.
    """

    def __init__(self, fn=None, divide_closed=True, bspline_restrict=False,
                 reloft=False, tol=0.01):
        self._bodies = {}
        self._divide = divide_closed
        self._restrict = bspline_restrict
        self._reloft = reloft
        self._tol = tol
        self._invalid = []

        if fn is not None:
            if fn.endswith('.step') or fn.endswith('.stp'):
                self.import_step(fn)
            else:
                raise TypeError('File extension not supported.')

    def __getitem__(self, key):
        return self.get_body(key)

    @property
    def bodies(self):
        """
        :return: The dictionary of Body instances where the key is the
            component name and the value is the Body.
        :rtype: dict
        """
        return self._bodies

    @property
    def all_bodies(self):
        """
        :return: List of Body instances.
        :rtype: list(afem.oml.entities.Body)
        """
        return list(self._bodies.values())

    @property
    def num_bodies(self):
        """
        :return: The number of bodies.
        :rtype: int
        """
        return len(self.all_bodies)

    @property
    def has_invalid(self):
        """
        :return: Check if any invalid shapes were found during import.
        :rtype: bool
        """
        return len(self._invalid) > 0

    @property
    def invalid_shapes(self):
        """
        :return: List of invalid shapes found during import.
        :rtype: list(afem.topology.entities.Shape)
        """
        return self._invalid

    def clear(self):
        """
        Clear any translated data.

        :return: None.
        """
        self._bodies.clear()

    def get_body(self, name):
        """
        Return Body by name.

        :param str name: Body name.

        :return: OML body.
        :rtype: afem.oml.entities.Body

        :raise KeyError: If *name* is not present.
        """
        return self._bodies[name]

    def get_bodies(self):
        """
        Get a copy of the underlying dictionary storing the Body instances.

        :return: Dictionary where the key is the component name and the value
            is the Body.
        :rtype: dict
        """
        return self._bodies.copy()

    def import_step(self, fn):
        """
        Import a STEP file generated by the OpenVSP version that has been
        modified to include metadata.

        :param str fn: The full path to the file.

        :return: None.
        """
        # Store data as dictionaries.
        bodies = {}
        indx = 0

        # Dictionaries to attach wing reference surfaces to wing bodies using
        # reference surface ID as the key.
        wing_bodies = {}
        ref_surfs = {}

        # Data structures for fuselage reference surfaces
        fuselage_bodies = {}
        href_surfs = {}
        vref_surfs = {}

        # Read STEP file
        step_reader = StepRead(fn)
        master_shape = step_reader.shape

        # Iterate over master shape to find compounds for geometric sets. These
        # sets contain the metadata and the surfaces that make up the
        # component.
        for compound in master_shape.shape_iter:
            # Get the metadata
            name = step_reader.name_from_shape(compound)

            # Unnamed body
            if not name:
                indx += 1
                comp_name = '.'.join(['Body', str(indx)])
                msg = ' '.join(['---Processing OpenVSP component:', comp_name])
                logger.info(msg)
                solid, invalid = _build_solid(compound, self._divide)
                self._invalid += invalid
                if solid is not None:
                    body = Body(solid, comp_name)
                    bodies[comp_name] = body
                continue
            metadata = json.loads(name)

            # Process reference surfaces and continue
            key = 'm_SurfType'
            if key in metadata and metadata[key] == 99:
                # Get surface
                sref = ImportVSP.process_sref(compound)
                # Get Sref ID
                sref_id = metadata['ID']
                ref_surfs[sref_id] = sref
                continue
            elif key in metadata and metadata[key] == 100:
                # Fuselage horizontal sref
                f = compound.faces[0]
                sref = f.surface
                sref.set_udomain(-1., 1.)
                sref.set_vdomain(0., 1.)
                sref.object.ExchangeUV()
                sref.object.UReverse()
                sref_id = metadata['ID']
                href_surfs[sref_id] = sref
                continue
            elif key in metadata and metadata[key] == 101:
                f = compound.faces[0]
                sref = f.surface
                sref.set_udomain(-1., 1.)
                sref.set_vdomain(0., 1.)
                sref.object.ExchangeUV()
                sref.object.UReverse()
                sref_id = metadata['ID']
                vref_surfs[sref_id] = sref
                continue

            comp_name = metadata['m_Name']
            if comp_name in bodies:
                indx += 1
                comp_name = '.'.join([comp_name, str(indx)])

            # Process component.
            msg = ' '.join(['---Processing OpenVSP component:', comp_name])
            logger.info(msg)

            # Wing
            if metadata['m_Type'] == 5 and metadata['m_SurfType'] != 99:
                wing, invalid = _process_wing(compound, self._divide,
                                              self._restrict, self._tol,
                                              self._reloft, comp_name)
                self._invalid += invalid
                if wing is not None:
                    bodies[comp_name] = wing
                    sref_id = metadata['Sref ID']
                    wing_bodies[sref_id] = wing

            # Fuselage
            elif metadata['m_Type'] in [4, 9]:
                fuse, invalid = _process_fuse(compound, self._divide,
                                              comp_name)
                self._invalid += invalid
                if fuse is not None:
                    bodies[comp_name] = fuse
                    sref_id = metadata['Sref ID']
                    fuselage_bodies[sref_id] = fuse

            # Unknown
            else:
                solid, invalid = _build_solid(compound, self._divide)
                self._invalid += invalid
                if solid:
                    body = Body(solid, comp_name)
                    bodies[comp_name] = body

        # Attach wing reference surfaces to the bodies.
        for sref_id in wing_bodies:
            if sref_id not in ref_surfs:
                continue
            wing = wing_bodies[sref_id]
            sref = ref_surfs[sref_id]
            wing.set_sref(sref)

        # Attach fuselage reference surfaces to the bodies.
        for sref_id in fuselage_bodies:
            if sref_id in href_surfs:
                fuselage = fuselage_bodies[sref_id]
                sref = href_surfs[sref_id]
                fuselage.metadata.set('hsref', sref)
            if sref_id in vref_surfs:
                fuselage = fuselage_bodies[sref_id]
                sref = vref_surfs[sref_id]
                fuselage.metadata.set('vsref', sref)

        # Update
        self._bodies.update(bodies)

    def export_step(self, fn, label_solids=True, label_faces=False,
                    names=None):
        """
        Export the OpenVSP model as a STEP file using Extended Data Exchange.
        Each OpenVSP component will be a named product in the STEP file
        structure.

        :param str fn: The filename.
        :param bool label_solids: Option to label the solid bodies in the
            STEP entity. The name will be the same as the OpenVSP component.
        :param bool label_faces: Option to label the faces in each of the
            solids. Each face of the solid body will be labeled "Face 1",
            "Face 2", etc.
        :param names: List of Body names that will be included in export. If
            *None* then all are exported.
        :type names: collections.Sequence(str) or None

        :return: None.
        """
        # Initialize the document
        doc = XdeDocument()

        # Get bodies to include
        if names is None:
            bodies = self.all_bodies
        else:
            bodies = [self.get_body(name) for name in names]

        # Gather OpenVSP bodies and names and build single compound
        solids = []
        names = []
        for body in bodies:
            solids.append(body.shape)
            names.append(body.name)
        cmp = Compound.by_shapes(solids)

        # Add main shape and top-level assembly
        main = doc.add_shape(cmp, 'Vehicle')

        # Each body should be a product so names are transferred to other
        # CAD systems
        for name, solid in zip(names, solids):
            doc.add_subshape(main, solid, name)

        # Transfer the document and then modify the STEP item name directly
        # rather than using a label. This only applies when labeling
        # sub-shapes.
        doc.transfer_step()
        if label_solids or label_faces:
            for name, solid in zip(names, solids):
                if label_solids:
                    doc.set_shape_name(solid, name)
                if label_faces:
                    i = 1
                    for f in solid.faces:
                        name = ' '.join(['Face', str(i)])
                        doc.set_shape_name(f, name)
                        i += 1

        doc.write_step(fn)

    def save_bodies(self, fn):
        """
        Save the Body instances.

        :param str fn: The filename. The extension will be ".xbf" and appended
            if not provided.

        :return: *True* if saved, *False* otherwise.
        :rtype: bool

        .. note::

            This method only saves the label, shape, and color of the Body.
            User-defined metadata is currently not saved.
        """
        bodies = list(self.bodies.values())
        return Body.save_bodies(fn, *bodies)

    @staticmethod
    def rebuild_wing_solid(srfs, divide_closed=True, reloft=False, tol=0.01):
        """
        Rebuild a solid shape from the OpenVSP wing surface(s). If only one
        surface is provided then it is assumed that a single surface models the
        OML and it will be split and modified at the root, tip, and trailing
        edge. This single surface should have similar form and
        parametrization as the original OpenVSP surface. If more than once
        surface is provided then it is assumed that the surfaces were split
        during OpenVSP export and are simply sewn together to form the solid.

        :param srfs: The wing surface(s) used to rebuild the solid.
        :type srfs: collections.Sequence(afem.geometry.entities.Surface)
        :param bool divide_closed: Option to divide closed faces.
        :param bool reloft: For wings that are not split, this option will
            extract isocurves at each spanwise cross section, tessellate the
            curve using *tol*, and then approximate the section using a C1
            continuous curve. These curves are then used to generate a new
            wing surface. This method is experimental.
        :param float tol: Tolerance for approximation if
            *bspline_restrict* or *reloft* is *True*.

        :return: The new solid.
        :rtype: afem.topology.entities.Solid

        :raise ValueError: If no surfaces are provided.
        """
        faces = [Face.by_surface(s) for s in srfs]
        compound = Compound.by_shapes(faces)

        nsrfs = len(srfs)
        if nsrfs == 1:
            solid, _ = _process_unsplit_wing(compound, divide_closed, reloft,
                                             tol)
            return solid
        elif nsrfs > 1:
            solid, _ = _build_solid(compound, divide_closed)
            return solid
        else:
            raise ValueError('No surfaces provided.')

    @staticmethod
    def process_sref(compound):
        """
        Process a wing reference surface. The compound should contain a single
        face. The underlying surface of this faces will be used to generate a
        new NurbsSurface. Since the OpenVSP surface uses uniform
        parametrization, a chord line is extracted at each unique knot in the
        v-direction. Then a linear surface is lofted through these lines to
        form a bilinear surface where the u-direction is chordwise and the
        v-direction is spanwise.

        :param afem.topology.entities.Compound compound: The compound.

        :return: The reference surface.
        :rtype: afem.geometry.entities.NurbsSurface
        """
        # Get underlying surface
        srf = compound.faces[0].surface

        # Loft new surface
        vknots = srf.vknots
        crvs = []
        for vi in vknots:
            c = srf.v_iso(vi)
            crvs.append(c)
        srf = NurbsSurfaceByInterp(crvs, 1).surface
        return srf

    @staticmethod
    def rebuild_wing_sref(srf):
        """
        Attempt to rebuild a wing reference surface using the given OML
        surface. This method is intended to operate on OpenVSP surfaces that
        define the OML of a wing component and/or have similar parametrization.

        :param afem.geometry.entities.NurbsSurface srf: The surface.

        :return: The reference surface.
        :rtype: afem.geometry.entities.NurbsSurface
        """
        # For VSP, wing surfaces are degree=1 in the spanwise direction, so
        # each knot vector usually represents where a wing cross section is
        # located. For each knot vector in spanwise direction, generate a
        # straight line segment between the LE and TE (i.e., the chord line).
        # These chords will serve as the wing reference surface. This assumes
        # that the LE is at v=0.5 in the local parametric domain.
        uknots = srf.uknots
        le_param = srf.local_to_global_param('v', 0.5)
        te_param = srf.local_to_global_param('v', 0.)
        chords = []
        for u in uknots:
            le = srf.eval(u, le_param)
            te = srf.eval(u, te_param)
            c = NurbsCurveByPoints([le, te]).curve
            chords.append(c)

        # VSP wing components wrap around the ends and there may be duplicate
        # chord lines at the root and tip. Retain only unique lines based on LE
        # point.
        unique_chords = [chords[0]]
        for i in range(1, len(chords)):
            c0 = unique_chords[-1]
            ci = chords[i]
            if not ci.eval(0.).is_equal(c0.eval(0.)):
                unique_chords.append(ci)

        # Create degree=1 reference surface by skinning chord lines
        sref = NurbsSurfaceByInterp(unique_chords, 1).surface

        # Set domains to be 0 to 1.
        sref.set_udomain(0., 1.)
        sref.set_vdomain(0., 1.)

        return sref


def _build_solid(compound, divide_closed):
    """
    Try to build a solid from the OpenVSP compound of faces.

    :param afem.topology.entities.Compound compound: The compound.
    :param bool divide_closed: Option to divide closed faces.

    :return: The solid.
    :rtype: afem.topology.entities.Solid
    """
    # Get all the faces in the compound. The surfaces must be split. Discard
    # any with zero area.
    faces = []
    for face in compound.faces:
        area = SurfaceProps(face).area
        if area > 1.0e-7:
            faces.append(face)

    # Replace any planar B-Spline surfaces with planes.
    non_planar_faces = []
    planar_faces = []
    for f in faces:
        srf = f.surface
        try:
            pln = srf.as_plane()
            if pln:
                w = f.outer_wire
                # Fix the wire because they are usually degenerate edges in
                # the planar end caps.
                builder = BRepBuilderAPI_MakeWire()
                for e in w.edges:
                    if LinearProps(e).length > 1.0e-7:
                        builder.Add(e.object)
                w = builder.Wire()
                fix = ShapeFix_Wire()
                fix.Load(w)
                fix.SetSurface(pln.object)
                fix.FixReorder()
                fix.FixConnected()
                fix.FixEdgeCurves()
                fix.FixDegenerated()
                w = Wire(fix.WireAPIMake())
                fnew = Face.by_wire(w)
                planar_faces.append(fnew)
            else:
                non_planar_faces.append(f)
        except RuntimeError:
            logger.info('Failed to check for planar face...')
            non_planar_faces.append(f)

    # Make a compound of the faces
    shape = Compound.by_shapes(non_planar_faces + planar_faces)

    # Split closed faces
    if divide_closed:
        shape = DivideClosedShape(shape).shape

    # Sew shape
    sewn_shape = SewShape(shape).sewed_shape
    if isinstance(sewn_shape, Face):
        sewn_shape = sewn_shape.to_shell()

    # Attempt to unify planar faces
    shell = UnifyShape(sewn_shape, False).shape

    # Make solid
    if not isinstance(shell, Shell):
        logger.info('\tA valid shell was not able to be generated.')
        check = CheckShape(shell)
        if not check.is_valid:
            logger.info('\tShape errors:')
            check.log_errors()
        return shell, check.invalid_shapes

    solid = Solid.by_shell(shell)

    # Limit tolerance
    FixShape.limit_tolerance(solid)

    # Check the solid and attempt to fix
    invalid = []
    check = CheckShape(solid)
    if not check.is_valid:
        logger.info('\tFixing the solid...')
        solid = FixShape(solid).shape
        check = CheckShape(solid)
        if not check.is_valid:
            logger.info('\t...solid could not be fixed.')
            logger.info('\tShape errors:')
            check.log_errors()
            failed = check.invalid_shapes
            invalid += failed
    else:
        tol = solid.tol_avg
        logger.info(
            '\tSuccessfully generated solid with tolerance={}'.format(tol))

    return solid, invalid


def _process_wing(compound, divide_closed, bspline_restrict, tol, reloft,
                  name):
    # Note that for VSP wings, the spanwise direction is u and the chord
    # direction is v, where v=0 is the TE and follows the lower surface fwd to
    # the LE, and then aft along the upper surface to the TE.

    # Process based on number of faces in compound assuming split/no split
    # option was used.
    faces = compound.faces
    vsp_surf = None
    if len(faces) == 1:
        solid, invalid = _process_unsplit_wing(compound, divide_closed, reloft,
                                               tol)
        vsp_surf = faces[0].surface
    else:
        solid, invalid = _build_solid(compound, divide_closed)

    if not solid:
        return None

    if bspline_restrict:
        solid = _bspline_restrict(solid, tol)

    wing = Body(solid, name)

    if vsp_surf:
        wing.metadata.set('vsp surface', vsp_surf)
        upr_srf = vsp_surf.copy()
        v_le = vsp_surf.local_to_global_param('v', 0.5)
        upr_srf.segment(vsp_surf.u1, vsp_surf.u2, v_le, vsp_surf.v2)
        wing.metadata.set('upper surface', upr_srf)
        lwr_srf = vsp_surf.copy()
        lwr_srf.segment(vsp_surf.u1, vsp_surf.u2, vsp_surf.v1, v_le)
        wing.metadata.set('lower surface', lwr_srf)

    return wing, invalid


def _process_fuse(compound, divide_closed, name):
    # For VSP fuselages, the longitudinal direction is u, and the
    # circumferential direction is v.
    # Build the solid.
    solid, invalid = _build_solid(compound, divide_closed)
    if not solid:
        return None, invalid

    fuselage = Body(solid, name)

    faces = compound.faces
    if len(faces) == 1:
        vsp_surf = faces[0].surface
        fuselage.metadata.set('vsp surface', vsp_surf)

    return fuselage, invalid


def _process_unsplit_wing(compound, divide_closed, reloft, tol):
    # Process a wing that was generated without "Split Surfs" option.

    faces = compound.faces
    if len(faces) != 1:
        return None, None
    face = faces[0]

    # Get the surface.
    master_surf = face.surface
    uknots, vknots = master_surf.uknots, master_surf.vknots
    vsplit = master_surf.local_to_global_param('v', 0.5)

    # Detect if a thick trailing edge exists. If so, adjust the chordwise
    # locations where the TE is trimmed.
    vk_te1, vk_te2 = vknots[1], vknots[-2]
    for u in uknots:
        p1 = master_surf.eval(u, vknots[1])
        p2 = master_surf.eval(u, vknots[-2])
        if p1.is_equal(p2):
            vk_te1, vk_te2 = vknots[0], vknots[-1]

    # Segment off the end caps and the trailing edges.
    u1, u2 = uknots[1], uknots[-2]
    v1, v2 = vk_te1, vk_te2
    s1 = master_surf.copy()
    s1.segment(u1, u2, v1, v2)

    # Reloft the surface by tessellating a curve at each spanwise knot. This
    # enforces C1 continuity but assumes linear spanwise wing which may not
    # support blending wing sections in newer versions of OpenVSP. Also, since
    # the tessellated curves may not match up to the wing end caps making
    # sewing unreliable, flat end caps are assumed.
    if reloft:
        s1 = _reloft_wing_surface(s1, tol)

        # Generate new flat end caps using isocurves at the root and tip of
        # this new surface
        c0 = s1.v_iso(s1.v1)
        c1 = s1.v_iso(s1.v2)
        e0 = Edge.by_curve(c0)
        e1 = Edge.by_curve(c1)
        w0 = Wire.by_edge(e0)
        w1 = Wire.by_edge(e1)
        f0 = Face.by_wire(w0)
        f1 = Face.by_wire(w1)

        # Make faces of surfaces
        f = Face.by_surface(s1)
        new_faces = [f, f0, f1]
    else:
        # Reparamterize knots in spanwise direction to be chord length instead
        # of uniform. Use isocurve at quarter-chord to determine knot values.
        # This only works as long as surfaces are linear.
        c0 = s1.u_iso(s1.u1)
        c0.segment(vsplit, c0.u2)
        qc_u = PointFromParameter(c0, vsplit, 0.25 * c0.length).parameter
        c = s1.v_iso(qc_u)
        pnts = [c.eval(u) for u in c.knots]
        new_uknots = geom_utils.chord_parameters(pnts, 0., 1.)
        s1.set_uknots(new_uknots)

        # Segment off end caps
        u1, u2 = uknots[0], uknots[1]
        v1, v2 = vk_te1, vsplit
        s2 = master_surf.copy()
        s2.segment(u1, u2, v1, v2)

        u1, u2 = uknots[0], uknots[1]
        v1, v2 = vsplit, vk_te2
        s3 = master_surf.copy()
        s3.segment(u1, u2, v1, v2)

        u1, u2 = uknots[-2], uknots[-1]
        v1, v2 = vk_te1, vsplit
        s4 = master_surf.copy()
        s4.segment(u1, u2, v1, v2)

        u1, u2 = uknots[-2], uknots[-1]
        v1, v2 = vsplit, vk_te2
        s5 = master_surf.copy()
        s5.segment(u1, u2, v1, v2)

        # Make faces of surfaces
        new_faces = []
        for s in [s1, s2, s3, s4, s5]:
            f = Face.by_surface(s)
            new_faces.append(f)

    # Segment off TE.
    u1, u2 = uknots[0], uknots[-1]
    v1, v2 = vknots[0], vknots[1]
    s6 = master_surf.copy()
    s6.segment(u1, u2, v1, v2)

    u1, u2 = uknots[0], uknots[-1]
    v1, v2 = vknots[-2], vknots[-1]
    s7 = master_surf.copy()
    s7.segment(u1, u2, v1, v2)

    # Split the TE surface at each u-knot.
    usplits = occ_utils.to_tcolstd_hseq_real(uknots[1:-1])

    split = ShapeUpgrade_SplitSurface()
    split.Init(s6.object)
    split.SetUSplitValues(usplits)
    split.Perform()
    comp_surf1 = split.ResSurfaces()

    split = ShapeUpgrade_SplitSurface()
    split.Init(s7.object)
    split.SetUSplitValues(usplits)
    split.Perform()
    comp_surf2 = split.ResSurfaces()

    # For each patch in the composite surfaces create a face.
    for i in range(1, comp_surf1.NbUPatches() + 1):
        for j in range(1, comp_surf1.NbVPatches() + 1):
            hpatch = comp_surf1.Patch(i, j)
            f = BRepBuilderAPI_MakeFace(hpatch, 0.).Face()
            new_faces.append(f)

    for i in range(1, comp_surf2.NbUPatches() + 1):
        for j in range(1, comp_surf2.NbVPatches() + 1):
            hpatch = comp_surf2.Patch(i, j)
            f = BRepBuilderAPI_MakeFace(hpatch, 0.).Face()
            new_faces.append(f)

    # Put all faces into a compound a generate solid.
    new_compound = Compound.by_shapes(new_faces)

    return _build_solid(new_compound, divide_closed)


def _bspline_restrict(solid, tol):
    """
    Attempt to re-fit OpenVSP surfaces with more continuity.
    """
    # Use dmax=1 because that was only way to get the tool actually refit the
    # surfaces other than another surface where the multiplicity equaled the
    # degree. Not sure why the tool operates this way.
    logger.info('\tApplying ShapeBSplineRestriction tool...')
    tool = ShapeBSplineRestriction(solid, dmax=1, tol3d=tol)
    if not tool.is_done:
        logger.info('Method unsuccessful. Using original solid.')
        return solid

    # Get new shape and solid
    new_solid = tool.modified_shape(solid)

    # Limit/fix tolerance
    FixShape.limit_tolerance(new_solid)
    tol = new_solid.tol_avg
    if not CheckShape(new_solid).is_valid:
        logger.info('Shape invalid. Using original solid.')
        return solid

    logger.info('\tMethod successful with surface error: {}'.format(
        tool.error_surface))
    logger.info('\tNew shape tolerance: {}'.format(tol))

    return new_solid


def _reloft_wing_surface(srf, tol):
    """
    Attempt to reloft an OpenVSP wing surface which was not split to achieve
    higher continuity.
    """
    logger.info('\tAttempting to reloft the surface...')
    # Gather isocurves at each section, tessellate, and approximate
    crvs = []
    for u in srf.uknots:
        c0 = srf.u_iso(u)
        adp_crv = AdaptorCurve.to_adaptor(c0)
        tool = GCPnts_QuasiUniformDeflection(adp_crv.object, tol)
        if not tool.IsDone():
            logger.info('\tTessellation failed. Using original surface.')
            return srf
        pnts = [c0.eval(tool.Parameter(i)) for i in
                range(1, tool.NbPoints() + 1)]
        c = NurbsCurveByApprox(pnts, tol=tol, continuity=Geometry.C1).curve
        crvs.append(c)
    return NurbsSurfaceByInterp(crvs, 1).surface
