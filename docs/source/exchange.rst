Exchange
========
The ``afem.exchange`` package provides a number of common tools for data
exchange including BREP (OpenCASCADE native format), STEP, IGES, and STL. The
tools can be imported by::

    from afem.exchange import *

Some specific use-case tools are provided including streamlined processing of
STEP files from NASA's OpenVSP program. These data exchange tools can be used
to import or export data to other CAD or CAE tools.

BREP
----
The ``afem.exchange.brep`` module contains two simple methods for reading and
writing OpenCASCADE BREP files.

.. automodule:: afem.exchange.brep

STEP
----
.. automodule:: afem.exchange.step

IGES
----
.. automodule:: afem.exchange.iges

STL
---
.. automodule:: afem.exchange.stl

XDE
---
.. automodule:: afem.exchange.xde

.. _importvsp:

OpenVSP
-------
The ``ImportVSP`` tool is developed specifically for translating STEP files
exported from the OpenVSP program. The primary motivation for this tool can be
summarized by needs:

1. Automatically translate the OpenVSP standard components (e.g., MS_Wing,
   Fuselage, etc.) into watertight solid shapes.

2. Communicate additional metadata about the OpenVSP model and components (e.g.,
   name, type, etc.) through the STEP file.

The first need is accomplished by essentially pre-processing the OpenVSP
geometry from the STEP file based on a set of assumptions and rules:

* Surfaces of unique OpenVSP components are grouped into geometric sets in the
  STEP file. These are translated to compounds and it is assumed that they
  define a single OpenVSP component (or one body of a component if symmetric).

* Each surface of the referenced by the geometric set is first checked to
  see if it is planar. If so, the underlying surface is replaced with a plane
  instead of the B-Spline surface. This may be applicable to surfaces
  representing wing caps or thick trailing edges.

* The surfaces are sewn together and at this point should form a closed solid
  shape.

In a script-based environment, the user cannot "point and click" to identify
which solid body represents a particular component. For that reason, a forked
version of OpenVSP was made and modified to include useful metadata in the
STEP file. The metadata includes the component name, type, and additional
degenerate geometry like a wing reference surface. All of this can be
optionally exported on the STEP export screen (default is off). Without this
metadata, solid bodies can still be built, but are given generic names like
"Body.1" and it will be left to the user to decipher which represents the
original OpenVSP component.

For now, the forked OpenVSP version with metadata can be found
`here <https://github.com/trelau/OpenVSP>`_
under the `step_metadata_support` and `step_metadata_support_v3.5.0` branches.
Support for version 3.5.0 was provided since the underlying surface
parametrization seemed to have changed afterwards and OpenCASCADE performance
suffered.

Both of these builds include the Python API.

.. automodule:: afem.exchange.vsp

NASTRAN
-------
.. automodule:: afem.exchange.nastran