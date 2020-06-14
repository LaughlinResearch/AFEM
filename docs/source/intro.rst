About
=====
AFEM is a "fit-for-purpose" engineering development toolkit designed to support
the use of high-order structural analysis during the early phases of aircraft
conceptual design. As a development toolkit, it provides the engineer with a
flexible, modular, and extensible library of components and tools to rapidly
build a useful structural model. It is **not** an end-user GUI application, but
rather a library enabling engineers to rapidly build their own application
specific tools and processes, encoding their own design rules and best
practices along the way.

Although AFEM targets airframe design and analysis applications, really only
the ``afem.structure`` and ``afem.oml`` packages directly support that cause.
All the other packages provide a more general and "Pythonic" set of entities
and tools that could potentially be used to develop applications in other
disciplines and/or domains.

Technology Stack
================
The AFEM core technology stack includes:

* `Python <https://www.python.org/>`_: The Python programming language enables
  rapid development and integration with other systems.

* `OpenCASCADE <https://www.opencascade.com>`_: This mature library provides
  advanced geometric modeling and CAD functionality and is under active
  development.

* `Netgen <https://sourceforge.net/projects/netgen-mesher>`_: This library
  enables advanced meshing capabilities including 3-D tetrahedral and 2-D
  unstructured quad-dominated surface meshing.

* `Salome Platform <http://www.salome-platform.org>`_: The core meshing library
  from this open source application serves as the central component for
  AFEM's mesh generation capabilities.

* `pyOCCT <https://github.com/trelau/pyOCCT>`_: This open source
  project provides Python bindings to the OpenCASCADE and Salome Platform
  meshing libraries.
