# AFEM — Airframe Finite Element Modeler

[![Documentation Status](https://readthedocs.org/projects/afem/badge/?version=latest)](http://afem.readthedocs.io/en/latest/?badge=latest)
[![Join the chat at https://gitter.im/AFEM_/Lobby](https://badges.gitter.im/AFEM_/Lobby.svg)](https://gitter.im/AFEM_/Lobby?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

![Workflow](https://github.com/LaughlinResearch/AFEM/workflows/Workflow/badge.svg)
[![Anaconda-Server Badge](https://anaconda.org/laughlinresearch/afem/badges/installer/conda.svg)](https://anaconda.org/laughlinresearch/afem)
[![Anaconda-Server Badge](https://anaconda.org/laughlinresearch/afem/badges/platforms.svg)](https://anaconda.org/laughlinresearch/afem)
[![Anaconda-Server Badge](https://anaconda.org/laughlinresearch/afem/badges/downloads.svg)](https://anaconda.org/laughlinresearch/afem)

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

## Getting started using conda
[Conda packages](https://anaconda.org/trelau/dashboard/) are
available for a number of platforms and Python versions. Get started with:

    conda create -n afem python=3.7
    activate afem
    conda install -c conda-forge -c trelau afem
    
This will create an environment named "afem" and install `AFEM` and all
necessary dependencies. You can replace the "afem" environment name with
anything you'd like.

The best way to get started is to examine and run the files in the *examples/*
folder. Running the script:

    python structure_wingbox.py
    
should generate an image similar to the one shown below. Remember to make sure
the appropriate environment is active when using AFEM is applicable.

![wingbox](./docs/source/resources/wingbox.png)

Installation files can be cleaned by:

    conda clean -a

# Building documentation
The documentation can be built from sources using sphinx. Install sphinx and
sphinx_rtd_theme in the desired conda environment by:

    conda install sphinx sphinx_rtd_theme
    
Then navigate to the *docs/* folder and run:

    make html

This should build html documentation in a *docs/build/html* folder. Open the 
index.html file with a web browser.

# Examples
Finding mesh nodes shared between the spars and ribs:
![wingbox](./docs/source/resources/associative_mesh.png)

Fuselage section with frames, floors, and floor supports modeled as beams:
![fuselage](./docs/source/resources/fuselage_section.png)

Flexibility to define custom structural components like a circular spar:
![spar](./docs/source/resources/circular_spar.png)

Structural geometry defined and joined between the wing and fuselage:
![spar](./docs/source/resources/wing_body.png)
