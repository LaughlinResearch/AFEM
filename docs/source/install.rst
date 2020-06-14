Installation
============
This section explains how to install AFEM and required dependencies. For new
users, `installing Anaconda <https://www.continuum.io/downloads>`_ is
**highly recommended**. Anaconda conveniently installs Python and many other
commonly used packages for scientific computing.

A specific environment for AFEM should be created using an Anaconda Command
Prompt. Installing AFEM and all necessary dependencies will look like::

    conda create -n afem python=3.7
    activate afem
    conda install -c conda-forge -c trelau afem

This example using Python 3.7, but any version of Python supported by AFEM can
be used.

Building Documentation
----------------------
The documentation can be built from sources using sphinx. Install sphinx and
sphinx_rtd_theme in the desired conda environment by::

    conda install sphinx sphinx_rtd_theme

Then navigate to the *docs/* folder and run::

    make html

This should build html documentation in a *docs/build/html* folder. Open the
index.html file with a web browser.
