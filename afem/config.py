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
import logging
import sys

# Initialize logger.
with open('afem.log', 'w') as log:
    log.write('-----------------------------\n')
    log.write('AFEM LOGGING FILE INITIALIZED\n')
    log.write('-----------------------------\n')
logger = logging.getLogger('afem')
logger.setLevel(logging.INFO)
_fh = logging.FileHandler('afem.log')
_fmt = logging.Formatter('%(levelname)s: %(message)s')
_fh.setFormatter(_fmt)
logger.addHandler(_fh)

# Dictionary for units
units_dict = {'i': 'INCH',
              'in': 'INCH',
              'inch': 'INCH',
              'inches': 'INCH',
              'f': 'FT',
              'ft': 'FT',
              'foot': 'FT',
              'feet': 'FT',
              'm': 'M',
              'meter': 'M',
              'meters': 'M',
              'mm': 'MM',
              'millimeter': 'MM',
              'millimeters': 'MM'}

log_dict = {'debug': logging.DEBUG,
            'info': logging.INFO,
            'warning': logging.WARNING,
            'error': logging.ERROR,
            'critical': logging.CRITICAL}

__all__ = ["Settings", "logger", "units_dict"]


class Settings(object):
    """
    Settings.

    :var str units: The default units ('in', 'ft', 'm', 'mm'). The default
        value is inches ('INCH').
    """
    # Class variables for settings
    units = 'INCH'

    @classmethod
    def set_units(cls, units='in'):
        """
        Set units.

        :param str units: The units ('in', 'ft', 'm', 'mm').

        :return: None.

        :raise KeyError: If the given units are not supported.
        """
        units = units.lower()
        cls.units = units_dict[units]

    @staticmethod
    def log_to_console():
        """
        Option to add a stream handler to the main file logger to log to
        stdout.

        :return: None.
        """
        chdlr = logging.StreamHandler(sys.stdout)
        chdlr.setFormatter(_fmt)
        logger.addHandler(chdlr)

    @staticmethod
    def set_loggging_level(level='info'):
        """
        Set the logging level.

        :param str level: Logging level ('debug', 'info', 'warning', 'error',
            'critical').

        :return: None.

        :raise KeyError: If the given level is not supported.
        """
        level = level.lower()
        logger.setLevel(log_dict[level])
