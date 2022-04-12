#!/usr/bin/env python3

# ------------------------------------------------------------------------------
# Copyright (c) 2020, Battelle Memorial Institute All rights reserved.
# Battelle Memorial Institute (hereinafter Battelle) hereby grants permission to any person or entity
# lawfully obtaining a copy of this software and associated documentation files (hereinafter the
# Software) to redistribute and use the Software in source and binary forms, with or without modification.
# Such person or entity may use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and may permit others to do so, subject to the following conditions:
# Redistributions of source code must retain the above copyright notice, this list of conditions and the
# following disclaimers.
# Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
# the following disclaimer in the documentation and/or other materials provided with the distribution.
# Other than as used herein, neither the name Battelle Memorial Institute or Battelle may be used in any
# form whatsoever without the express written consent of Battelle.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# BATTELLE OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
# General disclaimer for use with OSS licenses
#
# This material was prepared as an account of work sponsored by an agency of the United States Government.
# Neither the United States Government nor the United States Department of Energy, nor Battelle, nor any
# of their employees, nor any jurisdiction or organization that has cooperated in the development of these
# materials, makes any warranty, express or implied, or assumes any legal liability or responsibility for
# the accuracy, completeness, or usefulness or any information, apparatus, product, software, or process
# disclosed, or represents that its use would not infringe privately owned rights.
#
# Reference herein to any specific commercial product, process, or service by trade name, trademark, manufacturer,
# or otherwise does not necessarily constitute or imply its endorsement, recommendation, or favoring by the United
# States Government or any agency thereof, or Battelle Memorial Institute. The views and opinions of authors expressed
# herein do not necessarily state or reflect those of the United States Government or any agency thereof.
#
# PACIFIC NORTHWEST NATIONAL LABORATORY operated by BATTELLE for the
# UNITED STATES DEPARTMENT OF ENERGY under Contract DE-AC05-76RL01830
# ------------------------------------------------------------------------------
"""
Created on Sept 29, 2020

@author: Gary D. Black
"""

__version__ = '0.1.0'

import sys
import json

# gridappsd-python module
from gridappsd import GridAPPSD
from gridappsd.simulation import Simulation

# global variables
gapps = None
appName = None
sim_id = None
feeder_mrid = None


def _main():
    global appName, sim_id, feeder_mrid, gapps

    if len(sys.argv)<2 or '-help' in sys.argv:
        usestr =  '\nUsage: ' + sys.argv[0] + ' #nodes\n'
        usestr += '''
Optional command line arguments:
        -help: show this usage message
        '''
        print(usestr, file=sys.stderr, flush=True)
        exit()

    appName = sys.argv[0]

    sim_config_file = './sim_starter/' + sys.argv[1] + '-config.json'
    gapps = GridAPPSD()

    with open(sim_config_file) as config_fp:
        sim_config = json.load(config_fp)

    # if there is a sys.argv[2] command line argument, that indicates not to start a simulation
    if len(sys.argv)<3:
        print('SIM_STARTER initializing simulation from: ' + sim_config_file, file=sys.stderr, flush=True)
        sim = Simulation(gapps, sim_config)
        print('SIM_STARTER about to start simulation...', file=sys.stderr, flush=True)
        sim.start_simulation(timeout=90)
        sim_id = sim.simulation_id
        print('SIM_STARTER simulation started with id: ' + sim_id, file=sys.stderr, flush=True)
        print(sim_id, flush=True)

    # need to dump sim_config before passing it on to downstream modules
    # in order to convert single quotes to double quotes
    sim_req = json.dumps(sim_config)
    print(sim_req, flush=True)

    gapps.disconnect()


if __name__ == '__main__':
    _main()

