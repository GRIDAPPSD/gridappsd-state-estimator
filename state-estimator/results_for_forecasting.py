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
Created on January 28, 2026

@author: Gary D. Black
"""

__version__ = '0.1.0'

import sys
import math
import numpy as np


def _main():
  vnom_dict = {}
  with open('vnom.csv', 'r') as fin:
    for line in fin:
      items = line.split(',')
      if items[0] == 'Nodename': # skip header
        continue
      vnom_dict[items[0]] = (float(items[1]), float(items[2]))

  node_name = []
  vnom_list = []
  with open('nodelist.csv', 'r') as fin:
    for line in fin:
      name = line.strip('\"\n')
      node_name.append(name)
      vnom_list.append(vnom_dict[name])
  #print(node_name)
  #print(vnom_list)

  num_nodes = len(node_name)
  Y = np.zeros((num_nodes, num_nodes), dtype=complex)

  with open('ysparse.csv', 'r') as fin:
    for line in fin:
      items = line.split(',')
      if items[0] == 'Row': # skip header
        continue

      Y[int(items[0])-1][int(items[1])-1] = \
      Y[int(items[1])-1][int(items[0])-1] = \
                         complex(float(items[2]), float(items[3]))
  #print(Y)

  fout = open('results_for_forecasting_data.csv', 'w')
  fout.write('Timestamp,Node,kW,kvar,Voltage_pu,Angle_deg,Angle_rad\n')

  skipMin = 1 # Don't skip any timestamps
  #skipMin = 15 # Skip 15 minutes of timestamps
  skipSec = skipMin*60

  with open('results_data.csv', 'r') as fin:
    for line in fin:
      items = line.split(',')
      if items[0] == 'timestamp': # skip header
        continue

      timestamp = int(items[0])
      if timestamp % skipSec != 0:
        continue

      print('timestamp: ' + items[0], flush=True)

      V_mag_pu = np.zeros(num_nodes, dtype=float)
      V_mag = np.zeros(num_nodes, dtype=float)
      V_ang = np.zeros(num_nodes, dtype=float)

      for idx in range(num_nodes):
        V_mag_pu[idx] = float(items[idx+1])
        V_mag[idx] = float(items[idx+1]) * vnom_list[idx][0]
        V_ang[idx] = math.radians(float(items[num_nodes+idx+1]))
        #V_ang[idx] = math.radians(vnom_list[idx][1])
      #print(V_mag)
      #print(V_ang)

      V = V_mag * np.exp(1j * V_ang)
      #print(V)

      I = Y @ V
      #print(I)

      S = V * np.conj(I)
      #print(S)

      for idx in range(num_nodes):
        fout.write(items[0] + ',' + node_name[idx] + ',' + str(S[idx].real/1000.0) + ',' + str(S[idx].imag/1000.0) + ',' + str(V_mag_pu[idx]) + ',' + str(math.degrees(V_ang[idx])) + ',' + str(V_ang[idx]) + '\n')
        #print(node_name[idx] + ': P: ' + str(S[idx].real/1000.0) + ', Q: ' + str(S[idx].imag/1000.0))

      #break # debug break after first timestamp

  fout.close()

if __name__ == '__main__':
  _main()

