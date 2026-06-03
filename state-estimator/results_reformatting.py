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
Created on April 3, 2026

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

  num_nodes = len(node_name)

  fout = open('results_data_forecasting.csv', 'w')
  fout.write('Timestamp,Node,kW,kvar,Voltage_pu,Angle_deg,Angle_rad\n')

  skipMin = 1 # Don't skip any timestamps
  #skipMin = 15 # Skip 15 minutes of timestamps
  skipSec = skipMin*60
  lineCount = 0

  skipEstimates = 12 # for results_data.csv

  PidxDict = {}
  QidxDict = {}

  with open('results_data.csv', 'r') as f1in, open('results_pq_data.csv', 'r') as f2in:
  #with open('results_data_pnv.csv', 'r') as f1in, open('results_data_pq.csv', 'r') as f2in:
    for line1, line2 in zip(f1in, f2in):
      items1 = line1.split(',')
      items2 = line2.split(',')
      if items1[0] == 'timestamp': # skip header
        # build P and Q index dictionaries from header line
        for i in range(1, len(items2)):
          node = items2[i][6:].strip()
          if items2[i].startswith('P_est_'):
            PidxDict[node] = i
          else:
            QidxDict[node] = i

        continue

      # discard the first 12 timestamps because state estimates are off
      # and would result in bad training data
      lineCount += 1
      if lineCount <= skipEstimates:
        continue

      timestamp = int(items1[0])
      if timestamp % skipSec != 0:
        continue

      print('timestamp: ' + items1[0], flush=True)

      for idx in range(num_nodes):
        ts = items1[0]
        vmag = items1[idx+1].strip()
        vang = items1[num_nodes+idx+1].strip()
        vang_rad = str(math.radians(float(vang)))
        node = node_name[idx]
        Pinj = 'NA'
        Qinj = 'NA'
        if node in PidxDict:
          Pinj = items2[PidxDict[node]].strip()
          Qinj = items2[QidxDict[node]].strip()

        fout.write(ts + ',' + node + ',' + Pinj + ',' + Qinj + ',' + vmag + ',' + vang + ',' + vang_rad + '\n')

      #break # debug break after first timestamp

  fout.close()

if __name__ == '__main__':
  _main()

