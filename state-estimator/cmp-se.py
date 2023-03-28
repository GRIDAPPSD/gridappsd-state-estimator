#!/usr/bin/env python3

"""
Created on March 28, 2023

@author: Gary Black
"""

import argparse
import csv


NearZero = 1e-12
FlagPerDiff = 5.0

def checkDiff(value1, value2, message):
  equalFlag = True
  if value1 == value2:
    print(message + ' values are equal', flush=True)
  else:
    if abs(value1)<NearZero and abs(value2)<NearZero:
      print(message + ' values are both within computational precision of zero and considered equal', flush=True)
    else:
      equalFlag = False
      perdiff = 100 * abs(value1 - value2)/((value1 + value2)/2)
      if perdiff >= FlagPerDiff:
        print('==> ' + message + ' values % diff: ' + str(round(perdiff, 4)) + ' <==', flush=True)
      else:
        print(message + ' values % diff: ' + str(round(perdiff, 4)), flush=True)

  return equalFlag


def checkEqual(value1, value2, message):
  equalFlag = True
  if value1 != value2:
    equalFlag = False
    print(message + ', sim 1: ' + str(value1) + ', sim 2: ' + str(value2), flush=True)

  return equalFlag


def _main():
  parser = argparse.ArgumentParser()
  parser.add_argument("sim_dir1", help="Simulation Directory 1")
  parser.add_argument("sim_dir2", help="Simulation Directory 2")
  opts = parser.parse_args()

  simDir1 = opts.sim_dir1 + '/test_suite/'
  simDir2 = opts.sim_dir2 + '/test_suite/'

  print('Begin INITALIZATION comparison:', flush=True)

  fp1 = open(simDir1 + 'init_accy.csv', 'r')
  fp2 = open(simDir2 + 'init_accy.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  next(reader1) # skip header rows
  next(reader2)

  initRow1 = next(reader1)
  initRow2 = next(reader2)

  #print(initRow1)
  #print(initRow2)

  # init_accy.csv entries: nodeqty,xqty,zqty,Jacobian_elements,Yphys_scaled_terms,F_width,F_height,F_entries,F_min,F_max,F_sum,F_mean,eyex_width,eyex_height,eyex_entries,eyex_min,eyex_max,eyex_sum,eyex_mean,R_width,R_height,R_entries,R_min,R_max,R_sum,R_mean

  # compare nodeqty, xqty, zqty to make sure it's the same model being compared
  # across simulations
  matchFlag = checkEqual(initRow1[0], initRow2[0], 'Different model number of nodes')

  matchFlag = matchFlag and checkEqual(initRow1[1], initRow2[1], 'Different model X dimension')

  matchFlag = matchFlag and checkEqual(initRow1[2], initRow2[2], 'Different model Z dimension')

  if not matchFlag:
    print('Mismatched models being compared--exiting!\n', flush=True)
    exit()

  checkEqual(initRow1[3], initRow2[3], 'Different number of Jacobian elements')

  checkEqual(initRow1[4], initRow2[4], 'Different number of Yphys scaled terms')

  # F
  checkEqual(initRow1[5], initRow2[5], 'Different F matrix width')
  checkEqual(initRow1[6], initRow2[6], 'Different F matrix height')
  checkEqual(initRow1[7], initRow2[7], 'Different number of F matrix entries')
  checkDiff(initRow1[8], initRow2[8], 'F matrix min')
  checkDiff(initRow1[9], initRow2[9], 'F matrix max')
  checkDiff(initRow1[11], initRow2[11], 'F matrix mean')

  # eyex
  checkEqual(initRow1[12], initRow2[12], 'Different eyex matrix width')
  checkEqual(initRow1[13], initRow2[13], 'Different eyex matrix height')
  checkEqual(initRow1[14], initRow2[14], 'Different number of eyex matrix entries')
  checkDiff(initRow1[15], initRow2[15], 'eyex matrix min')
  checkDiff(initRow1[16], initRow2[16], 'eyex matrix max')
  checkDiff(initRow1[18], initRow2[18], 'eyex matrix mean')

  # R
  checkEqual(initRow1[19], initRow2[19], 'Different R matrix width')
  checkEqual(initRow1[20], initRow2[20], 'Different R matrix height')
  checkEqual(initRow1[21], initRow2[21], 'Different number of R matrix entries')
  checkDiff(initRow1[22], initRow2[22], 'R matrix min')
  checkDiff(initRow1[23], initRow2[23], 'R matrix max')
  checkDiff(initRow1[25], initRow2[25], 'R matrix mean')

  # tests of checkDiff
  checkDiff(10.0, 8.0, 'Test')
  checkDiff(1e-13, 1e-16, 'Zero')
  checkDiff(1e-8, 1e-16, 'Low')

  print('End INITALIZATION comparison\n', flush=True)

  print('DONE!', flush=True)


if __name__ == "__main__":
  _main()

