#!/usr/bin/env python3

"""
Created on March 28, 2023

@author: Gary Black
"""

import argparse
import csv


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

  print(initRow1)
  print(initRow2)

  # init_accy.csv entries: nodeqty,xqty,zqty,Jacobian_elements,Yphys_scaled_terms,F_width,F_height,F_entries,F_min,F_max,F_sum,F_mean,eyex_width,eyex_height,eyex_entries,eyex_min,eyex_max,eyex_sum,eyex_mean,R_width,R_height,R_entries,R_min,R_max,R_sum,R_mean

  matchFlag = True

  # compare nodeqty, xqty, zqty to make sure it's the same model being compared
  # across simulations
  matchFlag = checkEqual(initRow1[0], initRow2[0], 'Different number of nodes')

  matchFlag = matchFlag and checkEqual(initRow1[1], initRow2[1], 'Different X dimension')

  matchFlag = matchFlag and checkEqual(initRow1[2], initRow2[2], 'Different Z dimension')

  if not matchFlag:
    print('Mismatched models being compared--exiting!\n', flush=True)
    exit()

  checkEqual(initRow1[3], initRow2[3], 'Different number of Jacobian elements')

  checkEqual(initRow1[4], initRow2[4], 'Different number of Yphys scaled terms')

  checkEqual(initRow1[5], initRow2[5], 'Different F matrix width')

  checkEqual(initRow1[6], initRow2[6], 'Different F matrix height')

  checkEqual(initRow1[7], initRow2[7], 'Different number of F matrix entries')

  print('End INITALIZATION comparison\n', flush=True)

  print('DONE!', flush=True)


if __name__ == "__main__":
  _main()

