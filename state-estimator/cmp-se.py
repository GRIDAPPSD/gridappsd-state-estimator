#!/usr/bin/env python3

"""
Created on March 28, 2023

@author: Gary Black
"""

import argparse
import csv

def _main():
  parser = argparse.ArgumentParser()
  parser.add_argument("sim_dir1", help="Simulation Directory 1")
  parser.add_argument("sim_dir2", help="Simulation Directory 2")
  opts = parser.parse_args()

  simDir1 = opts.sim_dir1 + '/test_suite/'
  simDir2 = opts.sim_dir2 + '/test_suite/'

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

  exitFlag = False

  # compare nodeqty, xqty, zqty to make sure it's the same model being compared
  # across simulations
  if initRow1[0] != initRow2[0]:
    exitFlag = True
    print('Different number of nodes, sim 1: ' + str(initRow1[0]) + ', sim 2: ' + str(initRow2[0]), flush=True)

  if initRow1[1] != initRow2[1]:
    exitFlag = True
    print('Different X dimension, sim 1: ' + str(initRow1[1]) + ', sim 2: ' + str(initRow2[1]), flush=True)

  if initRow1[2] != initRow2[2]:
    exitFlag = True
    print('Different Z dimension, sim 1: ' + str(initRow1[2]) + ', sim 2: ' + str(initRow2[2]), flush=True)

  if exitFlag:
    print('Mismatched models being compared--exiting!\n', flush=True)
    exit()

  if initRow1[3] != initRow2[3]:
    print('Different number of Jacobian elements, sim 1: ' + str(initRow1[3]) + ', sim 2: ' + str(initRow2[3]), flush=True)

  if initRow1[4] != initRow2[4]:
    print('Different number of Yphys scaled terms, sim 1: ' + str(initRow1[4]) + ', sim 2: ' + str(initRow2[4]), flush=True)

  print('DONE!', flush=True)


if __name__ == "__main__":
  _main()

