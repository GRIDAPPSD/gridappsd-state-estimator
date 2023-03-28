#!/usr/bin/env python3

"""
Created on March 28, 2023

@author: Gary Black
"""

import argparse
import csv


# absolute values less than this are considered zero due to limits on
# computational precision
NearZero = 1e-12

# print a warning flag for percent difference values greater than this
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
      if perdiff > FlagPerDiff:
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

  fp1.close()
  fp2.close()

  print('End INITALIZATION comparison\n', flush=True)

  # for this pass just process the first row of data in each file to compare
  # matrix dimensions that should be the same across all timestamps

  print('Begin ESTIMATE matrix dimensions comparison:', flush=True)

  fp1 = open(simDir1 + 'est_accy.csv', 'r')
  fp2 = open(simDir2 + 'est_accy.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  next(reader1) # skip header rows
  next(reader2)

  estRow1 = next(reader1)
  estRow2 = next(reader2)

  #print(estRow1)
  #print(estRow2)

  # P
  checkEqual(estRow1[1], estRow2[1], 'Different P matrix width')
  checkEqual(estRow1[2], estRow2[2], 'Different P matrix height')
  checkEqual(estRow1[3], estRow2[3], 'Different number of P matrix entries')

  # P1
  checkEqual(estRow1[8], estRow2[8], 'Different P1 matrix width')
  checkEqual(estRow1[9], estRow2[9], 'Different P1 matrix height')
  checkEqual(estRow1[10], estRow2[10], 'Different number of P1 matrix entries')

  # P2
  checkEqual(estRow1[15], estRow2[15], 'Different P2 matrix width')
  checkEqual(estRow1[16], estRow2[16], 'Different P2 matrix height')
  checkEqual(estRow1[17], estRow2[17], 'Different number of P2 matrix entries')

  # P3
  checkEqual(estRow1[22], estRow2[22], 'Different P3 matrix width')
  checkEqual(estRow1[23], estRow2[23], 'Different P3 matrix height')
  checkEqual(estRow1[24], estRow2[24], 'Different number of P3 matrix entries')

  # Q
  checkEqual(estRow1[29], estRow2[29], 'Different Q matrix width')
  checkEqual(estRow1[30], estRow2[30], 'Different Q matrix height')
  checkEqual(estRow1[31], estRow2[31], 'Different number of Q matrix entries')

  # Ppre
  checkEqual(estRow1[36], estRow2[36], 'Different Ppre matrix width')
  checkEqual(estRow1[37], estRow2[37], 'Different Ppre matrix height')
  checkEqual(estRow1[38], estRow2[38], 'Different number of Ppre matrix entries')

  # x
  checkEqual(estRow1[43], estRow2[43], 'Different x matrix width')
  checkEqual(estRow1[44], estRow2[44], 'Different x matrix height')
  checkEqual(estRow1[45], estRow2[45], 'Different number of x matrix entries')

  # xpre
  checkEqual(estRow1[50], estRow2[50], 'Different xpre matrix width')
  checkEqual(estRow1[51], estRow2[51], 'Different xpre matrix height')
  checkEqual(estRow1[52], estRow2[52], 'Different number of xpre matrix entries')

  # J
  checkEqual(estRow1[57], estRow2[57], 'Different J matrix width')
  checkEqual(estRow1[58], estRow2[58], 'Different J matrix height')
  checkEqual(estRow1[59], estRow2[59], 'Different number of J matrix entries')

  # S1
  checkEqual(estRow1[64], estRow2[64], 'Different S1 matrix width')
  checkEqual(estRow1[65], estRow2[65], 'Different S1 matrix height')
  checkEqual(estRow1[66], estRow2[66], 'Different number of S1 matrix entries')

  # S2
  checkEqual(estRow1[71], estRow2[71], 'Different S2 matrix width')
  checkEqual(estRow1[72], estRow2[72], 'Different S2 matrix height')
  checkEqual(estRow1[73], estRow2[73], 'Different number of S2 matrix entries')

  # S3
  checkEqual(estRow1[78], estRow2[78], 'Different S3 matrix width')
  checkEqual(estRow1[79], estRow2[79], 'Different S3 matrix height')
  checkEqual(estRow1[80], estRow2[80], 'Different number of S3 matrix entries')

  fp1.close()
  fp2.close()

  print('End ESTIMATE matrix dimensions comparison\n', flush=True)

  print('DONE!', flush=True)


if __name__ == "__main__":
  _main()

