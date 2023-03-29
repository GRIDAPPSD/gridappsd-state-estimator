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

# print a warning flag for difference values greater than this
FlagPerDiff = 4.0
FlagAbsDiff = 0.1

def checkDiff(value1, value2, message):
  value1 = float(value1)
  value2 = float(value2)
  equalFlag = True

  if value1 == value2:
    #print(message + ' values are equal', flush=True)
    pass
  else:
    if abs(value1)<NearZero and abs(value2)<NearZero:
      #print(message + ' values are both within computational precision of zero and considered equal', flush=True)
      pass
    else:
      equalFlag = False
      absdiff = abs(value1 - value2)
      perdiff = abs(100 * absdiff/((value1 + value2)/2))
      if perdiff>FlagPerDiff and absdiff>FlagAbsDiff:
        print('*** ' + message + ' values abs diff: ' + str(round(absdiff, 4)) + ', % diff: ' + str(round(perdiff, 4)), flush=True)
      #else:
      #  print(message + ' values abs diff: ' + str(round(absdiff, 4)) + ', % diff: ' + str(round(perdiff, 4)), flush=True)

  return equalFlag


def checkEqual(value1, value2, message):
  equalFlag = True
  if value1 != value2:
    equalFlag = False
    print(message + ', sim 1: ' + str(value1) + ', sim 2: ' + str(value2), flush=True)

  return equalFlag


def checkSizes(matName, startIndex, row1, row2):
  equalFlag = True
  if row1[startIndex] != row2[startIndex]:
    equalFlag = False
    print(matName + ' matrix width mismatch, sim 1: ' + str(row1[startIndex]) + ', sim 2: ' + str(row2[startIndex]), flush=True)

  if row1[startIndex+1] != row2[startIndex+1]:
    equalFlag = False
    print(matName + ' matrix height mismatch, sim 1: ' + str(row1[startIndex+1]) + ', sim 2: ' + str(row2[startIndex+1]), flush=True)

  if row1[startIndex+2] != row2[startIndex+2]:
    equalFlag = False
    print(matName + ' matrix number of entries mismatch, sim 1: ' + str(row1[startIndex+2]) + ', sim 2: ' + str(row2[startIndex+2]), flush=True)

    return equalFlag


def checkStats(matName, startIndex, row1, row2):
  checkDiff(row1[startIndex], row2[startIndex], matName + ' matrix min')
  checkDiff(row1[startIndex+1], row2[startIndex+1], matName + ' matrix max')
  checkDiff(row1[startIndex+2], row2[startIndex+2], matName + ' matrix mean')


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

  initHeader1 = next(reader1)
  initHeader2 = next(reader2)

  if initHeader1 != initHeader2:
    print('Mismatched init_accy.csv headers being compared--exiting!\n', flush=True)
    exit()

  initRow1 = next(reader1)
  initRow2 = next(reader2)

  # init_accy.csv entries: nodeqty,xqty,zqty,Jacobian_elements,Yphys_scaled_terms,F_width,F_height,F_entries,F_min,F_max,F_mean,eyex_width,eyex_height,eyex_entries,eyex_min,eyex_max,eyex_mean,R_width,R_height,R_entries,R_min,R_max,R_mean

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

  checkSizes('F', 5, initRow1, initRow2)
  checkStats('F', 8, initRow1, initRow2)

  checkSizes('eyex', 11, initRow1, initRow2)
  checkStats('eyex', 14, initRow1, initRow2)

  checkSizes('R', 17, initRow1, initRow2)
  checkStats('R', 20, initRow1, initRow2)

  # tests of checkDiff
  #checkDiff(10.0, 8.0, 'Test')
  #checkDiff(1e-13, 1e-16, 'Zero')
  #checkDiff(1e-8, 1e-16, 'Low')

  fp1.close()
  fp2.close()

  print('End INITALIZATION comparison\n', flush=True)

  print('Begin ESTIMATE comparison:', flush=True)

  fp1 = open(simDir1 + 'est_accy.csv', 'r')
  fp2 = open(simDir2 + 'est_accy.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  estHeader1 = next(reader1)
  estHeader2 = next(reader2)

  if estHeader1 != estHeader2:
    print('Mismatched est_accy.csv headers being compared--exiting!\n', flush=True)
    exit()

  estRow1 = next(reader1)
  estRow2 = next(reader2)

  # to get index numbers
  #ct = 0
  #for item in estHeader1:
  #  print (str(ct) + ': ' + item + ', ' + str(estRow1[ct]), flush=True)
  #  ct += 1

  itCount = 0
  sumPerErr1 = 0.0
  sumPerErr2 = 0.0

  while True:
    try:
      # if either of these next calls fails, bail from comparison loop
      estRow1 = next(reader1)
      estRow2 = next(reader2)

      itCount += 1
      print('timestamp #' + str(itCount) + ': ' + estRow1[0], flush=True)

      checkSizes('P', 1, estRow1, estRow2)
      checkStats('P', 4, estRow1, estRow2)

      checkSizes('P1', 7, estRow1, estRow2)
      checkStats('P1', 10, estRow1, estRow2)

      checkSizes('P2', 13, estRow1, estRow2)
      checkStats('P2', 16, estRow1, estRow2)

      checkSizes('P3', 19, estRow1, estRow2)
      checkStats('P3', 22, estRow1, estRow2)

      checkSizes('Q', 25, estRow1, estRow2)
      checkStats('Q', 28, estRow1, estRow2)

      checkSizes('Ppre', 31, estRow1, estRow2)
      checkStats('Ppre', 34, estRow1, estRow2)

      checkSizes('x', 37, estRow1, estRow2)
      checkStats('x', 40, estRow1, estRow2)

      checkSizes('xpre', 43, estRow1, estRow2)
      checkStats('xpre', 46, estRow1, estRow2)

      checkSizes('J', 49, estRow1, estRow2)
      checkStats('J', 52, estRow1, estRow2)

      checkSizes('S1', 55, estRow1, estRow2)
      checkStats('S1', 58, estRow1, estRow2)

      checkSizes('S2', 61, estRow1, estRow2)
      checkStats('S2', 64, estRow1, estRow2)

      checkSizes('S3', 67, estRow1, estRow2)
      checkStats('S3', 70, estRow1, estRow2)

      checkStats('R', 73, estRow1, estRow2)

      checkSizes('Supd', 76, estRow1, estRow2)
      checkStats('Supd', 79, estRow1, estRow2)

      checkDiff(estRow1[82], estRow2[82], 'Supd matrix condition number estimate')

      checkSizes('K3', 83, estRow1, estRow2)
      checkStats('K3', 86, estRow1, estRow2)

      checkSizes('K2', 89, estRow1, estRow2)
      checkStats('K2', 92, estRow1, estRow2)

      checkSizes('Kupd', 95, estRow1, estRow2)
      checkStats('Kupd', 98, estRow1, estRow2)
      #print('Kupd min 1: ' + str(estRow1[98]), flush=True)
      #print('Kupd min 2: ' + str(estRow2[98]), flush=True)
      #print('Kupd max 1: ' + str(estRow1[99]), flush=True)
      #print('Kupd max 2: ' + str(estRow2[99]), flush=True)
      #print('Kupd mean 1: ' + str(estRow1[100]), flush=True)
      #print('Kupd mean 2: ' + str(estRow2[100]), flush=True)

      checkSizes('z', 101, estRow1, estRow2)
      checkStats('z', 104, estRow1, estRow2)

      checkSizes('h', 107, estRow1, estRow2)
      checkStats('h', 110, estRow1, estRow2)

      checkSizes('yupd', 113, estRow1, estRow2)
      checkStats('yupd', 116, estRow1, estRow2)

      checkSizes('x1', 119, estRow1, estRow2)
      checkStats('x1', 122, estRow1, estRow2)

      checkSizes('xupd', 125, estRow1, estRow2)
      checkStats('xupd', 128, estRow1, estRow2)

      checkSizes('P4', 131, estRow1, estRow2)
      checkStats('P4', 134, estRow1, estRow2)

      checkSizes('P5', 137, estRow1, estRow2)
      checkStats('P5', 140, estRow1, estRow2)

      checkSizes('Pupd', 143, estRow1, estRow2)
      checkStats('Pupd', 146, estRow1, estRow2)

      checkDiff(estRow1[149], estRow2[149], 'Measurement voltage magnitude min')
      checkDiff(estRow1[150], estRow2[150], 'Measurement voltage magnitude max')
      checkDiff(estRow1[151], estRow2[151], 'Measurement voltage magnitude mean')

      checkDiff(estRow1[152], estRow2[152], 'Estimate voltage magnitude min')
      checkDiff(estRow1[153], estRow2[153], 'Estimate voltage magnitude max')
      checkDiff(estRow1[154], estRow2[154], 'Estimate voltage magnitude mean')

      checkDiff(estRow1[155], estRow2[155], 'Estimate voltage magnitude percent error')

      sumPerErr1 += float(estRow1[155])
      sumPerErr2 += float(estRow2[155])

      # break after first timestamp for debugging
      #if itCount == 1:
      #  break

    except:
      break

  fp1.close()
  fp2.close()

  meanPerErr1 = sumPerErr1/itCount
  meanPerErr2 = sumPerErr2/itCount
  print('\nMean estimate voltage magnitude percent error sim1: ' + str(round(meanPerErr1, 4)) + ', sim2: ' + str(round(meanPerErr2, 4)), flush=True)

  print('End ESTIMATE comparison\n', flush=True)

  print('DONE!', flush=True)


if __name__ == "__main__":
  _main()

