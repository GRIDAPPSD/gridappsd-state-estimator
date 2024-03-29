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
FlagPerDiff = 2.0
FlagAbsDiff = 0.1

def checkDiff(value1, value2, message, absFlag=True, printFlag=False):
  value1 = float(value1)
  value2 = float(value2)
  equalFlag = True

  if value1 == value2:
    if printFlag:
      print(message + ' values are equal', flush=True)
  else:
    if abs(value1)<NearZero and abs(value2)<NearZero:
      if printFlag:
        print(message + ' values are both within computational precision of zero and considered equal', flush=True)
    else:
      equalFlag = False
      absdiff = abs(value1 - value2)
      perdiff = abs(100 * absdiff/((value1 + value2)/2))
      if not absFlag and perdiff>FlagPerDiff:
        print('*** ' + message + ' values % diff: ' + str(round(perdiff, 4)), flush=True)
      elif perdiff>FlagPerDiff and absdiff>FlagAbsDiff:
        print('*** ' + message + ' values abs diff: ' + str(round(absdiff, 4)) + ', % diff: ' + str(round(perdiff, 4)), flush=True)
      elif printFlag:
        print(message + ' values abs diff: ' + str(round(absdiff, 4)) + ', % diff: ' + str(round(perdiff, 4)), flush=True)

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

  print('Begin INITALIZATION accuracy comparison:', flush=True)

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

  print('End INITALIZATION accuracy comparison\n', flush=True)

  print('Begin ESTIMATE accuracy comparison:', flush=True)

  fp1 = open(simDir1 + 'est_accy.csv', 'r')
  fp2 = open(simDir2 + 'est_accy.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  estHeader1 = next(reader1)
  estHeader2 = next(reader2)

  if estHeader1 != estHeader2:
    print('Mismatched est_accy.csv headers being compared--exiting!\n', flush=True)
    exit()

  # to get index numbers
  #ctr = 0
  #for item in estHeader1:
  #  print (str(ctr) + ': ' + item, flush=True)
  #  ctr += 1

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

      checkDiff(estRow1[149], estRow2[149], 'Measurement vmag min')
      checkDiff(estRow1[150], estRow2[150], 'Measurement vmag max')
      checkDiff(estRow1[151], estRow2[151], 'Measurement vmag mean')

      checkDiff(estRow1[152], estRow2[152], 'Estimate vmag min')
      checkDiff(estRow1[153], estRow2[153], 'Estimate vmag max')
      checkDiff(estRow1[154], estRow2[154], 'Estimate vmag mean')

      checkDiff(estRow1[155], estRow2[155], 'Estimate vmag percent error')
      print('Estimate vmag percent error for timestamp sim1: ' + str(round(float(estRow1[155]), 4)) + ', sim2: ' + str(round(float(estRow2[155]), 4)), flush=True)

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
  print('\nMean estimate vmag percent error over all timestamps sim1: ' + str(round(meanPerErr1, 4)) + ', sim2: ' + str(round(meanPerErr2, 4)), flush=True)

  print('End ESTIMATE accuracy comparison\n', flush=True)

  print('Begin ESTIMATE performance comparison:', flush=True)

  fp1 = open(simDir1 + 'est_perf.csv', 'r')
  fp2 = open(simDir2 + 'est_perf.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  estHeader1 = next(reader1)
  estHeader2 = next(reader2)

  if estHeader1 != estHeader2:
    print('Mismatched est_perf.csv headers being compared--exiting!\n', flush=True)
    exit()

  itCount = 0
  sumEstTime1 = 0.0
  sumEstTime2 = 0.0

  while True:
    try:
      # if either of these next calls fails, bail from comparison loop
      estRow1 = next(reader1)
      estRow2 = next(reader2)

      itCount += 1
      print('timestamp #' + str(itCount) + ': ' + estRow1[0], flush=True)

      checkDiff(estRow1[1], estRow2[1], 'Supd inverse time')
      checkDiff(estRow1[2], estRow2[2], 'Supd inverse virtual memory')
      checkDiff(estRow1[3], estRow2[3], 'Kupd multiply time')
      checkDiff(estRow1[4], estRow2[4], 'Kupd multiply virtual memory')
      checkDiff(estRow1[5], estRow2[5], 'Total estimate time')
      checkDiff(estRow1[6], estRow2[6], 'Total estimate virtual memory')

      sumEstTime1 += float(estRow1[5])
      sumEstTime2 += float(estRow2[5])

    except:
      break

  fp1.close()
  fp2.close()

  meanEstTime1 = sumEstTime1/itCount
  meanEstTime2 = sumEstTime2/itCount
  print('\nMean total estimate time sim1: ' + str(round(meanEstTime1, 4)) + ', sim2: ' + str(round(meanEstTime2, 4)), flush=True)
  checkDiff(meanEstTime1, meanEstTime2, 'Mean total estimate time', False, True)

  print('End ESTIMATE performance comparison\n', flush=True)

  print('Begin ESTIMATE vs. measurement for ' + opts.sim_dir1 + ':', flush=True)

  fpM = open(simDir1 + 'meas_vmagpu.csv', 'r')
  fpE = open(simDir1 + 'est_vmagpu.csv', 'r')

  readerM = csv.reader(fpM)
  readerE = csv.reader(fpE)

  headerM = next(readerM)
  headerE = next(readerE)

  if headerM != headerE:
    print('Mismatched meas_vmagpu.csv and est_vmagpu.csv headers being compared--exiting!\n', flush=True)
    exit()

  numNodes = len(headerM)-1
  itCount = 0
  sumCount = 0
  sumMeasVMag = 0.0
  sumEstVMag = 0.0

  while True:
    try:
      # if either of these next calls fails, bail from comparison loop
      rowM = next(readerM)
      rowE = next(readerE)

      itCount += 1
      print('timestamp #' + str(itCount) + ': ' + rowM[0], flush=True)

      for inode in range(1, numNodes+1):
        checkDiff(rowM[inode], rowE[inode],
                  headerM[inode] + ' estimate vs. measurement')
        sumMeasVMag += float(rowM[inode])
        sumEstVMag += float(rowE[inode])

      sumCount += numNodes

    except:
      break

  fpM.close()
  fpE.close()

  meanMeasVMag = sumMeasVMag/sumCount
  meanEstVMag = sumEstVMag/sumCount
  print('\nMean measurement vmag: ' + str(round(meanMeasVMag, 4)) + ', estimate vmag: ' + str(round(meanEstVMag, 4)), flush=True)
  checkDiff(meanMeasVMag, meanEstVMag, 'Mean measurement vs. estimate vmag over all nodes and timestamps', False, True)

  print('End ESTIMATE vs. measurement for ' + opts.sim_dir1 + '\n', flush=True)

  print('Begin ESTIMATE vs. measurement for ' + opts.sim_dir2 + ':', flush=True)

  fpM = open(simDir2 + 'meas_vmagpu.csv', 'r')
  fpE = open(simDir2 + 'est_vmagpu.csv', 'r')

  readerM = csv.reader(fpM)
  readerE = csv.reader(fpE)

  headerM = next(readerM)
  headerE = next(readerE)

  if headerM != headerE:
    print('Mismatched meas_vmagpu.csv and est_vmagpu.csv headers being compared--exiting!\n', flush=True)
    exit()

  numNodes = len(headerM)-1
  itCount = 0
  sumCount = 0
  sumMeasVMag = 0.0
  sumEstVMag = 0.0

  while True:
    try:
      # if either of these next calls fails, bail from comparison loop
      rowM = next(readerM)
      rowE = next(readerE)

      itCount += 1
      print('timestamp #' + str(itCount) + ': ' + rowM[0], flush=True)

      for inode in range(1, numNodes+1):
        checkDiff(rowM[inode], rowE[inode],
                  headerM[inode] + ' estimate vs. measurement')
        sumMeasVMag += float(rowM[inode])
        sumEstVMag += float(rowE[inode])

      sumCount += numNodes

    except:
      break

  fpM.close()
  fpE.close()

  meanMeasVMag = sumMeasVMag/sumCount
  meanEstVMag = sumEstVMag/sumCount
  print('\nMean measurement vmag: ' + str(round(meanMeasVMag, 4)) + ', estimate vmag: ' + str(round(meanEstVMag, 4)), flush=True)
  checkDiff(meanMeasVMag, meanEstVMag, 'Mean measurement vs. estimate vmag over all nodes and timestamps', False, True)

  print('End ESTIMATE vs. measurement for ' + opts.sim_dir2 + '\n', flush=True)

  print('Begin ESTIMATE voltage magnitude comparison:', flush=True)

  fp1 = open(simDir1 + 'est_vmagpu.csv', 'r')
  fp2 = open(simDir2 + 'est_vmagpu.csv', 'r')

  reader1 = csv.reader(fp1)
  reader2 = csv.reader(fp2)

  estHeader1 = next(reader1)
  estHeader2 = next(reader2)

  if estHeader1 != estHeader2:
    print('Mismatched est_vmagpu.csv headers being compared--exiting!\n', flush=True)
    exit()

  numNodes = len(estHeader1)-1
  itCount = 0
  sumCount = 0
  sumEstVMag1 = 0.0
  sumEstVMag2 = 0.0

  while True:
    try:
      # if either of these next calls fails, bail from comparison loop
      estRow1 = next(reader1)
      estRow2 = next(reader2)

      itCount += 1
      print('timestamp #' + str(itCount) + ': ' + estRow1[0], flush=True)

      for inode in range(1, numNodes+1):
        checkDiff(estRow1[inode], estRow2[inode],
                  estHeader1[inode] + ' estimate')
        sumEstVMag1 += float(estRow1[inode])
        sumEstVMag2 += float(estRow2[inode])

      sumCount += numNodes

    except:
      break

  fp1.close()
  fp2.close()

  meanEstVMag1 = sumEstVMag1/sumCount
  meanEstVMag2 = sumEstVMag2/sumCount
  print('\nMean estimate vmag sim1: ' + str(round(meanEstVMag1, 4)) + ', sim2: ' + str(round(meanEstVMag2, 4)), flush=True)
  checkDiff(meanEstVMag1, meanEstVMag2, 'Mean estimate vmag', False, True)

  print('End ESTIMATE voltage magnitude comparison\n', flush=True)


if __name__ == "__main__":
  _main()

