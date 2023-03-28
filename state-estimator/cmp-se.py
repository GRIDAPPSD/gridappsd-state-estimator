#!/usr/bin/env python3

"""
Created on March 28, 2023

@author: Gary Black
"""

import argparse

def _main():
  parser = argparse.ArgumentParser()
  parser.add_argument("sim_dir1", help="Simulation Directory 1")
  parser.add_argument("sim_dir2", help="Simulation Directory 2")
  opts = parser.parse_args()

  simDir1 = opts.sim_dir1 + '/test_suite/'
  simDir2 = opts.sim_dir2 + '/test_suite/'

  print(simDir1)
  print(simDir2)


if __name__ == "__main__":
  _main()

