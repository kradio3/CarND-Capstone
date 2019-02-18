#!/usr/bin/env python

import csv

def main():
    steer_file = "ros/src/twist_controller/steers.csv"
    mse = 0
    cnt = 0
    with open(steer_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=',')
        for row in reader:
            cnt += 1
            actual = float(row['actual'])
            proposed = float(row['proposed'])
            diff = actual - proposed
            mse += diff*diff
    mse = mse / cnt
    print('Steering performance based on %d samples = %.16f' % (cnt, mse))

if __name__ == "__main__":
    main()