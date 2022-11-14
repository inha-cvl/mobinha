import numpy as np
import matplotlib.pyplot as plt
import csv
import pymap3d as pm

def main():
    infile = open('../data/hitech.csv', 'r')
    data = csv.reader(infile)

    #Baseline
    b_x, b_y = 126.891563662157, 37.5777786160887

    #Data creation
    # sample = [i**2 for i in range(10)]
    # values = np.arange(0., 20., 0.1)
    # plt.axis([0, 100, 0, 100])

    #CSV Check
    for i in data:
        pm.geodetic2enu
        res = pm.geodetic2enu(float(i[0]), float(i[1]), 0, b_y, b_x, 0)
        plt.scatter(res[0], res[1])
        plt.pause(0.001)
        input()

    # plt.show()

if __name__ == '__main__':
    main()
