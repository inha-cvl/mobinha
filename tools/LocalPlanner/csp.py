import cubic_spline_planner
import numpy as np

def test():
    x = [i for i in range(0,100)]
    y = [i for i in range(0,200, 2)]

    csp = cubic_spline_planner.Spline2D(x,y)
    s = np.arange(0, csp.s[-1], 0.1)
    print(s[2])


if __name__ == '__main__':
    test()
