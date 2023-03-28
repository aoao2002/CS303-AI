import numba
from numba import jit
import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer
import agent
import numpy as np


@jit(nopython=True)
def ff():
    x = 0
    for i in np.arange(50000000):
        x += i
    return x


if __name__ == '__main__':
    # t = timer()
    # ff()
    # t_c = timer()
    # print(t_c - t)
    # N = 100
    # for x in range(1, 1001):
    #     y = max(15000*x/(30+4*x), 7500)
    #     plt.scatter(x, y)
    # plt.show()
    c = np.array([[1, 2, 0], [0, 1, 0]])
    print(c)
    '''
    [[1 2 0]
     [0 1 0]]
    '''
    print(np.count_nonzero(c))
