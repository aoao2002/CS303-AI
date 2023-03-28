from timeit import default_timer as timer
import agent_pl
import agent
import time
import numpy as np
import platform

if __name__ == '__main__':
    agent = agent_pl.AI(8, 1, 10)
    chessboard = np.loadtxt('chess_log.txt', dtype=int)
    # chessboard = np.zeros((8, 8), dtype=int)
    # chessboard[3, 3] = 1
    # chessboard[4, 4] = 1
    # chessboard[3, 4] = -1
    # chessboard[4, 3] = -1
    t = time.time()
    agent.go(chessboard)
    t_c = time.time()
    print(t_c - t)
    print('系统:', platform.system())
