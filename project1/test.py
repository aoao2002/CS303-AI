from timeit import default_timer as timer
import agent
import agenta
import agentb
import agentc
import agentd
import agente
import agentf
import agentg
import agenth
import agenti
import agent_pl
import time
import numpy as np
import platform


def play(agent1, agent2):
    chessboard = np.zeros((8, 8), dtype=int)
    chessboard[3, 3] = 1
    chessboard[4, 4] = 1
    chessboard[3, 4] = -1
    chessboard[4, 3] = -1
    print("Game begin!")
    while np.count_nonzero(chessboard) < 64:
        t = time.time()
        agent1.go(chessboard)
        if agent1.candidate_list:
            chess = agent1.candidate_list[-1]
            t_c = time.time()
            print('AI1 time:')
            print(t_c - t)
            if t_c - t > 5:
                print('AI1 Timeout')
                chess = agent1.candidate_list[-2]
            chessboard = agent.change_board(chessboard, chess, -1)
            print(chessboard)
        t = time.time()
        agent2.go(chessboard)
        if agent2.candidate_list:
            chess = agent2.candidate_list[-1]
            t_c = time.time()
            print('AI2 time:')
            print(t_c - t)
            if t_c - t > 5:
                print('AI2 Timeout')
                chess = agent2.candidate_list[-2]
            chessboard = agent.change_board(chessboard, chess, 1)
            print(chessboard)
        if not agent1.candidate_list and not agent2.candidate_list:
            break
    if np.sum(chessboard) > 0:
        print("black win!")
    else:
        print("white win!")


if __name__ == '__main__':

    agenta = agentd.AI(8, -1, 10)
    agentb = agentc.AI(8, 1, 10)
    play(agenta, agentb)
