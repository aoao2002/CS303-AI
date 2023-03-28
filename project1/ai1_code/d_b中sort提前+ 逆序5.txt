import numpy as np
import random
from numba import jit
import time

COLOR_BLACK = -1
COLOR_WHITE = 1
COLOR_NONE = 0

board_weight = np.array([[500, -25, 10, 5, 5, 10, -25, 500],
                         [-25, -45, 1, 1, 1, 1, -45, -25],
                         [10, 1, 3, 2, 2, 3, 1, 10],
                         [5, 1, 2, 1, 1, 2, 1, 5],
                         [5, 1, 2, 1, 1, 2, 1, 5],
                         [10, 1, 3, 2, 2, 3, 1, 10],
                         [-25, -45, 1, 1, 1, 1, -45, -25],
                         [500, -25, 10, 5, 5, 10, -25, 500]])
board_weight_3 = np.array([[8, 1, 1, 1, 1, 1, 1, 8],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [8, 1, 1, 1, 1, 1, 1, 8]])
random.seed(0)


@jit(nopython=True)
def change_board(chessboard, chess, color):
    chessboard_c = chessboard.copy()
    directions = np.array([[1, 0], [0, 1], [-1, 0], [0, -1],
                           [-1, -1], [-1, 1], [1, -1], [1, 1]])
    chessboard_c[chess] = color
    change_list = []
    for d in directions:
        chess_c = (chess[0] + d[0], chess[1] + d[1])
        if is_not_valid(chess_c, chessboard_c):
            continue
        list_c = []
        while chessboard_c[chess_c] == -color:
            list_c.append(chess_c)
            chess_c = (chess_c[0] + d[0], chess_c[1] + d[1])
            if is_not_valid(chess_c, chessboard_c):
                break
        if is_not_valid(chess_c, chessboard_c):
            continue
        if chessboard_c[chess_c] == color:
            change_list.append(list_c)
    for list_c in change_list:
        for chess_c in list_c:
            chessboard_c[chess_c] = color
    return chessboard_c


@jit(cache=True, inline='always')
def is_not_valid(chess_c, chessboard_c):
    bol = False
    if chess_c[0] < 0 or chess_c[0] > len(chessboard_c) - 1 or chess_c[1] < 0 or chess_c[1] > len(
            chessboard_c[0]) - 1:
        bol = True
    return bol


@jit(nopython=True)
def can_move(chessboard, color, index):
    directions = np.array([[1, 0], [0, 1], [-1, 0], [0, -1],
                           [-1, -1], [-1, 1], [1, -1], [1, 1]])
    for i, neighbour in enumerate(index + directions):
        if is_not_valid(neighbour, chessboard) or chessboard[(neighbour[0], neighbour[1])] != -color:
            continue
        while not is_not_valid(neighbour, chessboard):
            if chessboard[(neighbour[0], neighbour[1])] == color:
                return True
            elif chessboard[(neighbour[0], neighbour[1])] == COLOR_NONE:
                break
            neighbour += directions[i]
    return False


@jit(nopython=True)
def judge_can_go(chessboard, color):
    can_go = []
    idx_none = np.argwhere(chessboard == COLOR_NONE)
    for index in idx_none:
        if can_move(chessboard, color, index):
            can_go.append(index)
    return can_go


# don't change the class name
class AI(object):
    # chessboard_size, color, time_out passed from agent
    def __init__(self, chessboard_size, color, time_out):
        self.chessboard_size = chessboard_size
        # You are white or black
        self.color = color
        # the max time you should use, your algorithm's run time must not exceed the time limit.
        self.time_out = time_out
        # You need to add your decision to your candidate_list.
        # The system will get the end of your candidate_list as your decision.
        self.candidate_list = []
        self.t = 0

    # The input is the current chessboard. Chessboard is a numpy array.
    def go(self, chessboard):
        # Clear candidate_list, must do this step
        self.t = time.time()
        self.candidate_list.clear()
        # ==================================================================
        # Write your algorithm here
        # Here is the simplest sample:Random decision
        # ==================================================================
        self.candidate_list = judge_can_go(chessboard, self.color)
        if len(self.candidate_list) == 0:
            return []
        # decision algorithm
        # self.candidate_list.append(random.choice(self.candidate_list))
        a = []
        for i in self.candidate_list:
            a.append((i.tolist()[0], i.tolist()[1]))
        self.candidate_list = a
        self.candidate_list = self.Sort(chessboard)
        self.candidate_list.append(self.worst_choice(chessboard))
        # print(self.candidate_list)
        # ==================================================================
        return self.candidate_list
        # ==============Find new pos========================================
        # Make sure that the position of your decision on the chess board is empty.
        # If not, the system will return error.
        # Add your decision into candidate_list, Records the chessboard
        # You need to add all the positions which are valid
        # candidate_list example: [(3,3),(4,4)]
        # You need append your decision at the end of the candidate_list,
        # candidate_list example: [(3,3),(4,4),(4,4)]
        # we will pick the last element of the candidate_list as the position you choose.
        # In above example, we will pick (4,4) as your decision.
        # If there is no valid position, you must return an empty list.

    # find the position that we can go in the reversi chessboard

    def map_weight(self, chessboard):
        # return sum(sum(chessboard * board_weight)) * self.color
        if np.count_nonzero(chessboard) > 40:
            return sum(sum(chessboard * board_weight_3 * self.color))
        return sum(sum(chessboard * board_weight * self.color))

    # def stable_factor(self, chessboard):
    #     val = 0
    #
    #     return val

    # def action_weight(self, chessboard):
    #     return len(self.judge_can_go(chessboard, self.color))-len(self.judge_can_go(chessboard, -self.color))

    def evaluate(self, chessboard):
        return self.map_weight(chessboard)
        # val = 1000000000
        # chess_choice = random.choice(self.candidate_list)
        # for chess in self.candidate_list:
        #     chessboard_c = chessboard.copy()
        #     chessboard_c[chess] = self.color
        #     val_c = self.map_weight(chessboard_c)
        #     if val_c < val:
        #         chess_choice = chess
        #         val = val_c
        # self.candidate_list.append(chess_choice)

    def worst_choice(self, chessboard):
        min_score = float('inf')
        chess_min = (0, 0)
        depth = 5
        # t = time.time()
        if len(self.candidate_list) > 7:
            depth = 4
        for chess_xy in self.candidate_list:
            chess_xy = (chess_xy[0], chess_xy[1])
            chessboard_c = change_board(chessboard, chess_xy, self.color)
            score = self.Mini_max_alpha_beta(False, depth, -float('inf'), float('inf'), chessboard_c, -self.color)
            if score < min_score:
                min_score = score
                chess_min = chess_xy
            if (time.time() - self.t) > 4:
                return chess_min
        return chess_min

    def Mini_max_alpha_beta(self, is_behind, depth, alpha, beta, chessboard, color):
        candidate_list = judge_can_go(chessboard, color)
        if depth == 0 or candidate_list == []:
            return self.evaluate(chessboard)
        if not is_behind:
            val = -float('inf')
            for chess in candidate_list:
                chess = (chess[0], chess[1])
                chessboard_c = change_board(chessboard, chess, color)
                val = max(val, self.Mini_max_alpha_beta(True, depth - 1, alpha, beta, chessboard_c, -color))
                alpha = max(alpha, val)
                if beta <= alpha:
                    break
            return val
        else:
            val = float('inf')
            for chess in candidate_list:
                chess = (chess[0], chess[1])
                chessboard_c = change_board(chessboard, chess, color)
                val = min(val, self.Mini_max_alpha_beta(False, depth - 1, alpha, beta, chessboard_c, -color))
                beta = min(beta, val)
                if beta <= alpha:
                    break
            return val

    def Sort(self, chessboard):
        candidate_list = []
        for chess in self.candidate_list:
            chessboard_c = change_board(chessboard, chess, self.color)
            score = self.Mini_max_alpha_beta(False, 2, -float('inf'), float('inf'), chessboard_c, -self.color)
            candidate_list.append((chess, score))
        candidate_list = [node[0] for node in sorted(candidate_list, key=lambda x: x[1])]
        candidate_list.reverse()
        return candidate_list
