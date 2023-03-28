import numpy as np
import random
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
board_weight_2 = np.array([[-4, -2, -2, -2, -2, -2, -2, -4],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-2, -1, -1, -1, -1, -1, -1, -2],
                           [-4, -2, -2, -2, -2, -2, -2, -4]])
board_weight_3 = np.array([[1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1],
                           [1, 1, 1, 1, 1, 1, 1, 1]])

random.seed(0)


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

    # The input is the current chessboard. Chessboard is a numpy array.
    def go(self, chessboard):
        # Clear candidate_list, must do this step
        self.candidate_list.clear()
        # ==================================================================
        # Write your algorithm here
        # Here is the simplest sample:Random decision
        # ==================================================================
        self.candidate_list = self.judge_can_go(chessboard, self.color)
        if len(self.candidate_list) == 0:
            return []
        # decision algorithm
        # self.candidate_list.append(random.choice(self.candidate_list))
        self.candidate_list.append(self.worst_choice(chessboard))
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

    def judge_can_go(self, chessboard, color):
        can_go = []
        idx_none = np.where(chessboard == COLOR_NONE)
        idx_none = list(zip(idx_none[0], idx_none[1]))
        idx_white = np.where(chessboard == COLOR_WHITE)
        idx_white = list(zip(idx_white[0], idx_white[1]))
        idx_black = np.where(chessboard == COLOR_BLACK)
        idx_black = list(zip(idx_black[0], idx_black[1]))
        directions = [[1, 0], [0, 1], [-1, 0], [0, -1],
                      [-1, -1], [-1, 1], [1, -1], [1, 1]]
        if color == COLOR_WHITE:
            for white_chess in idx_white:
                for d in directions:
                    chess = (white_chess[0] + d[0], white_chess[1] + d[1])
                    while chess in idx_black:
                        chess = (chess[0] + d[0], chess[1] + d[1])
                    if chess in idx_none and (chess[0] - d[0], chess[1] - d[1]) != white_chess and chess not in can_go:
                        can_go.append(chess)
        else:
            for black_chess in idx_black:
                for d in directions:
                    chess = (black_chess[0] + d[0], black_chess[1] + d[1])
                    while chess in idx_white:
                        chess = (chess[0] + d[0], chess[1] + d[1])
                    if chess in idx_none and (chess[0] - d[0], chess[1] - d[1]) != black_chess and chess not in can_go:
                        can_go.append(chess)
        return can_go

    def map_weight(self, chessboard):
        # return sum(sum(chessboard * board_weight)) * self.color
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
        for chess_xy in self.candidate_list:
            chessboard_c = self.change_board(chessboard, chess_xy, self.color)
            score = self.Mini_max_alpha_beta(False, 3, -float('inf'), float('inf'), chessboard_c, -self.color)
            if score < min_score:
                min_score = score
                chess_min = chess_xy
        return chess_min

    def change_board(self, chessboard, chess, color):
        chessboard_c = chessboard.copy()
        directions = [[1, 0], [0, 1], [-1, 0], [0, -1],
                      [-1, -1], [-1, 1], [1, -1], [1, 1]]
        chessboard_c[chess] = color
        change_list = []
        for d in directions:
            chess_c = (chess[0] + d[0], chess[1] + d[1])
            if chess_c[0] < 0 or chess_c[0] > len(chessboard_c) - 1 or chess_c[1] < 0 or chess_c[1] > len(
                    chessboard_c[0]) - 1:
                continue
            list_c = []
            while chessboard_c[chess_c] == -color:
                list_c.append(chess_c)
                chess_c = (chess_c[0] + d[0], chess_c[1] + d[1])
                if chess_c[0] < 0 or chess_c[0] > len(chessboard_c) - 1 or chess_c[1] < 0 or chess_c[1] > len(
                        chessboard_c[0]) - 1:
                    break
            if chess_c[0] < 0 or chess_c[0] > len(chessboard_c) - 1 or chess_c[1] < 0 or chess_c[1] > len(
                    chessboard_c[0]) - 1:
                continue
            if chessboard_c[chess_c] == color:
                change_list.append(list_c)
        for list_c in change_list:
            for chess_c in list_c:
                chessboard_c[chess_c] = color
        return chessboard_c

    def Mini_max_alpha_beta(self, is_behind, depth, alpha, beta, chessboard, color):
        candidate_list = self.judge_can_go(chessboard, color)
        if depth == 0 or candidate_list == []:
            return self.evaluate(chessboard)
        if not is_behind:
            val = -float('inf')
            for chess in candidate_list:
                chessboard_c = self.change_board(chessboard, chess, color)
                val = max(val, self.Mini_max_alpha_beta(True, depth - 1, alpha, beta, chessboard_c, -color))
                alpha = max(alpha, val)
                if beta <= alpha:
                    break
            return val
        else:
            val = float('inf')
            for chess in candidate_list:
                chessboard_c = self.change_board(chessboard, chess, color)
                val = min(val, self.Mini_max_alpha_beta(False, depth - 1, alpha, beta, chessboard_c, -color))
                beta = min(beta, val)
                if beta <= alpha:
                    break
            return val