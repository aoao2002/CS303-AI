import os, copy

n = 8  # 板子尺寸（偶数）
board = [['0' for x in range(n)] for y in range(n)]
# 8个方向
dirx = [-1, 0, 1, -1, 1, -1, 0, 1]
diry = [-1, -1, -1, 0, 0, 1, 1, 1]


def InitBoard():
    if n % 2 == 0:  # 如果板子是偶数
        z = int((n - 2) / 2)
        board[z][z] = '2'
        board[n - 1 - z][z] = '1'
        board[z][n - 1 - z] = '1'
        board[n - 1 - z][n - 1 - z] = '2'


def PrintBoard():
    m = len(str(n - 1))
    for y in range(n):
        row = ''
        for x in range(n):
            row += board[y][x]
            row += ' ' * m
        print(row + ' ' + str(y))
    print()
    row = ''
    for x in range(n):
        row += str(x).zfill(m) + ' '
    print(row + '\n')


def MakeMove(board, x, y, player):  # 采取有效措施
    totctr = 0  # 对手总数
    board[y][x] = player
    for d in range(8):  # 8个方向
        ctr = 0
        for i in range(n):
            dx = x + dirx[d] * (i + 1)
            dy = y + diry[d] * (i + 1)
            if dx < 0 or dx > n - 1 or dy < 0 or dy > n - 1:
                ctr = 0;
                break
            elif board[dy][dx] == player:
                break
            elif board[dy][dx] == '0':
                ctr = 0;
                break
            else:
                ctr += 1
        for i in range(ctr):
            dx = x + dirx[d] * (i + 1)
            dy = y + diry[d] * (i + 1)
            board[dy][dx] = player
        totctr += ctr
    return (board, totctr)


def ValidMove(board, x, y, player):
    if x < 0 or x > n - 1 or y < 0 or y > n - 1:
        return False
    if board[y][x] != '0':
        return False
    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
    if totctr == 0:
        return False
    return True


minEvalBoard = -1  # min - 1
maxEvalBoard = n * n + 4 * n + 4 + 1  # max + 1


def EvalBoard(board, player):
    tot = 0
    for y in range(n):
        for x in range(n):
            if board[y][x] == player:
                if (x == 0 or x == n - 1) and (y == 0 or y == n - 1):
                    tot += 4  # corner
                elif (x == 0 or x == n - 1) or (y == 0 or y == n - 1):
                    tot += 2  # side
                else:
                    tot += 1
    return tot


# 如果没有有效的动作，则为真
def IsTerminalNode(board, player):
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                return False
    return True


def GetSortedNodes(board, player):
    sortedNodes = []
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                sortedNodes.append((boardTemp, EvalBoard(boardTemp, player)))
    sortedNodes = sorted(sortedNodes, key=lambda node: node[1], reverse=True)
    sortedNodes = [node[0] for node in sortedNodes]
    return sortedNodes


def Minimax(board, player, depth, maximizingPlayer):
    if depth == 0 or IsTerminalNode(board, player):
        return EvalBoard(board, player)
    if maximizingPlayer:
        bestValue = minEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = Minimax(boardTemp, player, depth - 1, False)
                    bestValue = max(bestValue, v)
    else:  # minimizingPlayer
        bestValue = maxEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = Minimax(boardTemp, player, depth - 1, True)
                    bestValue = min(bestValue, v)
    return bestValue


def AlphaBeta(board, player, depth, alpha, beta, maximizingPlayer):
    if depth == 0 or IsTerminalNode(board, player):
        return EvalBoard(board, player)
    if maximizingPlayer:
        v = minEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = max(v, AlphaBeta(boardTemp, player, depth - 1, alpha, beta, False))
                    alpha = max(alpha, v)
                    if beta <= alpha:
                        break  # beta cut-off
        return v
    else:  # 最小化播放器
        v = maxEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = min(v, AlphaBeta(boardTemp, player, depth - 1, alpha, beta, True))
                    beta = min(beta, v)
                    if beta <= alpha:
                        break  # alpha cut-off
        return v


def AlphaBetaSN(board, player, depth, alpha, beta, maximizingPlayer):
    if depth == 0 or IsTerminalNode(board, player):
        return EvalBoard(board, player)
    sortedNodes = GetSortedNodes(board, player)
    if maximizingPlayer:
        v = minEvalBoard
        for boardTemp in sortedNodes:
            v = max(v, AlphaBetaSN(boardTemp, player, depth - 1, alpha, beta, False))
            alpha = max(alpha, v)
            if beta <= alpha:
                break  # beta cut-off
        return v
    else:  # minimizingPlayer
        v = maxEvalBoard
        for boardTemp in sortedNodes:
            v = min(v, AlphaBetaSN(boardTemp, player, depth - 1, alpha, beta, True))
            beta = min(beta, v)
            if beta <= alpha:
                break  # alpha cut-off
        return v


def Negamax(board, player, depth, color):
    if depth == 0 or IsTerminalNode(board, player):
        return color * EvalBoard(board, player)
    bestValue = minEvalBoard
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                v = -Negamax(boardTemp, player, depth - 1, -color)
                bestValue = max(bestValue, v)
    return bestValue


def NegamaxAB(board, player, depth, alpha, beta, color):
    if depth == 0 or IsTerminalNode(board, player):
        return color * EvalBoard(board, player)
    bestValue = minEvalBoard
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                v = -NegamaxAB(boardTemp, player, depth - 1, -beta, -alpha, -color)
                bestValue = max(bestValue, v)
                alpha = max(alpha, v)
                if alpha >= beta:
                    break
    return bestValue


def NegamaxABSN(board, player, depth, alpha, beta, color):
    if depth == 0 or IsTerminalNode(board, player):
        return color * EvalBoard(board, player)
    sortedNodes = GetSortedNodes(board, player)
    bestValue = minEvalBoard
    for boardTemp in sortedNodes:
        v = -NegamaxABSN(boardTemp, player, depth - 1, -beta, -alpha, -color)
        bestValue = max(bestValue, v)
        alpha = max(alpha, v)
        if alpha >= beta:
            break
    return bestValue


def Negascout(board, player, depth, alpha, beta, color):
    if depth == 0 or IsTerminalNode(board, player):
        return color * EvalBoard(board, player)
    firstChild = True
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                if not firstChild:
                    score = -Negascout(boardTemp, player, depth - 1, -alpha - 1, -alpha, -color)
                    if alpha < score and score < beta:
                        score = -Negascout(boardTemp, player, depth - 1, -beta, -score, -color)
                else:
                    firstChild = False
                    score = -Negascout(boardTemp, player, depth - 1, -beta, -alpha, -color)
                alpha = max(alpha, score)
                if alpha >= beta:
                    break
    return alpha


def NegascoutSN(board, player, depth, alpha, beta, color):
    if depth == 0 or IsTerminalNode(board, player):
        return color * EvalBoard(board, player)
    sortedNodes = GetSortedNodes(board, player)
    firstChild = True
    for boardTemp in sortedNodes:
        if not firstChild:
            score = -NegascoutSN(boardTemp, player, depth - 1, -alpha - 1, -alpha, -color)
            if alpha < score and score < beta:
                score = -NegascoutSN(boardTemp, player, depth - 1, -beta, -score, -color)
        else:
            firstChild = False
            score = -NegascoutSN(boardTemp, player, depth - 1, -beta, -alpha, -color)
        alpha = max(alpha, score)
        if alpha >= beta:
            break
    return alpha


def BestMove(board, player):
    maxPoints = 0
    mx = -1;
    my = -1
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                if opt == 0:
                    points = EvalBoard(boardTemp, player)
                elif opt == 1:
                    points = Minimax(boardTemp, player, depth, True)
                elif opt == 2:
                    points = AlphaBeta(board, player, depth, minEvalBoard, maxEvalBoard, True)
                elif opt == 3:
                    points = Negamax(boardTemp, player, depth, 1)
                elif opt == 4:
                    points = NegamaxAB(boardTemp, player, depth, minEvalBoard, maxEvalBoard, 1)
                elif opt == 5:
                    points = Negascout(boardTemp, player, depth, minEvalBoard, maxEvalBoard, 1)
                elif opt == 6:
                    points = AlphaBetaSN(board, player, depth, minEvalBoard, maxEvalBoard, True)
                elif opt == 7:
                    points = NegamaxABSN(boardTemp, player, depth, minEvalBoard, maxEvalBoard, 1)
                elif opt == 8:
                    points = NegascoutSN(boardTemp, player, depth, minEvalBoard, maxEvalBoard, 1)
                if points > maxPoints:
                    maxPoints = points
                    mx = x;
                    my = y
    return (mx, my)


print('REVERSI OTHELLO董事会游戏')
print('0: 评估板')
print('1: 极小值')
print('2: 带有Alpha-Beta修剪的Minimax')
print('3: negamax')
print('4: Negamax与Alpha-Beta修剪')
print('5: Negascout（主要变化搜索)')
print('6: 带有排序节点的Alpha-Beta修剪的Minimax')
print('7: N带排序节点的Alpha-Beta修剪的negamax')
print('8: Negascout（主要变异搜索）w排序的节点')
opt = int(input('SelectAIAlgorithm '))
if opt > 0 and opt < 9:
    depth = 4
    depthStr = input('选择搜索深度（默认值：4）: ')
    if depthStr != '': depth = int(depth)
print('\n1: User 2: AI (只需按Enter键即可退出！)')
InitBoard()
while True:
    for p in range(2):
        print()
        PrintBoard()
        player = str(p + 1)
        print('玩家： ' + player)
        if IsTerminalNode(board, player):
            print('玩家无法玩！游戏结束了！')
            print('评分用户： ' + str(EvalBoard(board, '1')))
            print('评分 AI: ' + str(EvalBoard(board, '2')))
            os._exit(0)
        if player == '1':  # user's turn
            while True:
                xy = input('X Y: ')
                if xy == '': os._exit(0)
                (x, y) = xy.split()
                x = int(x);
                y = int(y)
                if ValidMove(board, x, y, player):
                    (board, totctr) = MakeMove(board, x, y, player)
                    print('# of pieces taken: ' + str(totctr))
                    break
                else:
                    print('无效移动，请重新尝试！')
        else:  # AI's turn
            (x, y) = BestMove(board, player)
            if not (x == -1 and y == -1):
                (board, totctr) = MakeMove(board, x, y, player)
                print('AI played (X Y): ' + str(x) + ' ' + str(y))
                print('# of pieces taken: ' + str(totctr))