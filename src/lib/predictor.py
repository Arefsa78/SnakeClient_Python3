from copy import deepcopy

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

from src.World import World
from src.base.Math import Vector2D
from src.lib.astar import astar, ozae_kossher
from src.ClientBest import get_action as B_action
from src.lib.killer import kill


def vec2tup(vec):
    return vec.i, vec.j


def min_t(param, param1):
    return param[0] - param1[0], param[1] - param1[1]


def dir2str(d):
    d = d[1], d[0]
    if d == (1, 0):
        return 'd'
    if d == (-1, 0):
        return 'u'
    if d == (0, 1):
        return 'r'
    if d == (0, -1):
        return 'l'


def str2dir(s):
    if s == 'l':
        return 0, -1
    if s == 'r':
        return 0, 1
    if s == 'u':
        return -1, 0
    if s == 'd':
        return 1, 0


def predict_move(wm):
    board = wm.board.copy()
    self = deepcopy(wm.get_self())
    goal_pos = wm.goal_position

    result, action_kill = kill(wm)
    if result:
        return action_kill

    bin_board = make_bin_board(board, wm.goal_id)
    bin_board[self.head.i][self.head.j] = 1
    predict_next_heads(wm, bin_board)

    if bin_board[goal_pos.i][goal_pos.j] <= 0:
        goal_pos = Vector2D(1, 1)

    grid = Grid(matrix=bin_board)
    start = grid.node(self.head.j, self.head.i)
    end = grid.node(goal_pos.j, goal_pos.i)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start, end, grid)

    action = "n"
    if path:
        action = dir2str(min_t(path[1], path[0]))
    else:
        wm.goal_position = goal_pos
        action = B_action(wm)

    return action


def make_bin_board(board, goal_id):
    bin = [[1 for j in range(len(board[0]))] for i in range(len(board))]
    for i in range(len(board)):
        for j in range(len(board[i])):
            if board[i][j] == 0:
                continue
            elif board[i][j] == goal_id:
                bin[i][j] = 1
            elif 1 <= board[i][j] <= 4:
                bin[i][j] = -board[i][j]
            else:
                bin[i][j] = 0

    return bin


def display(board):
    line = ""
    for i in range(len(board)):
        for j in range(len(board[i])):
            line += str(board[i][j]) + (" " if board[i][j] < 0 else "  ")
        line += "\n"
    return line


def predict_next_heads(wm, bin_board):
    goal_pos = wm.goal_position
    op_sum = 0
    for snake in wm.snakes.values():
        if snake.id == wm.self_id:
            continue

        grid = Grid(matrix=bin_board)
        start = grid.node(snake.head.j, snake.head.i)
        end = grid.node(goal_pos.j, goal_pos.i)
        path, runs = [], 0

        if op_sum <= 200 and wm.get_self().head.dist(snake.head) <= 4:
            finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
            path, runs = finder.find_path(start, end, grid)
            op_sum += runs

        if path:
            bin_board[path[1][1]][path[1][0]] = 0
            action = ozae_kossher(bin_board, snake.head, goal_pos)
            p = str2dir(action)
            bin_board[p[0] + snake.head.i][p[1] + snake.head.j] = 0
