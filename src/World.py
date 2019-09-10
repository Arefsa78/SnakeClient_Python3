from src.base.Math import *


class Snake:
    def __init__(self, id):
        self.id = id
        self.head = None
        self.body = []
        self.name = ''

    def get_body(self):
        return self.body

    def add_body(self, pos):
        self.body.append(pos)

    def get_head(self):
        return self.head

    def set_head(self, pos):
        self.head = pos

    def get_id(self):
        return self.id

    def reset(self, name):
        self.head = None
        self.body.clear()
        self.name = name


class World:
    def __init__(self):
        self.board = None
        self.cycle = None
        self.self_id = None
        self.goal_id = None
        self.goal_position = None
        self.snakes = {}
        for s in range(1, 5):
            self.snakes[s] = Snake(s)
        self.walls = []

    def set_id(self, self_id, goal_id):
        self.self_id = self_id
        self.goal_id = goal_id

    def update(self, message):
        self.board = message.world['board']
        self.cycle = message.cycle
        self.walls.clear()
        n = 0
        for s in self.snakes:
            self.snakes[s].reset(list(message.score.keys())[n])
            n += 1

        for i in range(len(self.board)):
            for j in range(len(self.board[i])):
                if self.board[i][j] == self.goal_id:
                    self.goal_position = Vector2D(i, j)
                elif self.board[i][j] > 0:
                    self.snakes[self.board[i][j]].add_body(Vector2D(i, j))
                elif self.board[i][j] == -1:
                    self.walls.append(Vector2D(i, j))

        for s in self.snakes:
            id = message.name_id[self.snakes[s].name]
            self.snakes[id].set_head(Vector2D(message.world['heads'][self.snakes[s].name][0],
                                              message.world['heads'][self.snakes[s].name][1]))

    def get_self(self):
        return self.snakes[self.self_id]

    def get_snake(self, id):
        return self.snakes[id]

    def get_walls(self):
        return self.walls

    def print(self):
        print('------------------------------------')
        print('cycle: {}'.format(self.cycle))
        for f in self.board:
            print(f)
        print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$')
        for s in self.snakes:
            print(self.snakes[s].get_id(), self.snakes[s].get_head(), self.snakes[s].get_body())
        print('------------------------------------')
