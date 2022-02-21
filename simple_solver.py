import heapq
import time
import numpy as np
import collections


class Queue:
    """Priority Queue data structure based on heapq"""

    def __init__(self):
        self.elements = []
        self.count = 0

    def add(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.elements, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.elements)
        return item


class State:
    def __init__(self, player, boxes):
        self.player = player
        self.boxes = boxes

    def goal_state(self, goals):
        return all(map(lambda g: g in self.boxes, goals))

    def manhattan_distance(self, box, goal):
        return abs(box[0] - goal[0]) + abs(box[1] - goal[1])

    def distance_heuristic(self, goals):
        remaining_boxes = list(self.boxes)
        result = 0
        for g in goals:
            min_distance = 999999
            min_box = None
            for box in remaining_boxes:
                distance = self.manhattan_distance(box, g)
                if distance < min_distance:
                    min_distance = distance
                    min_box = box
            box = min_box
            result += min_distance
            remaining_boxes.remove(box)
        return result

    def plot_notation(self, walls, goals):
        plot = ""  # ["" for i in range(len(self.walls))]
        for y in range(0, len(walls[0])):
            for x in range(0, len(walls)):
                if walls[x][y]:
                    plot += "#"
                elif [x, y] == self.player:
                    plot += "@"
                elif [x, y] in self.boxes:
                    plot += "$"
                elif [x, y] in goals:
                    plot += "."
                else:
                    plot += " "
            plot += '\n'
        print(plot)


class AISolver:
    def __init__(self, board):

        self.initial_state, self.walls, self.goals = self.standard_notation_to_board_state(board)

    def solve(self, algorithm):

        start = time.time()
        self.nr_nodes = 0

        if algorithm == "BFS":
            print("Search using breadth first search")
            solution = self.breadthFirstSearch(self.initial_state)
        elif algorithm == "A*":
            print("Search using A* search")
            solution = self.a_star_search(self.initial_state)
        else:
            print("Algorithm ", algorithm, "is not implemented")
            solution = None

        print("Actions: ")
        print(solution)
        print("Computation time:", time.time() - start, "seconds")
        print("Number of generated nodes: ", self.nr_nodes)

        return solution

    def standard_notation_to_board_state(self, board):
        walls = np.zeros((len(max(board, key=len)), len(board)))
        boxes = []
        goals = []
        y = 0
        for row in board:
            x = 0
            for char in row:
                if char == "#":
                    walls[x, y] = 1
                elif char == "@":
                    player = [x, y]
                elif char == "$":
                    boxes.append([x, y])
                elif char == ".":
                    goals.append([x, y])
                elif char == " ":
                    pass
                elif char == "+":
                    goals.append([x, y])
                    player = [x, y]
                elif char == "*":
                    goals.append([x, y])
                    boxes.append([x, y])
                else:
                    print("Unknown character")
                x += 1
            y += 1

        board_state = State(player, boxes)
        return board_state, walls, goals

    def a_star_search(self, state):
        """
        A* search
        """

        start_state = state
        fringe = Queue()  # store states
        fringe.add([start_state], start_state.distance_heuristic(self.goals))  # add start state to fringe
        explored = set()  # store set of explored states
        actions = Queue()  # store actions and their f-value
        actions.add([], start_state.distance_heuristic(self.goals))  # add current
        while fringe:
            node = fringe.pop()  # get node
            node_action = actions.pop()  # get path of actions to current node
            if node[-1].goal_state(self.goals):  # check if goal state is reached
                solution = node_action  # solution is path / list of actions
                break
            if not self.in_explored_set(node[-1], explored):  # if current state is not in explored set
                explored.add(node[-1])  # add current state to explored set
                cost = len(node_action[1:])  # cost of current path of actions equals number of actions
                moves = ["up", "down", "right", "left"]
                for direction in moves:  # iterate possible moves
                    new_state = self.move(node[-1], direction)  # move
                    if new_state is None:  # if move is not legal
                        continue
                    n_new = node.copy()
                    n_new.append(new_state)  # append new state to path of current node
                    fringe.add(n_new, new_state.distance_heuristic(
                        self.goals) + cost)  # append node and its f-value to path of current node
                    actions.add(node_action + [direction], new_state.distance_heuristic(
                        self.goals) + cost)  # append action and its cost to path of actions
                    self.nr_nodes += 1
        return solution

    def breadthFirstSearch(self, state):
        """
        Implementation of breadth first search
        """

        start_state = state

        fringe = collections.deque([[start_state]])  # store states
        actions = collections.deque([[]])  # store actions
        explored = set()  # store set of explored states
        while fringe:
            node = fringe.popleft()  # get node
            node_action = actions.popleft()  # get path of actions to current node
            if node[-1].goal_state(self.goals):  # check if goal state is reached
                solution = node_action  # solution is path / list of actions
                break
            if not self.in_explored_set(node[-1], explored):  # if current state is not in explored set
                explored.add(node[-1])  # add current state to explored set
                moves = ["up", "down", "right", "left"]
                for direction in moves:  # iterate possible moves
                    new_state = self.move(node[-1], direction)  # move
                    if new_state is None:  # if move is not legal
                        continue
                    n_new = node.copy()
                    n_new.append(new_state)  # append new state to path of current node
                    fringe.append(n_new)  # append node to fringe
                    actions.append(node_action + [direction])  # append path of actions
                    self.nr_nodes += 1
        return solution

    def in_explored_set(self, state, explored_set):
        contained = False
        for s in explored_set:
            if state.boxes == s.boxes and state.player == s.player:
                contained = True
                break
        return contained

    def move(self, state, direction):
        new_state = State(state.player.copy(), state.boxes.copy())

        if direction == "up":
            target_player = [state.player[0], state.player[1] - 1]
            target_box = [state.player[0], state.player[1] - 2]

        if direction == "down":
            target_player = [state.player[0], state.player[1] + 1]
            target_box = [state.player[0], state.player[1] + 2]

        if direction == "right":
            target_player = [state.player[0] + 1, state.player[1]]
            target_box = [state.player[0] + 2, state.player[1]]

        if direction == "left":
            target_player = [state.player[0] - 1, state.player[1]]
            target_box = [state.player[0] - 2, state.player[1]]

        if 0 <= target_player[0] < len(self.walls[0]) and 0 <= target_player[1] < len(
                self.walls):  # if not out of bounds
            if not self.walls[target_player[0]][target_player[1]]:
                new_state.player = target_player
                if target_player in state.boxes:  # if box is on player target position
                    if 0 <= target_box[0] < len(self.walls[0]) and 0 <= target_box[1] < len(
                            self.walls):  # if box target position is not out of bounds
                        if not self.walls[target_box[0]][
                            target_box[1]] and not target_box in state.boxes:  # if box target position is not wall
                            new_state.boxes[new_state.boxes.index(target_player)] = target_box
                        else:
                            new_state = None
                    else:
                        new_state = None
            else:
                new_state = None
        else:
            new_state = None
        return new_state
