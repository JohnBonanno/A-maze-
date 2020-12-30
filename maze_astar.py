#JOHN BONANNO

import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse
from platform import node
import matplotlib
matplotlib.use('TKAgg')

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start

class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index

    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
        draw(self.map, path[::-1], self.map_index)

    def goal_test(self, current_state):
        return current_state == self.goal_state

    def get_cost(self, current_state, next_state):
        return 1

    def get_successors(self, state):
        successors = []
        # row, col = np.where(0.5 == state) 
        row = state[0]
        col = state[1]
        ways = [[0,1],[1,0],[0,-1],[-1,0]] # right, down, left, up

        for way in ways:
            x = row + way[0]
            y = col + way[1]
            if x not in range(self.m) or y not in range(self.n):
                continue
            if (self.map[x][y]  == 1.0):         
                successors.append((x,y))
        return successors
    # heuristics function
    def heuristics(self, state):
        distance = 0
        for i in range(self.m):
            for j in range(self.n):
                x1 = state[0]
                y1 = state[1]
                x2 = self.goal_state[0]
                y2 = self.goal_state[1]
                distance += abs(x1 - x2) + abs(y1 - y2)
        return distance
        
    def priority(self, node):
        # your code goes here:
        return self.heuristics(node.state)

    # solve it
    def solve(self):
        if self.goal_test(self.start_state): return
        container = []
        count = 1
        state = self.start_state
        node = Node( state, 0, None)
        self.visited.append(state)
        heappush( container, (self.priority(node) + count, count, node ))
        while container:
            node = heappop(container)
            node = node[2]
            successors = self.get_successors(node.state) 
            for next_state in successors:
                if next_state not in self.visited:
                    self.visited.append( next_state )
                    next_cost = node.cost_from_start + self.get_cost(node.state, next_state)
                    next_node = Node(next_state, next_cost, node)
                    if( self.goal_test( next_state ) ):
                        self.draw( next_node )
                        return
                    heappush(container, (self.priority(next_node) + count, count, next_node))
                    count = count + 1
        
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1sa
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
