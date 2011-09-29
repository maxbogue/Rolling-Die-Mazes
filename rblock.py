"""A simple script to perform a Uniform Cost Search.

Author: Max Bogue

"""

from heapq import heappush, heappop
from sys import argv, exit

# A mapping of node names to a list of (neighbor name, edge cost) pairs.
graph = {}

# A mapping of node names to lowest found cost.
visited = {}

# A heap of States.
frontier = []

def print_graph():
    for k, v in graph.items():
        print("%s: %s" % (k, v))

def init_graph(edges):
    for a, b, cost in edges:
        graph.setdefault(a, []).append((b, cost))
        graph.setdefault(b, []).append((a, cost))

class State(object):
    """Represents a configuration of the problem space."""
    
    def __init__(self, x, y, die, action):
        self.x = x
        self.y = y
        self.die = die
        self.action = action
    
    @staticmethod
    def is_valid(state):
        return True
    
    def neighbors(self):
        """Naively return ALL neighbors of this state."""
        # Faces: (up, north, east)
        (u, n, e) = self.die
        return [
            State(self.x, self.y + 1, (7 - n, u, e), "N"), # North
            State(self.x, self.y - 1, (n, 7 - u, e), "S"), # South
            State(self.x + 1, self.y, (7 - e, n, u), "E"), # East
            State(self.x - 1, self.y, (e, n, 7 - u), "W"), # West
        ]
    
    def __equal__(self, o):
        return self.x == o.x and self.y == o.y and self.die == o.die
    
    def __hash__(self):
        return hash((x, y, dice))
    

class Node(object):
    """Represents a node in the algorithm search tree."""
    
    def __init__(self, state, cost, parent):
        self.state = state
        self.cost = cost
        self.parent = parent
    
    def expand(self):
        for neighbor, cost in graph[self.state]:
            next_cost = self.cost + cost
            if neighbor not in visited or next_cost < visited[neighbor]:
                heappush(frontier, State(neighbor, next_cost, self))
                visited[neighbor] = next_cost
    
    def unwind(self):
        if self.parent == None:
            return [self.state]
        else:
            path = self.parent.unwind()
            path.append(self.state)
            return path
    
    def is_outdated(self):
        return self.state in visited and visited[self.state] < self.cost
    
    def __lt__(self, other):
        """Used by built-in sorting algorithms."""
        return self.cost < other.cost

def uniform_cost_search(start, goal):
    assert start in graph, "Invalid start state '%s'" % start
    assert goal in graph, "Invalid goal state '%s'" % goal
    frontier.append(Node(start, 0, None))
    while True:
        if len(frontier) == 0:
            return ["FAIL"]
        state = None
        while not state or state.is_outdated():
            state = heappop(frontier)
        if state.node == goal:
            return state.unwind()
        else:
            state.expand()

if __name__ == "__main__":
    if len(argv) < 4:
        print("Usage: rsearch.py map_file start_node goal_node")
        exit(1)
    with open(argv[1], 'r') as f:
        def line_to_edge(line):
            parts = line.split(",")
            return (parts[0], parts[1], int(parts[2]))
        edges = map(line_to_edge, f.read().splitlines())
        init_graph(edges)
    print(uniform_cost_search(argv[2], argv[3]))
