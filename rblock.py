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
    """Represents a state in the algorithm."""
    def __init__(self, node, cost, parent):
        self.node = node
        self.cost = cost
        self.parent = parent
    
    def expand(self):
        for neighbor, cost in graph[self.node]:
            next_cost = self.cost + cost
            if neighbor not in visited or next_cost < visited[neighbor]:
                heappush(frontier, State(neighbor, next_cost, self))
                visited[neighbor] = next_cost
    
    def unwind(self):
        if self.parent == None:
            return [self.node]
        else:
            path = self.parent.unwind()
            path.append(self.node)
            return path
    
    def is_outdated(self):
        return self.node in visited and visited[self.node] < self.cost
    
    def __lt__(self, other):
        """Used by built-in sorting algorithms."""
        return self.cost < other.cost

def uniform_cost_search(start, goal):
    assert start in graph, "Invalid start node '%s'" % start
    assert goal in graph, "Invalid goal node '%s'" % goal
    frontier.append(State(start, 0, None))
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
