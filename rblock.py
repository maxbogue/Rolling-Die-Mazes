"""A simple script to perform a Uniform Cost Search.

Author: Max Bogue

"""

from heapq import heappush, heappop
from sys import argv, exit

# A list of lists of puzzle symbols (.*SG).
puzzle = []

# A mapping of node names to lowest found cost.
visited = {}

# A heap of States.
frontier = []

def print_puzzle():
    for y in reversed(range(len(puzzle[0]))):
        print("".join([puzzle[x][y] for x in range(len(puzzle))]))

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
        """Tests a state for validity.
        
        Returns False if the state is off the board, a "*" space,
        or has a 6 on top.
        
        """
        return (
            0 <= state.x < len(puzzle) and
            0 <= state.y < len(puzzle[0]) and
            puzzle[state.x][state.y] != '*' and
            state.die[0] != 6
        )
    
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
    def __str__(self):
        return "<%s, %s %s>" % (self.x, self.y, self.die)
    __repr__ = __str__
    
    def __hash__(self):
        return hash((self.x, self.y, self.die))
    

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
    
    def __str__(self):
        return "%s | %s" % (self.state, self.cost)
    __repr__ = __str__
    

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
    if len(argv) < 2:
        print("Usage: rblock.py puzzle_file")
        exit(1)
    start = None
    goal_loc = None
    with open(argv[1], 'r') as f:
        lines = f.read().splitlines()
        lines.reverse()
    # Validate input and find the start state.
    puzzle = [[] for _ in range(len(lines[0]))]
    for y, line in enumerate(lines):
        for x, c in enumerate(line):
            puzzle[x].append(c)
            if c not in '.*SG':
                print("Invalid character '%s' detected!" % c)
                exit(1)
            if c == 'S':
                if start != None:
                    print("Multiple start states detected!")
                    exit(1)
                start = State(x, y, (1, 2, 3), None)
            if c == 'G':
                if goal_loc != None:
                    print("Multiple goal states detected!")
                    exit(1)
                goal_loc = (x, y)
    if start == None:
        print("No start state found!")
        exit(1)
    if goal_loc == None:
        print("No goal location detected!")
        exit(1)
    print_puzzle()
    # print(a_star_search(start, goal))
