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

# Global counters for number of states visited and generated.
num_visited = 0
num_generated = 0

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
        """Return all valid neighbors of this state."""
        # Die faces: (up, north, east)
        (u, n, e) = self.die
        return filter(State.is_valid, [
            State(self.x, self.y + 1, (7 - n, u, e), "N"), # North
            State(self.x, self.y - 1, (n, 7 - u, e), "S"), # South
            State(self.x + 1, self.y, (7 - e, n, u), "E"), # East
            State(self.x - 1, self.y, (e, n, 7 - u), "W"), # West
        ])
    
    def __eq__(self, o):
        return isinstance(o, State) and self.x == o.x and self.y == o.y and self.die == o.die
    
    def __hash__(self):
        return hash((self.x, self.y, self.die))
    
    def __str__(self):
        return "<%s, %s %s>" % (self.x, self.y, self.die)
    __repr__ = __str__

class Node(object):
    """Represents a node in the algorithm search tree."""
    
    def __init__(self, state, cost, parent):
        self.state = state
        self.cost = cost
        self.parent = parent
    
    def expand(self):
        global num_generated, num_visited
        visited[self.state] = self.cost
        num_visited += 1
        for neighbor in self.state.neighbors():
            next_cost = self.cost + 1
            if neighbor not in visited or next_cost < visited[neighbor]:
                heappush(frontier, Node(neighbor, next_cost, self))
                num_generated += 1
    
    def unwind(self):
        if self.parent == None:
            return []
        else:
            path = self.parent.unwind()
            path.append(self.state.action)
            return path
    
    def is_outdated(self):
        return self.state in visited and visited[self.state] < self.cost
    
    def __lt__(self, other):
        """Used by built-in sorting algorithms."""
        return self.cost < other.cost
    
    def __str__(self):
        return "%s | %s" % (self.state, self.cost)
    __repr__ = __str__
    

def uniform_cost_search(start, goal_loc):
    def is_goal(state):
        return (state.x, state.y) == goal_loc and state.die[0] == 1
    frontier.append(Node(start, 0, None))
    visited[start] = 0
    global num_visited, num_generated
    num_visited = 1
    num_generated = 0
    while True:
        if len(frontier) == 0:
            return ["FAIL"], num_visited, num_generated
        node = None
        while not node or node.is_outdated():
            node = heappop(frontier)
        if is_goal(node.state):
            return node.unwind(), num_visited, num_generated
        else:
            node.expand()

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
    print(uniform_cost_search(start, goal_loc))
