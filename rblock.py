"""A simple script to perform a Uniform Cost Search.

Author: Max Bogue

"""

from heapq import heappush, heappop
from sys import argv, exit

# A list of lists of puzzle symbols (.*SG).
puzzle = []

# The goal location (x, y).
goal_loc = None

# A mapping of node names to lowest found cost.
visited = {}

# A heap of States.
frontier = []

# Global counters for number of states visited and generated.
num_visited = 0
num_generated = 0

# The heuristic function
heuristic = lambda state: 0

def print_puzzle(other=None):
    """Prints a puzzle configuration."""
    p = other if other else puzzle
    for y in reversed(range(len(p[0]))):
        print("".join([str(p[x][y]) for x in range(len(p))]))

def valid_space(x, y):
    """Tests whether the given x, y is in the puzzle space and not a '*'."""
    return (0 <= x < len(puzzle) and
        0 <= y < len(puzzle[0]) and
        puzzle[x][y] != '*')

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
        return valid_space(state.x, state.y) and state.die[0] != 6
    
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
        """For dicts/sets."""
        return isinstance(o, State) and self.x == o.x and self.y == o.y and self.die == o.die
    
    def __hash__(self):
        """For dicts/sets."""
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
        """Expand this node and add its state's neighbors to the frontier."""
        global num_generated, num_visited
        visited[self.state] = self.cost
        num_visited += 1
        for neighbor in self.state.neighbors():
            next_cost = self.cost + 1 + heuristic(neighbor)
            if neighbor not in visited or next_cost < visited[neighbor]:
                heappush(frontier, Node(neighbor, next_cost, self))
                num_generated += 1
    
    def unwind(self):
        """Unwind the path of actions and states taken to reach this ndoe."""
        if self.parent == None:
            return []
        else:
            path = self.parent.unwind()
            path.append((self.state.action, self.state))
            return path
    
    def is_outdated(self):
        """See if a better cost has been found for this node's state."""
        return self.state in visited and visited[self.state] < self.cost
    
    def __lt__(self, other):
        """Used by built-in sorting algorithms."""
        return self.cost < other.cost
    
    def __str__(self):
        return "%s | %s" % (self.state, self.cost)
    __repr__ = __str__
    

def a_star_search(start):
    """Perform A* search."""
    global num_visited, num_generated, goal_loc, frontier, visited
    frontier = []
    visited = {}
    num_visited = 1
    num_generated = 0
    def is_goal(state):
        return (state.x, state.y) == goal_loc and state.die[0] == 1
    frontier.append(Node(start, 0, None))
    visited[start] = 0
    while True:
        if len(frontier) == 0:
            return ["FAIL"], num_visited, num_generated
        node = None
        # This loop skips outdated nodes so I don't have to search through
        # the frontier for them.
        while not node or node.is_outdated():
            node = heappop(frontier)
        if is_goal(node.state):
            return node.unwind(), num_visited, num_generated
        else:
            node.expand()

# Heuristics!

def euclidean_distance(state):
    return ((state.x - goal_loc[0]) ** 2 + (state.y - goal_loc[1]) ** 2) ** 0.5

def manhattan_distance(state):
    return abs(state.x - goal_loc[0]) + abs(state.y - goal_loc[1])

# Path distances for the path heuristic.
distances = []

def generate_distances():
    """A breadth first search to find distances from the goal for each loc."""
    global distances
    # Copy the puzzle.
    distances = list(map(list, puzzle))
    def unseen(v):
        x, y = v
        return valid_space(x, y) and type(distances[x][y]) != int
    def next_states(x, y):
        return filter(unseen, [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)
        ])
    frontier = [(goal_loc[0], goal_loc[1])]
    d = 0
    while len(frontier) > 0:
        layer = list(frontier)
        frontier = set([])
        for x, y in layer:
            distances[x][y] = d
            frontier.update(next_states(x, y))
        d += 1

def path_distance(state):
    return distances[state.x][state.y]

def main():
    if len(argv) < 2:
        print("Usage: rblock.py puzzle_file")
        exit(1)
    global puzzle, goal_loc, heuristic
    start = None
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
    def print_results():
        path, nvisited, ngenerated = a_star_search(start)
        for action, new_state in path:
            print("%s -> %s" % (action, new_state))
        print("%s visited out of %s generated." % (nvisited, ngenerated))
    print("Euclidean distance:")
    heuristic = euclidean_distance
    print_results()
    print("Manhattan distance:")
    heuristic = manhattan_distance
    print_results()
    print("Path distance:")
    generate_distances()
    heuristic = path_distance
    print_results()

if __name__ == "__main__":
    main()
