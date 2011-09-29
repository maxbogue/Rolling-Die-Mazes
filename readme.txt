Run the program with:

    python3 rblock.py <puzzle_file>

The output will be a series of lines like this:

    N -> <0, 0 (1, 2, 3)>

which is of the form

    direction -> new_state
    D -> <x, y, (up, north, east)>

where direction is N, S, E or W for the cardinal directions.
(up, north, east) is the die orientation at the new state.
