"""Blocks-world planning solved with A*.

The package is split along the model of the subject:

    ``cube``       the blocks and their predicates
    ``robot``      the arm and the four production rules
    ``node``       a state of the search, with the cost and heuristic functions
    ``operators``  the twelve operators generating the children of a state
    ``astar``      the search itself
    ``scenarios``  the initial and goal states to run on
"""
