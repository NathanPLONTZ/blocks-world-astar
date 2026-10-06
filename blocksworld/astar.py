"""A* search over the blocks-world state space."""

from blocksworld.node import COST_FUNCTIONS, HEURISTIC_FUNCTIONS
from blocksworld.operators import successors


def astar(start, goal, cost="g1", heuristic="h1"):
    """Search a path of operators leading from ``start`` to ``goal``.

    ``cost`` and ``heuristic`` select the functions to use, by the names of
    :data:`blocksworld.node.COST_FUNCTIONS` and
    :data:`blocksworld.node.HEURISTIC_FUNCTIONS`.

    Returns the pair ``(path, visited)``, where ``path`` is the list of nodes
    from ``start`` to ``goal`` and ``visited`` is the number of nodes taken out
    of OPEN, i.e. the size of the search. ``path`` is ``None`` when the goal
    cannot be reached.
    """
    cost_function = COST_FUNCTIONS[cost]
    heuristic_function = HEURISTIC_FUNCTIONS[heuristic]

    start.g = start.h = start.f = 0
    goal.g = goal.h = goal.f = 0

    # OPEN holds the nodes still to examine, CLOSED the ones already expanded.
    open_list = [start]
    closed_list = []

    visited = 1

    while len(open_list) > 0:
        visited = visited + 1

        # Pick the node of OPEN with the lowest f. On a tie the earliest node
        # of the list wins, because the comparison is strict.
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Move the chosen node from OPEN to CLOSED.
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Goal reached: walk the parent links back to the initial state.
        if current_node == goal:
            path = []
            node = current_node
            while node is not None:
                path.append(node)
                node = node.parent
            return list(reversed(path)), visited

        for child in successors(current_node):
            present = False

            # Discard the child if CLOSED already holds the same state reached
            # at no greater cost, and drop the entry of CLOSED when this child
            # reaches the state more cheaply.
            #
            # NOTE: this test runs *before* g is computed below, which is the
            # order of the submitted program and the reason the node counts
            # reported in the README are reproducible. Because g still holds
            # its initial value of 0 here, the comparison only keeps a child
            # whose state matches a node of CLOSED of cost 0, and re-opens the
            # others. The search therefore expands some states more than once.
            # Moving the three assignments below above this loop yields the
            # textbook A*, the same solution paths, and smaller node counts
            # (12 instead of 13 with g1/h1, 18 instead of 20 with g2/h2).
            for index, closed_child in enumerate(closed_list):
                if child == closed_child and child.g >= closed_child.g:
                    present = True
                if child == closed_child and child.g < closed_child.g:
                    closed_list.pop(index)

            # Evaluate the child.
            child.g = cost_function(child, current_node)
            child.h = heuristic_function(child, goal)
            child.f = child.g + child.h

            # Same comparison against OPEN.
            for index, open_node in enumerate(open_list):
                if child == open_node and child.g >= open_node.g:
                    present = True
                if child == open_node and child.g < open_node.g:
                    open_list.pop(index)

            # The child brings nothing better than a state already known.
            if present:
                continue

            open_list.append(child)

    return None, visited
