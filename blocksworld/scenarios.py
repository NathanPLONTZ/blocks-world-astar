"""Initial and goal states the program can be run on.

A scenario is a function returning the ``(start, goal)`` pair of nodes. Adding
a new placement of the blocks only means adding a function here and an entry in
:data:`SCENARIOS`.
"""

from blocksworld.cube import Cube
from blocksworld.node import Node
from blocksworld.robot import Robot


def subject():
    """The states of the subject: figure (a) to figure (b).

    Initial state, C stacked on A::

        C
        A    B
        ---------- table

    Goal state, A on B on C::

        A
        B
        C
        ---------- table

    A is not free in the initial state since C rests on it, and C is not free
    in the goal state since B rests on it.
    """
    cube_a = Cube("A", False, False, None, True)
    cube_b = Cube("B", True, False, None, True)
    cube_c = Cube("C", True, False, cube_a, False)
    start = Node(None, cube_a, cube_b, cube_c, Robot(True))

    goal_c = Cube("C", False, False, None, True)
    goal_b = Cube("B", False, False, goal_c, False)
    goal_a = Cube("A", True, False, goal_b, False)
    goal = Node(None, goal_a, goal_b, goal_c, Robot(True))

    return start, goal


def flat():
    """A shorter scenario: the three blocks start on the table.

    Initial state::

        A    B    C
        ---------- table

    Goal state, A stacked on C::

        A
        C    B
        ---------- table
    """
    cube_a = Cube("A", True, False, None, True)
    cube_b = Cube("B", True, False, None, True)
    cube_c = Cube("C", True, False, None, True)
    start = Node(None, cube_a, cube_b, cube_c, Robot(True))

    goal_c = Cube("C", False, False, None, True)
    goal_b = Cube("B", True, False, None, True)
    goal_a = Cube("A", True, False, goal_c, False)
    goal = Node(None, goal_a, goal_b, goal_c, Robot(True))

    return start, goal


#: Scenarios selectable from the command line.
SCENARIOS = {"subject": subject, "flat": flat}
