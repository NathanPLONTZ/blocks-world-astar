"""Search node, cost functions and heuristic functions."""

from blocksworld.cube import Cube
from blocksworld.robot import Robot

#: Names of the three blocks, in the order used throughout the project.
CUBE_NAMES = ("A", "B", "C")

#: Block attributes compared one by one by the first heuristic.
COMPARED_ATTRIBUTES = ("free", "held", "on", "on_table")


class Node:
    """A state of the search tree: the three blocks plus the robot arm.

    Besides the state itself a node carries the usual A* bookkeeping:

        ``g`` -- cost of the path from the initial state to this node
        ``h`` -- heuristic estimate of the cost remaining to the goal
        ``f`` -- ``g + h``, the value A* minimises
        ``parent`` -- the node this one was expanded from, used to rebuild the path
    """

    def __init__(self, parent=None, cube_a=None, cube_b=None, cube_c=None, robot=None):
        self.parent = parent
        self.cube_a = cube_a
        self.cube_b = cube_b
        self.cube_c = cube_c
        self.robot = robot

        self.g = 0
        self.h = 0
        self.f = 0

    def cubes(self):
        """Return the three blocks in the canonical A, B, C order."""
        return (self.cube_a, self.cube_b, self.cube_c)

    def cubes_by_name(self):
        """Return the three blocks indexed by name, to apply an operator by name."""
        return {"A": self.cube_a, "B": self.cube_b, "C": self.cube_c}

    def copy_from(self, node):
        """Copy ``node`` into ``self`` as an independent state.

        Each block and the robot are duplicated, so applying an operator to the
        copy cannot affect the original. ``Cube.copy_from`` duplicates the stack
        below a block as a throwaway chain of new objects, so the ``on``
        references are relinked afterwards to point at the three blocks of this
        node. Without that step two copies of the same block could coexist and
        the state comparisons would no longer be reliable.

        The parent link is intentionally not copied: callers build the children
        with an explicit parent.
        """
        self.g = node.g
        self.h = node.h
        self.f = node.f

        copied = []
        for cube in node.cubes():
            duplicate = Cube()
            duplicate.copy_from(cube)
            copied.append(duplicate)
        self.cube_a, self.cube_b, self.cube_c = copied

        robot = Robot()
        robot.copy_from(node.robot)
        self.robot = robot

        # Relink the stacks onto the blocks of this node.
        by_name = self.cubes_by_name()
        for cube in self.cubes():
            if cube.on is not None:
                cube.on = by_name[cube.on.name]

    def heuristic_h1(self, goal):
        """First heuristic: count the predicates that differ from the goal.

        Every block attribute (LIBRE, TENU, SUR, SURTABLE) is compared with its
        counterpart in the goal state, and BRASVIDE is compared as well. Each
        difference adds 1, so the result ranges from 0 to 13.
        """
        result = 0
        for cube, goal_cube in zip(self.cubes(), goal.cubes()):
            for attribute in COMPARED_ATTRIBUTES:
                if getattr(goal_cube, attribute) != getattr(cube, attribute):
                    result = result + 1

        if goal.robot.arm_empty != self.robot.arm_empty:
            result = result + 1

        return result

    def heuristic_h2(self, goal):
        """Second heuristic: count the blocks that differ from the goal.

        A block counts for 1 as soon as any of its predicates differs, instead
        of counting each predicate separately. The robot arm is compared the
        same way, so the result ranges from 0 to 4. This is a coarser estimate
        than :meth:`heuristic_h1`: it stays much closer to zero and therefore
        guides the search far less.
        """
        result = 0
        for cube, goal_cube in zip(self.cubes(), goal.cubes()):
            if cube != goal_cube:
                result = result + 1
        if self.robot != goal.robot:
            result = result + 1
        return result

    def cost_g1(self, node):
        """First cost function: every operator costs 1."""
        return node.g + 1

    def cost_g2(self, node):
        """Second cost function: only putting a block down costs 1.

        The arm of ``self`` is empty exactly when the operator that produced
        this node was a POSER, so picking a block up is free and a complete
        move (pick up, then put down) still costs 1.
        """
        if self.robot.arm_empty:
            return node.g + 1
        return node.g

    def __str__(self):
        """Return the multi-line description of the node, as in the original program."""
        return "Noeud (f:{}) :\n{}\n{}\n{}\n{}\n\n".format(
            self.f, self.robot, self.cube_a, self.cube_b, self.cube_c
        )

    def __eq__(self, other):
        """Two nodes are equal when their three blocks and their arm are equal.

        The A* bookkeeping and the parent link are ignored on purpose: equality
        means "same state of the world", which is what the OPEN and CLOSED
        lookups need.
        """
        return (
            self.cube_a == other.cube_a
            and self.cube_b == other.cube_b
            and self.cube_c == other.cube_c
            and self.robot == other.robot
        )


#: Cost functions selectable from the command line.
COST_FUNCTIONS = {"g1": Node.cost_g1, "g2": Node.cost_g2}

#: Heuristic functions selectable from the command line.
HEURISTIC_FUNCTIONS = {"h1": Node.heuristic_h1, "h2": Node.heuristic_h2}
