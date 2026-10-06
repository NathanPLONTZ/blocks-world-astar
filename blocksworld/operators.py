"""The twelve operators that generate the children of a state.

The original program repeated the same block of code twelve times, once per
operator. The operators are described here as data instead, and
:func:`successors` applies them in a single loop.
"""

from blocksworld.node import Node

#: Identifier of the TENIR action.
HOLD = "hold"

#: Identifier of the POSER action.
PUT = "put"

#: The twelve operators, as ``(block, action, target)`` triples.
#:
#: ``target`` is the block to stack onto, or ``None`` for the table and for the
#: TENIR action. The order is the one of the original program and must not be
#: changed: A* keeps the first node of minimal ``f`` it encounters, so the order
#: in which the children are produced decides how ties are broken and, in turn,
#: how many nodes the search visits.
OPERATORS = (
    ("A", HOLD, None),
    ("A", PUT, "B"),
    ("A", PUT, "C"),
    ("A", PUT, None),
    ("B", HOLD, None),
    ("B", PUT, "A"),
    ("B", PUT, "C"),
    ("B", PUT, None),
    ("C", HOLD, None),
    ("C", PUT, "A"),
    ("C", PUT, "B"),
    ("C", PUT, None),
)


def successors(node):
    """Return the children reachable from ``node``.

    Each operator is applied to its own copy of ``node``, so an operator whose
    preconditions fail cannot leave a half-modified state behind, and a
    successful one cannot interfere with the operators tested after it. Only
    the operators that actually fired produce a child.
    """
    children = []
    for cube_name, action, target_name in OPERATORS:
        candidate = Node()
        candidate.copy_from(node)
        blocks = candidate.cubes_by_name()

        if action == HOLD:
            applied = candidate.robot.hold(blocks[cube_name])
        else:
            target = blocks[target_name] if target_name is not None else None
            applied = candidate.robot.put(blocks[cube_name], target)

        if applied:
            children.append(
                Node(
                    node,
                    candidate.cube_a,
                    candidate.cube_b,
                    candidate.cube_c,
                    candidate.robot,
                )
            )

    return children
