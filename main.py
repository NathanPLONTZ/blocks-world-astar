"""Command-line entry point: solve a scenario and print the solution path.

Examples::

    python main.py                                  # subject states, g1 and h1
    python main.py --cost g2 --heuristic h2         # the second pair of functions
    python main.py --scenario flat                  # the three blocks on the table
"""

import argparse

from blocksworld.astar import astar
from blocksworld.node import COST_FUNCTIONS, HEURISTIC_FUNCTIONS
from blocksworld.scenarios import SCENARIOS


def parse_arguments(argv=None):
    """Read the scenario and the pair of functions to use from the command line."""
    parser = argparse.ArgumentParser(
        description="Solve the blocks-world problem with A*."
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default="subject",
        help="initial and goal states to use (default: %(default)s)",
    )
    parser.add_argument(
        "--cost",
        choices=sorted(COST_FUNCTIONS),
        default="g1",
        help="cost function g (default: %(default)s)",
    )
    parser.add_argument(
        "--heuristic",
        choices=sorted(HEURISTIC_FUNCTIONS),
        default="h1",
        help="heuristic function h (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    """Run the search and print the result. Returns the process exit code."""
    arguments = parse_arguments(argv)

    start, goal = SCENARIOS[arguments.scenario]()
    path, visited = astar(start, goal, cost=arguments.cost, heuristic=arguments.heuristic)

    print("Nombre de noeuds parcourus:{}".format(visited))

    if path is None:
        print("Aucune solution trouvee")
        return 1

    print("------------Solution------------")
    for node in path:
        print(node)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
