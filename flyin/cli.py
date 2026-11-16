"""Command-line interface for the Fly-in simulator."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from flyin.parser import MapParser
from flyin.planner import Simulator
from flyin.visualizer import TerminalVisualizer


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser.

    Returns:
        argparse.ArgumentParser: Parser for the map path and display options.
    """
    parser = argparse.ArgumentParser(
        description="Route drones through a capacity-constrained zone network."
    )
    parser.add_argument(
        "map_file", type=Path, help="path to a Fly-in map file"
    )
    parser.add_argument(
        "--visual",
        action="store_true",
        help="show a per-turn zone and drone view on stderr",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="disable ANSI colors in the optional visual view",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the parser, scheduler, and requested output views.

    Args:
        argv: Optional argument list, defaulting to process arguments.

    Returns:
        int: Zero on success, or two when map loading or routing fails.
    """
    arguments = build_argument_parser().parse_args(argv)
    try:
        graph = MapParser().parse_file(arguments.map_file)
        result = Simulator(graph).run()
    except (ValueError, OSError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2

    for turn in result.turns:
        print(" ".join(turn.movements))
    if arguments.visual:
        TerminalVisualizer(use_color=not arguments.no_color).render(
            graph, result
        )
    return 0
