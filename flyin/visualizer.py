"""Optional terminal visualization of zones and drone positions."""

from __future__ import annotations

import sys
from collections import defaultdict
from typing import TextIO

from flyin.domain import MapDefinition, Zone
from flyin.planner import DronePlan, SimulationResult


class TerminalVisualizer:
    """Render a compact, color-aware view of each simulation turn."""

    _ANSI_COLORS: dict[str, str] = {
        "black": "30",
        "red": "31",
        "green": "32",
        "yellow": "33",
        "blue": "34",
        "magenta": "35",
        "purple": "35",
        "cyan": "36",
        "white": "37",
        "gray": "90",
        "grey": "90",
    }

    def __init__(
        self, use_color: bool = True, stream: TextIO | None = None
    ) -> None:
        """Create a renderer with optional ANSI colors.

        Args:
            use_color: Whether recognized metadata colors use ANSI escapes.
            stream: Output stream, defaulting to standard error.
        """
        self.use_color = use_color
        self.stream = stream or sys.stderr

    def render(self, graph: MapDefinition, result: SimulationResult) -> None:
        """Print the network and occupant list for each turn.

        Args:
            graph: Parsed map to display.
            result: Completed simulation to visualize.

        Returns:
            None: Text is written to the configured stream.
        """
        self.stream.write(
            f"Map: {graph.start_name} -> {graph.end_name} "
            f"({len(graph.zones)} zones, "
            f"{len(graph.connections)} connections)\n"
        )
        for connection in graph.connections:
            self.stream.write(
                f"  {connection.name} [capacity {connection.max_drones}]\n"
            )
        for turn in result.turns:
            self.stream.write(f"\nTurn {turn.number}/{result.turn_count}\n")
            locations = self._locations_at(result.plans, turn.number)
            for zone in graph.zones.values():
                occupants = locations.get(zone.name, [])
                self._write_zone(zone, occupants)
            flights = [
                (location, drones)
                for location, drones in locations.items()
                if location not in graph.zones
            ]
            for connection_name, drones in flights:
                self.stream.write(
                    f"  {connection_name}: {', '.join(drones)} (in flight)\n"
                )
            delivered = sum(
                1
                for plan in result.plans
                if plan.finish_time <= turn.number
            )
            self.stream.write(
                f"  Delivered: {delivered}/{len(result.plans)}\n"
            )

    @staticmethod
    def _locations_at(
        plans: list[DronePlan], time: int
    ) -> dict[str, list[str]]:
        """Group drone identifiers by their state after a turn.

        Args:
            plans: Schedules whose locations should be displayed.
            time: Turn to inspect.

        Returns:
            dict[str, list[str]]: Drone IDs grouped by zone or connection.
        """
        locations: dict[str, list[str]] = defaultdict(list)
        for plan in plans:
            state_times = [
                state_time
                for state_time in plan.states
                if state_time <= time
            ]
            if not state_times:
                continue
            last_time = max(state_times)
            if time <= plan.finish_time:
                locations[plan.states[last_time]].append(f"D{plan.drone_id}")
        return locations

    def _write_zone(self, zone: Zone, occupants: list[str]) -> None:
        """Print a zone's coordinates, properties, and occupants.

        Args:
            zone: Zone metadata to display.
            occupants: Drone IDs at the zone after the current turn.

        Returns:
            None: The formatted line is written to the configured stream.
        """
        label = zone.name
        color_code = self._ANSI_COLORS.get((zone.color or "").lower())
        if self.use_color and color_code is not None:
            label = f"\033[{color_code}m{label}\033[0m"
        role = f" {zone.role}" if zone.role != "hub" else ""
        capacity = "unlimited" if zone.capacity is None else str(zone.capacity)
        state = ", ".join(occupants) if occupants else "empty"
        self.stream.write(
            f"  {label}{role} ({zone.x}, {zone.y}; {zone.kind.value}; "
            f"capacity {capacity}): {state}\n"
        )
