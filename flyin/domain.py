"""Typed domain objects for maps, zones, and connections."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ZoneKind(str, Enum):
    """Supported zone types and their movement characteristics."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    @property
    def movement_cost(self) -> int:
        """Return turns needed to enter this zone.

        Returns:
            int: Two for restricted zones and one for all others.
        """
        return 2 if self is ZoneKind.RESTRICTED else 1


@dataclass(frozen=True, slots=True)
class Zone:
    """A named point in the drone network."""

    name: str
    x: int
    y: int
    kind: ZoneKind = ZoneKind.NORMAL
    color: str | None = None
    max_drones: int | None = 1
    role: str = "hub"

    @property
    def is_unlimited(self) -> bool:
        """Return whether this zone is exempt from occupancy limits.

        Returns:
            bool: True for the start and end hubs.
        """
        return self.role in {"start_hub", "end_hub"}

    @property
    def capacity(self) -> int | None:
        """Return the effective occupancy limit.

        Returns:
            int | None: Zone limit, or None for an unlimited hub.
        """
        return None if self.is_unlimited else self.max_drones


@dataclass(frozen=True, slots=True)
class Connection:
    """A bidirectional link between two zones."""

    identifier: int
    first: str
    second: str
    max_drones: int = 1

    @property
    def name(self) -> str:
        """Return the connection name used in movement output.

        Returns:
            str: The declared endpoint names joined by a dash.
        """
        return f"{self.first}-{self.second}"

    def other(self, zone_name: str) -> str:
        """Return the endpoint opposite ``zone_name``.

        Args:
            zone_name: One endpoint of this connection.

        Returns:
            str: The other endpoint.

        Raises:
            ValueError: If ``zone_name`` is not on this connection.
        """
        if zone_name == self.first:
            return self.second
        if zone_name == self.second:
            return self.first
        raise ValueError(f"Zone {zone_name!r} is not on {self.name!r}")


@dataclass(slots=True)
class MapDefinition:
    """A parsed zone graph and its drone count."""

    drone_count: int
    start_name: str
    end_name: str
    zones: dict[str, Zone]
    connections: list[Connection]
    adjacency: dict[str, list[Connection]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Build an adjacency index when one was not supplied.

        Returns:
            None: The index is stored on this map definition.
        """
        if not self.adjacency:
            self.adjacency = {name: [] for name in self.zones}
            for connection in self.connections:
                self.adjacency[connection.first].append(connection)
                self.adjacency[connection.second].append(connection)

    def has_route(self) -> bool:
        """Return whether a usable path exists without capacity checks.

        Returns:
            bool: True when the end can be reached without entering blocked
            zones.
        """
        pending = [self.start_name]
        visited = {self.start_name}
        while pending:
            current = pending.pop()
            if current == self.end_name:
                return True
            for connection in self.adjacency[current]:
                neighbor = connection.other(current)
                if (
                    neighbor not in visited
                    and self.zones[neighbor].kind is not ZoneKind.BLOCKED
                ):
                    visited.add(neighbor)
                    pending.append(neighbor)
        return False
