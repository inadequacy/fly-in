"""Strict parser for Fly-in map files."""

from __future__ import annotations

import re
from pathlib import Path

from flyin.domain import Connection, MapDefinition, Zone, ZoneKind


class MapParseError(ValueError):
    """A map syntax or validation error with a source line number."""

    def __init__(self, line_number: int, message: str) -> None:
        super().__init__(f"line {line_number}: {message}")
        self.line_number = line_number
        self.message = message


class MapParser:
    """Parse and validate a map while preserving declaration order."""

    _ZONE_PATTERN = re.compile(r"([^\s\-\[\]]+)\s+([+-]?\d+)\s+([+-]?\d+)$")
    _CONNECTION_PATTERN = re.compile(r"([^\s\-\[\]]+)-([^\s\-\[\]]+)$")
    _DECLARATION_PATTERN = re.compile(
        r"(nb_drones|start_hub|end_hub|hub|connection):\s*(.*?)\s*$"
    )

    def parse_file(self, path: str | Path) -> MapDefinition:
        """Read a map file and convert file errors to clear errors.

        Args:
            path: File system path to the map.

        Returns:
            MapDefinition: Validated graph and drone count.

        Raises:
            ValueError: If the file cannot be read or decoded, or is invalid.
        """
        map_path = Path(path)
        try:
            with map_path.open("r", encoding="utf-8") as map_file:
                return self.parse_text(map_file.read())
        except OSError as error:
            raise ValueError(
                f"cannot read map file {str(map_path)!r}: {error}"
            ) from error
        except UnicodeError as error:
            raise ValueError(
                f"map file {str(map_path)!r} is not UTF-8 text"
            ) from error

    def parse_text(self, text: str) -> MapDefinition:
        """Parse map text and return a validated graph definition.

        Args:
            text: Complete UTF-8 map contents.

        Returns:
            MapDefinition: Parsed zones, connections, and drone count.

        Raises:
            MapParseError: If syntax or map constraints are invalid.
        """
        drone_count: int | None = None
        zones: dict[str, Zone] = {}
        connections: list[Connection] = []
        seen_edges: set[frozenset[str]] = set()
        start_name: str | None = None
        end_name: str | None = None

        for line_number, original_line in enumerate(
            text.splitlines(), start=1
        ):
            line = original_line.split("#", maxsplit=1)[0].strip()
            if not line:
                continue
            if drone_count is None:
                count_match = re.fullmatch(r"nb_drones:\s*(\d+)\s*", line)
                if count_match is None:
                    raise MapParseError(
                        line_number,
                        "the first declaration must be "
                        "nb_drones: <positive_integer>",
                    )
                drone_count = int(count_match.group(1))
                if drone_count <= 0:
                    raise MapParseError(
                        line_number, "drone count must be a positive integer"
                    )
                continue

            declaration = self._DECLARATION_PATTERN.fullmatch(line)
            if declaration is None:
                raise MapParseError(
                    line_number, "unrecognized declaration or malformed syntax"
                )
            kind, body = declaration.groups()
            if kind == "nb_drones":
                raise MapParseError(
                    line_number,
                    "nb_drones may only be declared once, on the first line",
                )
            body, metadata = self._split_metadata(body, line_number)
            if kind == "connection":
                connection = self._parse_connection(
                    body, metadata, zones, line_number, len(connections)
                )
                edge_key = frozenset((connection.first, connection.second))
                if edge_key in seen_edges:
                    raise MapParseError(
                        line_number,
                        f"duplicate connection {connection.name!r}",
                    )
                seen_edges.add(edge_key)
                connections.append(connection)
                continue

            zone = self._parse_zone(kind, body, metadata, line_number)
            if zone.name in zones:
                raise MapParseError(
                    line_number, f"duplicate zone name {zone.name!r}"
                )
            zones[zone.name] = zone
            if kind == "start_hub":
                if start_name is not None:
                    raise MapParseError(
                        line_number, "exactly one start_hub is allowed"
                    )
                start_name = zone.name
            elif kind == "end_hub":
                if end_name is not None:
                    raise MapParseError(
                        line_number, "exactly one end_hub is allowed"
                    )
                end_name = zone.name

        if drone_count is None:
            raise MapParseError(1, "missing nb_drones declaration")
        if start_name is None:
            raise MapParseError(1, "missing start_hub declaration")
        if end_name is None:
            raise MapParseError(1, "missing end_hub declaration")
        if start_name == end_name:
            raise MapParseError(
                1, "start_hub and end_hub must be distinct zones"
            )

        return MapDefinition(
            drone_count, start_name, end_name, zones, connections
        )

    def _split_metadata(
        self, body: str, line_number: int
    ) -> tuple[str, dict[str, str]]:
        """Separate an optional bracketed metadata block from a body.

        Args:
            body: Declaration content after its type prefix.
            line_number: Source line used in any parsing error.

        Returns:
            tuple[str, dict[str, str]]: Metadata-free body and parsed tags.
        """
        if "[" not in body and "]" not in body:
            return body.strip(), {}
        match = re.fullmatch(r"(.*?)\s+\[([^\[\]]*)\]\s*", body)
        if match is None:
            raise MapParseError(line_number, "malformed metadata block")
        content = match.group(2).strip()
        if not content:
            raise MapParseError(line_number, "metadata block cannot be empty")
        values: dict[str, str] = {}
        for item in content.split():
            if item.count("=") != 1:
                raise MapParseError(
                    line_number,
                    f"invalid metadata item {item!r}; expected key=value",
                )
            key, value = item.split("=", maxsplit=1)
            if not key or not value:
                raise MapParseError(
                    line_number,
                    f"invalid metadata item {item!r}; expected key=value",
                )
            if key in values:
                raise MapParseError(
                    line_number, f"duplicate metadata key {key!r}"
                )
            values[key] = value
        return match.group(1).strip(), values

    def _parse_zone(
        self,
        role: str,
        body: str,
        metadata: dict[str, str],
        line_number: int,
    ) -> Zone:
        """Validate and create one zone declaration.

        Args:
            role: Declaration prefix identifying the hub role.
            body: Zone name and coordinates.
            metadata: Parsed zone tags.
            line_number: Source line used in any parsing error.

        Returns:
            Zone: Validated zone record.
        """
        match = self._ZONE_PATTERN.fullmatch(body)
        if match is None:
            raise MapParseError(
                line_number, "zone syntax is '<name> <x> <y> [metadata]'"
            )
        name, x_text, y_text = match.groups()
        unknown = set(metadata) - {"zone", "color", "max_drones"}
        if unknown:
            raise MapParseError(
                line_number,
                f"unknown zone metadata key {sorted(unknown)[0]!r}",
            )
        zone_text = metadata.get("zone", ZoneKind.NORMAL.value)
        try:
            zone_kind = ZoneKind(zone_text)
        except ValueError as error:
            raise MapParseError(
                line_number, f"invalid zone type {zone_text!r}"
            ) from error
        color = metadata.get("color")
        if color is not None and re.fullmatch(r"[^\s\[\]]+", color) is None:
            raise MapParseError(line_number, f"invalid color value {color!r}")
        max_drones: int | None = 1
        if role not in {"start_hub", "end_hub"}:
            if "max_drones" in metadata:
                max_drones = self._positive_integer(
                    metadata["max_drones"], "max_drones", line_number
                )
        return Zone(
            name, int(x_text), int(y_text), zone_kind, color, max_drones, role
        )

    def _parse_connection(
        self,
        body: str,
        metadata: dict[str, str],
        zones: dict[str, Zone],
        line_number: int,
        identifier: int,
    ) -> Connection:
        """Validate and create one connection declaration.

        Args:
            body: Two declared zone names.
            metadata: Parsed connection tags.
            zones: Zones defined before this line.
            line_number: Source line used in any parsing error.
            identifier: Stable connection index.

        Returns:
            Connection: Validated bidirectional link.
        """
        match = self._CONNECTION_PATTERN.fullmatch(body)
        if match is None:
            raise MapParseError(
                line_number,
                "connection syntax is '<zone1>-<zone2> [metadata]'",
            )
        first, second = match.groups()
        if first == second:
            raise MapParseError(
                line_number, "a connection must join two different zones"
            )
        for endpoint in (first, second):
            if endpoint not in zones:
                raise MapParseError(
                    line_number,
                    f"connection references undefined zone {endpoint!r}",
                )
        unknown = set(metadata) - {"max_link_capacity"}
        if unknown:
            raise MapParseError(
                line_number,
                f"unknown connection metadata key {sorted(unknown)[0]!r}",
            )
        capacity = self._positive_integer(
            metadata.get("max_link_capacity", "1"),
            "max_link_capacity",
            line_number,
        )
        return Connection(identifier, first, second, capacity)

    @staticmethod
    def _positive_integer(value: str, key: str, line_number: int) -> int:
        """Parse a positive integer metadata value.

        Args:
            value: Text to parse.
            key: Metadata key used in error messages.
            line_number: Source line used in any parsing error.

        Returns:
            int: The parsed positive capacity.

        Raises:
            MapParseError: If the value is not a positive integer.
        """
        if re.fullmatch(r"\d+", value) is None or int(value) <= 0:
            raise MapParseError(
                line_number, f"{key} must be a positive integer"
            )
        return int(value)
