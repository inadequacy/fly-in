"""Time-aware route planning and capacity-safe simulation scheduling."""

from __future__ import annotations

import heapq
from collections import defaultdict
from dataclasses import dataclass, field
from itertools import count

from flyin.domain import Connection, MapDefinition, ZoneKind


class NoRouteError(ValueError):
    """Raised when the map has no usable route from start to end."""


@dataclass(frozen=True, slots=True)
class RouteAction:
    """One wait or movement selected by the time-expanded planner."""

    kind: str
    source: str
    destination: str
    start_time: int
    arrival_time: int
    connection: Connection | None = None


@dataclass(slots=True)
class DronePlan:
    """Scheduled states and movement tokens for one drone."""

    drone_id: int
    states: dict[int, str]
    movements: dict[int, str]
    finish_time: int
    link_uses: dict[int, int] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class Turn:
    """Movements that happen during one simulation turn."""

    number: int
    movements: tuple[str, ...]


@dataclass(slots=True)
class SimulationResult:
    """All drone plans and formatted simulation turns."""

    plans: list[DronePlan]
    turns: list[Turn]

    @property
    def turn_count(self) -> int:
        """Return turns through the last delivery.

        Returns:
            int: Number of simulation turn lines.
        """
        return len(self.turns)


@dataclass(slots=True)
class ReservationTable:
    """Occupancy and link use already committed by earlier drones."""

    zone_occupancy: dict[str, dict[int, int]] = field(
        default_factory=lambda: defaultdict(dict)
    )
    link_use: dict[int, dict[int, int]] = field(
        default_factory=lambda: defaultdict(dict)
    )
    latest_time: int = 0

    def zone_count(self, zone_name: str, time: int) -> int:
        """Return planned occupants of a zone after ``time``.

        Args:
            zone_name: Zone to look up.
            time: Turn whose resulting state is checked.

        Returns:
            int: Number of earlier drones reserved there.
        """
        return self.zone_occupancy.get(zone_name, {}).get(time, 0)

    def link_count(self, connection_id: int, time: int) -> int:
        """Return scheduled uses of a connection during ``time``.

        Args:
            connection_id: Stable connection index.
            time: Turn in which use is checked.

        Returns:
            int: Number of earlier drones using the link.
        """
        return self.link_use.get(connection_id, {}).get(time, 0)

    def reserve(self, plan: DronePlan, graph: MapDefinition) -> None:
        """Commit a drone's zones and link uses to the shared schedule.

        Args:
            plan: Completed drone schedule.
            graph: Map whose end zone is unbounded and untracked.

        Returns:
            None: Reservations are updated in place.
        """
        for time, location in plan.states.items():
            if location in graph.zones and location != graph.end_name:
                bucket = self.zone_occupancy[location]
                bucket[time] = bucket.get(time, 0) + 1
            self.latest_time = max(self.latest_time, time)
        for time, connection_id in plan.link_uses.items():
            bucket = self.link_use[connection_id]
            bucket[time] = bucket.get(time, 0) + 1
            self.latest_time = max(self.latest_time, time)


class Simulator:
    """Schedule drones with a time-expanded, reservation-aware search."""

    def __init__(self, graph: MapDefinition) -> None:
        """Create a simulator for one parsed map.

        Args:
            graph: Validated map and its capacity metadata.
        """
        self.graph = graph
        self.reservations = ReservationTable()

    def run(self) -> SimulationResult:
        """Plan every drone and return the complete schedule.

        Returns:
            SimulationResult: Drone plans and movement lines.

        Raises:
            NoRouteError: If no usable start-to-end schedule exists.
        """
        if not self.graph.has_route():
            raise NoRouteError(
                f"no path connects start zone {self.graph.start_name!r} "
                f"to end zone {self.graph.end_name!r}"
            )
        plans: list[DronePlan] = []
        for drone_id in range(1, self.graph.drone_count + 1):
            plan = self._plan_drone(drone_id)
            self.reservations.reserve(plan, self.graph)
            plans.append(plan)
        final_time = max((plan.finish_time for plan in plans), default=0)
        turns: list[Turn] = []
        for time in range(1, final_time + 1):
            movements = tuple(
                plan.movements[time]
                for plan in plans
                if time in plan.movements
            )
            turns.append(Turn(time, movements))
        return SimulationResult(plans, turns)

    def _plan_drone(self, drone_id: int) -> DronePlan:
        """Find the earliest feasible route for one additional drone.

        Args:
            drone_id: One-based identifier used in the movement log.

        Returns:
            DronePlan: Route and waits that respect existing reservations.

        Raises:
            NoRouteError: If no feasible schedule is found in the horizon.
        """
        horizon = self.reservations.latest_time + 2 * len(self.graph.zones) + 2
        start_state = (self.graph.start_name, 0)
        serial = count()
        start_score = (0, 0, 0)
        queue: list[tuple[int, int, int, int, int, str]] = [
            (0, *start_score, next(serial), self.graph.start_name)
        ]
        best: dict[tuple[str, int], tuple[int, int, int]] = {
            start_state: start_score
        }
        previous: dict[
            tuple[str, int], tuple[tuple[str, int], RouteAction]
        ] = {}
        goal_state: tuple[str, int] | None = None

        while queue:
            time, non_priority, waits, hops, _, zone_name = heapq.heappop(
                queue
            )
            state = (zone_name, time)
            score = (non_priority, waits, hops)
            if best.get(state) != score:
                continue
            if zone_name == self.graph.end_name:
                goal_state = state
                break

            if time < horizon:
                next_time = time + 1
                if self._zone_available(zone_name, next_time):
                    action = RouteAction(
                        "wait", zone_name, zone_name, time, next_time
                    )
                    self._relax(
                        (zone_name, next_time), state, action,
                        (non_priority, waits + 1, hops), best, previous, queue,
                        serial,
                    )

            for connection in self.graph.adjacency[zone_name]:
                neighbor = connection.other(zone_name)
                destination = self.graph.zones[neighbor]
                if destination.kind is ZoneKind.BLOCKED:
                    continue
                cost = destination.kind.movement_cost
                arrival_time = time + cost
                link_time = time + 1
                if arrival_time > horizon:
                    continue
                if not self._link_available(connection, link_time):
                    continue
                if not self._zone_available(neighbor, arrival_time):
                    continue
                action_kind = "restricted" if cost == 2 else "move"
                action = RouteAction(
                    action_kind, zone_name, neighbor, time, arrival_time,
                    connection,
                )
                priority_penalty = (
                    0 if destination.kind is ZoneKind.PRIORITY else 1
                )
                self._relax(
                    (neighbor, arrival_time), state, action,
                    (non_priority + priority_penalty, waits, hops + 1),
                    best, previous, queue, serial,
                )

        if goal_state is None:
            raise NoRouteError(
                f"no capacity-feasible schedule found for drone D{drone_id}"
            )
        return self._build_plan(drone_id, goal_state, previous)

    def _zone_available(self, zone_name: str, time: int) -> bool:
        """Check whether a candidate can occupy a zone after a turn.

        Args:
            zone_name: Candidate destination or waiting zone.
            time: Turn whose resulting occupancy is checked.

        Returns:
            bool: True when existing reservations leave one place.
        """
        capacity = self.graph.zones[zone_name].capacity
        return (
            capacity is None
            or self.reservations.zone_count(zone_name, time) < capacity
        )

    def _link_available(self, connection: Connection, time: int) -> bool:
        """Check whether a candidate can use a connection in a turn.

        Args:
            connection: Link the drone would traverse.
            time: Turn in which the traversal starts.

        Returns:
            bool: True when existing link use is below its capacity.
        """
        return (
            self.reservations.link_count(connection.identifier, time)
            < connection.max_drones
        )

    @staticmethod
    def _relax(
        next_state: tuple[str, int],
        previous_state: tuple[str, int],
        action: RouteAction,
        new_score: tuple[int, int, int],
        best: dict[tuple[str, int], tuple[int, int, int]],
        previous: dict[tuple[str, int], tuple[tuple[str, int], RouteAction]],
        queue: list[tuple[int, int, int, int, int, str]],
        serial: count[int],
    ) -> None:
        """Record an improved time-state and queue it for exploration.

        Args:
            next_state: Zone and arrival turn under consideration.
            previous_state: State from which it was reached.
            action: Wait or movement that connects the states.
            new_score: Tie-break score for the same zone and time.
            best: Best known scores by time-state.
            previous: Predecessor actions for route reconstruction.
            queue: Priority queue of states to explore.
            serial: Monotonic tie-breaker for stable queue ordering.

        Returns:
            None: Search structures are updated in place.
        """
        old = best.get(next_state)
        if old is not None and old <= new_score:
            return
        best[next_state] = new_score
        previous[next_state] = (previous_state, action)
        zone_name, time = next_state
        non_priority, waits, hops = new_score
        heapq.heappush(
            queue,
            (time, non_priority, waits, hops, next(serial), zone_name),
        )

    def _build_plan(
        self,
        drone_id: int,
        goal_state: tuple[str, int],
        previous: dict[tuple[str, int], tuple[tuple[str, int], RouteAction]],
    ) -> DronePlan:
        """Reconstruct the timed states and output tokens for a drone.

        Args:
            drone_id: One-based identifier used in output tokens.
            goal_state: End zone and arrival turn.
            previous: Predecessor actions recorded by the search.

        Returns:
            DronePlan: Reconstructed route, occupancy, and movements.
        """
        actions: list[RouteAction] = []
        state = goal_state
        while state in previous:
            prior_state, action = previous[state]
            actions.append(action)
            state = prior_state
        actions.reverse()

        states: dict[int, str] = {0: self.graph.start_name}
        movements: dict[int, str] = {}
        link_uses: dict[int, int] = {}
        for action in actions:
            if action.kind == "wait":
                states[action.arrival_time] = action.source
                continue
            if action.connection is None:
                raise RuntimeError("movement action is missing its connection")
            link_uses[action.start_time + 1] = action.connection.identifier
            if action.kind == "restricted":
                connector_time = action.start_time + 1
                states[connector_time] = action.connection.name
                movements[connector_time] = (
                    f"D{drone_id}-{action.connection.name}"
                )
                states[action.arrival_time] = action.destination
                movements[action.arrival_time] = (
                    f"D{drone_id}-{action.destination}"
                )
            else:
                states[action.arrival_time] = action.destination
                movements[action.arrival_time] = (
                    f"D{drone_id}-{action.destination}"
                )

        return DronePlan(drone_id, states, movements, goal_state[1], link_uses)
