*This project has been created as part of the 42 curriculum by the_inadequate.*

# Fly-in

## Description

Fly-in routes a fleet of drones from a start hub to an end hub through a
capacity-constrained network. It parses the assignment's text map format,
plans turn-by-turn movement, and emits the required movement log. Blocked,
restricted, and priority zones affect routing; zone and connection capacities
are enforced.

## Instructions

The runtime uses only the Python 3.10+ standard library. Python 3.10 or newer
must be installed.

```sh
python3 main.py example.map
python3 main.py example.map --visual
```

Movement lines are written to standard output in the assignment format. The
optional visual view is written to standard error, so the movement log remains
easy to redirect or parse. Use `--no-color` with `--visual` to disable ANSI
colors.

The Makefile provides the required targets:

```sh
make run
make debug
make lint
make lint-strict
make clean
```

`make install` installs the development tools `flake8` and `mypy`. They are not
needed to run the simulator.

## Map format

The first declaration is `nb_drones: <positive integer>`. Define one
`start_hub`, one `end_hub`, and any number of `hub` zones using
`<name> <x> <y>`. Optional zone metadata includes `zone=normal|blocked|
restricted|priority`, `color=<word>`, and `max_drones=<positive integer>`.
Connections use `connection: <zone1>-<zone2>` with optional
`max_link_capacity=<positive integer>`. Names cannot contain spaces or dashes.
Lines beginning with `#`, and text following `#`, are comments. Connections
must refer to zones already declared.

Example output for a two-drone, single-link map:

```text
D1-mid
D1-goal D2-mid
D2-goal
```

Use this matching input:

```text
nb_drones: 2
start_hub: start 0 0
hub: mid 1 0
end_hub: goal 2 0
connection: start-mid
connection: mid-goal
```

## Algorithm and implementation

The parser creates typed `Zone`, `Connection`, and `MapDefinition` objects and
reports syntax or validation errors with source line numbers. It rejects
unknown metadata, malformed capacities, duplicate or forward-referenced
connections, and invalid zone types. A disconnected map is reported as an
unsolvable map rather than a parser error.

The simulator plans drones in identifier order. For each drone, a
time-expanded Dijkstra search considers movement and waiting states. It
minimizes arrival turns first, then prefers routes through priority zones,
fewer waits, and fewer edges. Earlier drone plans reserve zone occupancy and
connection use by turn. New plans may use a zone in a turn when an earlier
drone's reservation shows that it has already moved out in that turn. A
restricted destination takes two turns: the log shows the drone on the
connection for its transit turn and at the destination on the following turn.
The destination must have room at arrival because a drone cannot wait on the
connection.

For `V` zones, `E` connections, `D` drones, and a schedule horizon `H`, one
time-expanded search takes `O(H (V + E) log(HV))` time and `O(HV)` search
memory. Sequential reservation planning repeats that search for each drone.
The implementation caches no graph paths: each search uses the latest
capacity reservations, which allows it to adapt to congestion and select
alternate routes.

## Visualization

`--visual` prints the connection list and a per-turn zone view to standard
error. Each zone shows its coordinates, type, capacity, and current drone
occupants. Recognized color names from map metadata are rendered with ANSI
colors in a terminal. Drones in restricted transit are listed under their
connection. This view helps trace waiting, bottlenecks, and deliveries without
changing the required standard-output movement format.

## Resources and AI use

- Python documentation: <https://docs.python.org/3/>
- `argparse`: <https://docs.python.org/3/library/argparse.html>
- `heapq`: <https://docs.python.org/3/library/heapq.html>
- 42 Fly-in assignment subject, version 2.0.

AI was used to help translate the subject into a design, draft the parser,
planner, command-line interface, visualization, and documentation, and review
edge cases. The project owner should read and understand the code, run the
provided checks, and review it with peers before submission, as the subject
requires being able to explain and modify the implementation.
