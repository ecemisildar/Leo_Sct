#!/usr/bin/env python3
"""Grid world primitives for SCT-controlled agents."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

from automata import SCTAutomaton


Position = Tuple[int, int]

HEADINGS = ("N", "E", "S", "W")
DELTAS = {
    "N": (0, -1),
    "E": (1, 0),
    "S": (0, 1),
    "W": (-1, 0),
}


def turn_left(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) - 1) % len(HEADINGS)]


def turn_right(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 1) % len(HEADINGS)]


@dataclass
class GridEnvironment:
    width: int
    height: int
    obstacles: set[Position] = field(default_factory=set)
    red_area: set[Position] = field(default_factory=set)
    blue_area: set[Position] = field(default_factory=set)
    objects: set[Position] = field(default_factory=set)

    def in_bounds(self, position: Position) -> bool:
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def blocked(
        self,
        position: Position,
        agents: Iterable["GridAgent"] = (),
        include_objects: bool = True,
    ) -> bool:
        occupied = {agent.position for agent in agents}
        return (
            not self.in_bounds(position)
            or position in self.obstacles
            or position in occupied
            or (include_objects and position in self.objects)
        )

    def collision_type(self, position: Position, agents: Iterable["GridAgent"] = ()) -> str | None:
        collision = self.collision_info(position, agents)
        return collision[0] if collision else None

    def collision_info(
        self,
        position: Position,
        agents: Iterable["GridAgent"] = (),
    ) -> tuple[str, str] | None:
        occupied = {agent.position: agent for agent in agents}
        if not self.in_bounds(position):
            return "wall", "wall"
        if position in self.obstacles:
            return "obstacle", f"obstacle@{position}"
        if position in self.objects:
            return "object", f"object@{position}"
        agent = occupied.get(position)
        if agent is not None:
            return "agent", agent.name
        return None

    def neighbor(self, position: Position, heading: str) -> Position:
        dx, dy = DELTAS[heading]
        return position[0] + dx, position[1] + dy

    def in_red_area(self, position: Position) -> bool:
        return position in self.red_area

    def in_blue_area(self, position: Position) -> bool:
        return position in self.blue_area

    def step_toward_red(self, position: Position, agents: Iterable["GridAgent"]) -> Position | None:
        if not self.red_area or position in self.red_area:
            return None

        other_agents = list(agents)
        candidates = []
        for heading in HEADINGS:
            candidate = self.neighbor(position, heading)
            if self.blocked(candidate, other_agents):
                continue
            distance = min(
                abs(candidate[0] - red_x) + abs(candidate[1] - red_y)
                for red_x, red_y in self.red_area
            )
            candidates.append((distance, heading, candidate))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], HEADINGS.index(item[1])))[2]

    def all_objects_in_red(self) -> bool:
        return bool(self.objects) and all(position in self.red_area for position in self.objects)

    def step_toward_nearest_object(
        self,
        position: Position,
        agents: Iterable["GridAgent"],
    ) -> Position | None:
        if not self.objects:
            return None

        other_agents = list(agents)
        candidates = []
        for heading in HEADINGS:
            candidate = self.neighbor(position, heading)
            if self.blocked(candidate, other_agents):
                continue
            distance = min(
                abs(candidate[0] - obj_x) + abs(candidate[1] - obj_y)
                for obj_x, obj_y in self.objects
            )
            candidates.append((distance, heading, candidate))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], HEADINGS.index(item[1])))[2]

    def step_toward_red_for_object(self, position: Position) -> Position | None:
        if not self.red_area or position in self.red_area:
            return None

        candidates = []
        for heading in HEADINGS:
            candidate = self.neighbor(position, heading)
            if (
                not self.in_bounds(candidate)
                or candidate in self.obstacles
                or candidate in self.objects
            ):
                continue
            distance = min(
                abs(candidate[0] - red_x) + abs(candidate[1] - red_y)
                for red_x, red_y in self.red_area
            )
            candidates.append((distance, heading, candidate))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], HEADINGS.index(item[1])))[2]

    def step_toward_push_position(
        self,
        object_pos: Position,
        robot_pos: Position,
        agents: Iterable["GridAgent"],
    ) -> Position | None:
        occupied_by_agents = {agent.position for agent in agents}
        candidates = []
        for heading in HEADINGS:
            push_from = (
                object_pos[0] - DELTAS[heading][0],
                object_pos[1] - DELTAS[heading][1],
            )
            object_target = self.neighbor(object_pos, heading)
            if (
                not self.in_bounds(push_from)
                or push_from in self.obstacles
                or push_from in self.objects
                or push_from in occupied_by_agents
            ):
                continue
            if (
                not self.in_bounds(object_target)
                or object_target in self.obstacles
                or object_target in self.objects
                or object_target == robot_pos
            ):
                continue
            red_distance = (
                min(
                    abs(object_target[0] - red_x) + abs(object_target[1] - red_y)
                    for red_x, red_y in self.red_area
                )
                if self.red_area
                else 0
            )
            robot_distance = abs(push_from[0] - robot_pos[0]) + abs(push_from[1] - robot_pos[1])
            candidates.append((red_distance, robot_distance, HEADINGS.index(heading), push_from))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], item[1], item[2]))[3]

    def step_toward_position(
        self,
        position: Position,
        target: Position,
        agents: Iterable["GridAgent"],
    ) -> Position | None:
        candidates = []
        for heading in HEADINGS:
            candidate = self.neighbor(position, heading)
            if self.blocked(candidate, agents):
                continue
            distance = abs(candidate[0] - target[0]) + abs(candidate[1] - target[1])
            candidates.append((distance, heading, candidate))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], HEADINGS.index(item[1])))[2]

    def best_push_position(
        self,
        robot_pos: Position,
        agents: Iterable["GridAgent"],
    ) -> Position | None:
        candidates = []
        for object_pos in self.objects:
            push_from = self.step_toward_push_position(object_pos, robot_pos, agents)
            if push_from is None:
                continue
            robot_distance = abs(push_from[0] - robot_pos[0]) + abs(push_from[1] - robot_pos[1])
            object_distance = (
                min(
                    abs(object_pos[0] - red_x) + abs(object_pos[1] - red_y)
                    for red_x, red_y in self.red_area
                )
                if self.red_area
                else 0
            )
            candidates.append((robot_distance, object_distance, push_from))

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], item[1], item[2]))[2]

    def step_away_from_blue(self, position: Position, agents: Iterable["GridAgent"]) -> Position | None:
        if not self.blue_area:
            return None

        other_agents = list(agents)
        candidates = []
        for heading in HEADINGS:
            candidate = self.neighbor(position, heading)
            if self.blocked(candidate, other_agents):
                continue
            distance = min(
                abs(candidate[0] - blue_x) + abs(candidate[1] - blue_y)
                for blue_x, blue_y in self.blue_area
            )
            outside_bonus = 1 if candidate not in self.blue_area else 0
            candidates.append((outside_bonus, distance, heading, candidate))

        if not candidates:
            return None
        return max(candidates, key=lambda item: (item[0], item[1], -HEADINGS.index(item[2])))[3]

    def render(self, agents: Sequence["GridAgent"]) -> str:
        by_position = {agent.position: agent for agent in agents}
        rows = []
        for y in range(self.height):
            cells = []
            for x in range(self.width):
                position = (x, y)
                if position in by_position:
                    cells.append(by_position[position].glyph)
                elif position in self.objects:
                    cells.append("O")
                elif position in self.obstacles:
                    cells.append("#")
                elif position in self.blue_area:
                    cells.append("B")
                elif position in self.red_area:
                    cells.append("R")
                else:
                    cells.append(".")
            rows.append(" ".join(cells))
        return "\n".join(rows)


@dataclass
class GridAgent:
    name: str
    automaton: SCTAutomaton
    position: Position
    heading: str = "E"
    glyph: str = "A"
    side_sensors: bool = False
    visited: set[Position] = field(default_factory=set)
    collisions: int = 0
    collision_by_type: dict[str, int] = field(
        default_factory=lambda: {"wall": 0, "obstacle": 0, "agent": 0, "object": 0}
    )
    collision_by_event: dict[str, int] = field(default_factory=dict)
    collision_by_hit: dict[str, int] = field(default_factory=dict)
    action_counts: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.visited.add(self.position)

    def perception_event(self, env: GridEnvironment, others: Iterable["GridAgent"]) -> str:
        if env.all_objects_in_red():
            return "detect_object_red"
        if env.in_blue_area(self.position):
            return "blue_detected"
        if not env.objects and env.in_red_area(self.position):
            return "red_detected"

        other_agents = [agent for agent in others if agent is not self]
        front = env.blocked(
            env.neighbor(self.position, self.heading),
            other_agents,
            include_objects=False,
        )
        if not self.side_sensors:
            return "obstacle_front" if front else "path_clear"

        left = env.blocked(
            env.neighbor(self.position, turn_left(self.heading)),
            other_agents,
            include_objects=False,
        )
        right = env.blocked(
            env.neighbor(self.position, turn_right(self.heading)),
            other_agents,
            include_objects=False,
        )

        if front:
            return "obstacle_front"
        if left:
            return "obstacle_left"
        if right:
            return "obstacle_right"
        return "path_clear"

    def step(self, env: GridEnvironment, all_agents: Sequence["GridAgent"], rng: random.Random) -> dict:
        perception = self.perception_event(env, all_agents)
        controllable = self.automaton.run_step(perception)
        moved = False
        collision = None
        collision_event = None
        collision_hit = None

        if controllable is not None:
            self.action_counts[controllable] = self.action_counts.get(controllable, 0) + 1
            moved, collision, collision_hit = self.execute(controllable, env, all_agents, rng)
            if collision is not None:
                collision_event = controllable
        else:
            self.action_counts["none"] = self.action_counts.get("none", 0) + 1

        self.visited.add(self.position)
        return {
            "agent": self.name,
            "state": self.automaton.current_state,
            "perception": perception,
            "observed": [perception],
            "action": controllable,
            "moved": moved,
            "collision": collision,
            "collision_event": collision_event,
            "collision_hit": collision_hit,
            "collisions": self.collisions,
            "position": self.position,
            "heading": self.heading,
        }

    def execute(
        self,
        event: str,
        env: GridEnvironment,
        all_agents: Sequence["GridAgent"],
        rng: random.Random,
    ) -> tuple[bool, str | None, str | None]:
        if event == "rotate_clockwise":
            self.heading = turn_right(self.heading)
            return False, None, None
        if event == "rotate_counterclockwise":
            self.heading = turn_left(self.heading)
            return False, None, None
        if event == "full_rotate":
            self.heading = HEADINGS[(HEADINGS.index(self.heading) + 2) % len(HEADINGS)]
            return False, None, None
        if event == "stop":
            return False, None, None
        if event == "move_forward":
            return self._move(env, all_agents, self.heading, event)
        if event == "move_backward":
            return self._move(env, all_agents, opposite_heading(self.heading), event)
        if event == "go_to_red":
            return self._move_toward_red(env, all_agents, event)
        if event == "escape_blue":
            return self._move_away_from_blue(env, all_agents, event)
        if event == "push":
            return self._push_object(env, all_agents, event)
        return False, None, None

    def _move_toward_red(
        self,
        env: GridEnvironment,
        all_agents: Sequence["GridAgent"],
        event: str,
    ) -> tuple[bool, str | None, str | None]:
        other_agents = [agent for agent in all_agents if agent is not self]
        target = env.step_toward_red(self.position, other_agents)
        if target is None:
            return False, None, None
        self.heading = heading_between(self.position, target)
        self.position = target
        return True, None, None

    def _push_object(
        self,
        env: GridEnvironment,
        all_agents: Sequence["GridAgent"],
        event: str,
    ) -> tuple[bool, str | None, str | None]:
        other_agents = [agent for agent in all_agents if agent is not self]
        front_object = env.neighbor(self.position, self.heading)

        if front_object not in env.objects:
            target = env.best_push_position(self.position, other_agents)
            if target is not None:
                target = (
                    target
                    if abs(target[0] - self.position[0]) + abs(target[1] - self.position[1]) == 1
                    else env.step_toward_position(self.position, target, other_agents)
                )
            if target is None:
                target = env.step_toward_nearest_object(self.position, other_agents)
            if target is None:
                return False, None, None
            if target == self.position:
                adjacent_objects = [
                    obj for obj in env.objects
                    if abs(obj[0] - self.position[0]) + abs(obj[1] - self.position[1]) == 1
                ]
                if not adjacent_objects:
                    return False, None, None
                best_object = min(
                    adjacent_objects,
                    key=lambda obj: (
                        min(
                            abs(obj[0] - red_x) + abs(obj[1] - red_y)
                            for red_x, red_y in env.red_area
                        )
                        if env.red_area
                        else 0,
                        obj,
                    ),
                )
                self.heading = heading_between(self.position, best_object)
                return self._push_object(env, all_agents, event)
            self.heading = heading_between(self.position, target)
            self.position = target
            return True, None, None

        object_target = env.neighbor(front_object, self.heading)
        occupied_by_agents = {agent.position for agent in other_agents}
        if (
            not env.in_bounds(object_target)
            or object_target in env.obstacles
            or object_target in env.objects
            or object_target == self.position
            or object_target in occupied_by_agents
        ):
            target = env.step_toward_push_position(front_object, self.position, other_agents)
            if target is None:
                return False, "object", f"object@{front_object}"
            if abs(target[0] - self.position[0]) + abs(target[1] - self.position[1]) == 1:
                move_target = target
            else:
                move_target = env.step_toward_position(self.position, target, other_agents)
                if move_target is None:
                    return False, "object", f"object@{front_object}"
            self.heading = heading_between(self.position, move_target)
            self.position = move_target
            return True, None, None

        env.objects.remove(front_object)
        env.objects.add(object_target)
        self.position = front_object
        return True, None, None

    def _move_away_from_blue(
        self,
        env: GridEnvironment,
        all_agents: Sequence["GridAgent"],
        event: str,
    ) -> tuple[bool, str | None, str | None]:
        other_agents = [agent for agent in all_agents if agent is not self]
        target = env.step_away_from_blue(self.position, other_agents)
        if target is None:
            return False, None, None
        self.heading = heading_between(self.position, target)
        self.position = target
        return True, None, None

    def _move(
        self,
        env: GridEnvironment,
        all_agents: Sequence["GridAgent"],
        heading: str,
        event: str,
    ) -> tuple[bool, str | None, str | None]:
        other_agents = [agent for agent in all_agents if agent is not self]
        target = env.neighbor(self.position, heading)
        collision = env.collision_info(target, other_agents)
        if collision is not None:
            collision_type, collision_hit = collision
            self.collisions += 1
            self.collision_by_type[collision_type] += 1
            self.collision_by_event[event] = self.collision_by_event.get(event, 0) + 1
            self.collision_by_hit[collision_hit] = self.collision_by_hit.get(collision_hit, 0) + 1
            return False, collision_type, collision_hit
        self.position = target
        return True, None, None


def opposite_heading(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 2) % len(HEADINGS)]


def heading_between(source: Position, target: Position) -> str:
    dx = target[0] - source[0]
    dy = target[1] - source[1]
    for heading, delta in DELTAS.items():
        if delta == (dx, dy):
            return heading
    raise ValueError(f"{target} is not adjacent to {source}.")
