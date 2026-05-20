#!/usr/bin/env python3
"""Small automaton loader/executor for Nadzoru-style supervisors."""

from __future__ import annotations

import json
import random
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SWARM_BASICS_SRC = REPO_ROOT / "swarm_basics"
if str(SWARM_BASICS_SRC) not in sys.path:
    sys.path.insert(0, str(SWARM_BASICS_SRC))

from swarm_basics.sct import SCT


DEFAULT_CONTROLLABLE_EVENTS = {
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    "stop",
    "go_to_red",
    "escape_blue",
    "push",
}


def normalize_event(name: object) -> str:
    text = str(name)
    return text[3:] if text.startswith("EV_") else text


@dataclass(frozen=True)
class Transition:
    source: str
    event: str
    target: str


class Automaton:
    """A deterministic finite-state supervisor with controllable events."""

    def __init__(
        self,
        states: Sequence[str],
        initial_state: str,
        controllable_events: Iterable[str],
        transitions: Sequence[Transition],
        name: str = "automaton",
    ) -> None:
        if initial_state not in states:
            raise ValueError(f"Initial state '{initial_state}' is not in the state list.")

        self.name = name
        self.states = list(states)
        self.initial_state = initial_state
        self.current_state = initial_state
        self.controllable_events = {normalize_event(e) for e in controllable_events}
        self.transitions: Dict[tuple[str, str], str] = {}

        for transition in transitions:
            key = (transition.source, normalize_event(transition.event))
            self.transitions[key] = transition.target

    @property
    def enabled_controllables(self) -> List[str]:
        events = [
            event
            for state, event in self.transitions
            if state == self.current_state and event in self.controllable_events
        ]
        return sorted(events)

    def reset(self) -> None:
        self.current_state = self.initial_state

    def accepts(self, event: str) -> bool:
        return (self.current_state, normalize_event(event)) in self.transitions

    def apply(self, event: str) -> bool:
        key = (self.current_state, normalize_event(event))
        target = self.transitions.get(key)
        if target is None:
            return False
        self.current_state = target
        return True

    def observe(self, events: Iterable[str]) -> List[str]:
        applied = []
        for event in events:
            if self.apply(event):
                applied.append(normalize_event(event))
        return applied

    def choose_controllable(self, rng: random.Random) -> str | None:
        enabled = self.enabled_controllables
        if not enabled:
            return None
        return rng.choice(enabled)


class SCTAutomaton:
    """Adapter that runs supervisors through swarm_basics.swarm_basics.sct.SCT."""

    def __init__(self, spec: Automaton) -> None:
        self.name = spec.name
        self.state_names = list(spec.states)
        self._latest_perception: str | None = None
        yaml_payload = build_sct_yaml(spec)

        with tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".yaml",
            prefix="basic_sim_sct_",
            delete=False,
            encoding="utf-8",
        ) as tmp:
            yaml.safe_dump(yaml_payload, tmp, sort_keys=False)
            tmp_path = Path(tmp.name)

        try:
            self.sct = SCT(str(tmp_path))
        finally:
            tmp_path.unlink(missing_ok=True)

        self.event_by_index = {idx: normalize_event(name) for name, idx in self.sct.EV.items()}
        self.index_by_event = {name: idx for idx, name in self.event_by_index.items()}
        self.controllable_events = {
            self.event_by_index[idx]
            for idx, flag in enumerate(self.sct.ev_controllable)
            if flag
        }
        self._install_uncontrollable_callbacks()

    @property
    def current_state(self) -> str:
        if not self.sct.sup_current_state:
            return "unknown"
        idx = self.sct.sup_current_state[0]
        if 0 <= idx < len(self.state_names):
            return self.state_names[idx]
        return f"q{idx}"

    def run_step(self, perception: str) -> str | None:
        self._latest_perception = normalize_event(perception)
        self.sct.input_buffer = []
        exists, event_idx = self.sct.run_step()
        if not exists or event_idx is None:
            return None
        return self.event_by_index[event_idx]

    def _install_uncontrollable_callbacks(self) -> None:
        for event, idx in self.index_by_event.items():
            if self.sct.ev_controllable[idx]:
                continue
            self.sct.add_callback(idx, None, self._perception_matches(event), None)

    def _perception_matches(self, event: str):
        def check(_sup_data: object) -> bool:
            return self._latest_perception == event

        return check


def load_automaton(path: str | Path) -> SCTAutomaton:
    source = Path(path)
    suffix = source.suffix.lower()
    if suffix == ".xml":
        return SCTAutomaton(load_nadzoru_xml(source))
    if suffix == ".json":
        return SCTAutomaton(load_json_supervisor(source))
    if suffix in {".yaml", ".yml"}:
        return SCTAutomaton(load_compact_yaml(source))
    raise ValueError(f"Unsupported automaton format '{source.suffix}' for {source}.")


def build_sct_yaml(spec: Automaton) -> Dict[str, object]:
    event_names = sorted(
        {transition.event for transition in spec_transitions(spec)} | set(spec.controllable_events)
    )
    events = [f"EV_{name}" for name in event_names]
    ev_controllable = [1 if name in spec.controllable_events else 0 for name in event_names]
    event_index = {name: idx for idx, name in enumerate(event_names)}
    state_index = {state: idx for idx, state in enumerate(spec.states)}

    outgoing: Dict[str, List[Transition]] = {state: [] for state in spec.states}
    for transition in spec_transitions(spec):
        outgoing.setdefault(transition.source, []).append(transition)

    sup_data: List[object] = []
    for state in spec.states:
        transitions = sorted(outgoing.get(state, []), key=lambda item: (item.event, item.target))
        sup_data.append(len(transitions))
        for transition in transitions:
            target_idx = state_index[transition.target]
            sup_data.append(f"EV_{transition.event}")
            sup_data.append(target_idx // 256)
            sup_data.append(target_idx % 256)

    return {
        "num_events": len(events),
        "num_supervisors": 1,
        "events": events,
        "ev_controllable": ev_controllable,
        "sup_events": [[1] * len(events)],
        "sup_init_state": [state_index[spec.initial_state]],
        "sup_current_state": [state_index[spec.initial_state]],
        "sup_data_pos": [0],
        "sup_data": sup_data,
    }


def spec_transitions(spec: Automaton) -> List[Transition]:
    return [
        Transition(source, event, target)
        for (source, event), target in spec.transitions.items()
    ]


def load_nadzoru_xml(path: Path) -> Automaton:
    root = ET.parse(path).getroot()
    data = root.find("data")
    if data is None:
        raise ValueError(f"{path} does not contain a <data> section.")

    states_by_id: Dict[str, str] = {}
    initial_state = None
    for state in data.findall("state"):
        state_id = required_attr(state, "id", path)
        name = required_attr(state, "name", path)
        states_by_id[state_id] = name
        if state.attrib.get("initial", "").strip().lower() == "true":
            initial_state = name

    events_by_id: Dict[str, str] = {}
    controllable = set()
    for event in data.findall("event"):
        event_id = required_attr(event, "id", path)
        name = normalize_event(required_attr(event, "name", path))
        events_by_id[event_id] = name
        if event.attrib.get("controllable", "").strip().lower() == "true":
            controllable.add(name)

    transitions = []
    for transition in data.findall("transition"):
        source = states_by_id[required_attr(transition, "source", path)]
        target = states_by_id[required_attr(transition, "target", path)]
        event = events_by_id[required_attr(transition, "event", path)]
        transitions.append(Transition(source, event, target))

    if initial_state is None:
        raise ValueError(f"{path} has no initial state.")

    return Automaton(
        states=list(states_by_id.values()),
        initial_state=initial_state,
        controllable_events=controllable,
        transitions=transitions,
        name=path.stem,
    )


def load_json_supervisor(path: Path) -> Automaton:
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw_transitions = payload.get("transitions")
    if not isinstance(raw_transitions, list) or not raw_transitions:
        raise ValueError(f"{path} must contain a non-empty 'transitions' list.")

    transitions = [_parse_json_transition(item) for item in raw_transitions]
    states = _ordered_states(payload.get("states", []), payload.get("new_states", []), transitions)
    initial = payload.get("initial_state") or (states[0] if states else None)
    if initial is None:
        raise ValueError(f"{path} does not define any states.")

    events = {transition.event for transition in transitions}
    controllable = payload.get("controllable_events")
    if controllable is None:
        controllable = sorted(events & DEFAULT_CONTROLLABLE_EVENTS)

    return Automaton(
        states=states,
        initial_state=initial,
        controllable_events=controllable,
        transitions=transitions,
        name=path.stem,
    )


def load_compact_yaml(path: Path) -> Automaton:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not payload:
        raise ValueError(f"{path} is empty or invalid.")
    if payload.get("num_supervisors", 1) != 1:
        raise ValueError("The basic simulator supports one supervisor per automaton file.")

    events = [normalize_event(event) for event in payload["events"]]
    controllable = [
        event for event, flag in zip(events, payload["ev_controllable"]) if int(flag) == 1
    ]

    sup_data = payload["sup_data"]
    start = int(payload.get("sup_data_pos", [0])[0])
    states = [f"q{i}" for i in range(_count_yaml_states(sup_data, start))]
    initial_idx = int(payload.get("sup_init_state", [0])[0])
    transitions = []

    pos = start
    for state_idx, source in enumerate(states):
        transition_count = int(sup_data[pos])
        pos += 1
        for _ in range(transition_count):
            event = normalize_event(sup_data[pos])
            target_idx = int(sup_data[pos + 1]) * 256 + int(sup_data[pos + 2])
            transitions.append(Transition(source, event, states[target_idx]))
            pos += 3

    return Automaton(
        states=states,
        initial_state=states[initial_idx],
        controllable_events=controllable,
        transitions=transitions,
        name=path.stem,
    )


def required_attr(element: ET.Element, attr: str, path: Path) -> str:
    value = element.attrib.get(attr)
    if value is None:
        raise ValueError(f"{path}: <{element.tag}> is missing '{attr}'.")
    return value


def _parse_json_transition(item: object) -> Transition:
    if isinstance(item, (list, tuple)) and len(item) == 3:
        source, event, target = item
        return Transition(str(source), normalize_event(event), str(target))

    if isinstance(item, dict):
        return Transition(
            str(item["source"]),
            normalize_event(item["event"]),
            str(item["target"]),
        )

    if isinstance(item, str):
        match = re.search(r'\("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)', item.strip())
        if match:
            source, event, target = match.groups()
            return Transition(source, normalize_event(event), target)

    raise ValueError(f"Could not parse transition: {item!r}")


def _ordered_states(
    explicit: Sequence[str],
    new_states: Sequence[str],
    transitions: Sequence[Transition],
) -> List[str]:
    ordered = []
    seen = set()

    def add(state: str) -> None:
        if state and state not in seen:
            ordered.append(state)
            seen.add(state)

    for state in explicit:
        add(str(state))
    for transition in transitions:
        add(transition.source)
        add(transition.target)
    for state in new_states:
        add(str(state))
    return ordered


def _count_yaml_states(sup_data: Sequence[object], start: int) -> int:
    count = 0
    pos = start
    while pos < len(sup_data):
        transition_count = int(sup_data[pos])
        pos += 1 + transition_count * 3
        count += 1
    return count
