#!/usr/bin/env python3
"""
Convert GPT-generated supervisor JSON (request_supervisor_updates.py output)
into Nadzoru-compatible XML without touching robot_navigation.cpp.

Typical workflow:
  1. Run request_supervisor_updates.py with --save <file>.json to capture GPT output.
  2. Run this script with --json <file>.json [additional flags] to emit XML.
"""

from __future__ import annotations

import argparse
import io
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

DEFAULT_CONTROLLABLE = [
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    # "random_walk",
]

DEFAULT_UNCONTROLLABLE = [
    "path_clear",
    "obstacle_front",
    "obstacle_left",
    "obstacle_right",
]


def _parse_transition_line(line: str) -> Dict[str, str]:
    pattern = r'\("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)'
    match = re.search(pattern, line.strip())
    if not match:
        raise ValueError(f"Unable to parse transition line: {line}")
    src, event, dst = match.groups()
    return {"source": src, "event": event, "target": dst} 


def _collect_states(
    explicit_states: Sequence[str],
    new_states: Sequence[str] | None,
    transitions: Sequence[Dict[str, str]],
) -> List[str]:
    ordered: List[str] = []
    seen = set()

    def add(name: str) -> None:
        if name and name not in seen:
            ordered.append(name)
            seen.add(name)

    for name in explicit_states:
        add(name)
    if new_states:
        for name in new_states:
            add(name)
    for t in transitions:
        add(t["source"])
        add(t["target"])

    return ordered


def _states_with_flags(
    state_names: Sequence[str],
    initial_states: Iterable[str],
    marked_states: Iterable[str],
) -> List[Dict[str, str]]:
    init_set = set(initial_states)
    marked_set = set(marked_states)
    states: List[Dict[str, str]] = []
    for idx, name in enumerate(state_names):
        states.append(
            {
                "id": str(idx),
                "name": name,
                "initial": "True" if name in init_set else "False",
                "marked": "True" if name in marked_set else "False",
                "x": str(150 + 200 * (idx % 3)),
                "y": str(200 + 80 * (idx // 3)),
            }
        )
    return states


def _event_entries(
    controllable: Sequence[str],
    uncontrollable: Sequence[str],
    transitions: Sequence[Dict[str, str]],
) -> List[Dict[str, str]]:
    ctrl_set = set(controllable)
    unctrl_set = set(uncontrollable)
    used_events = [t["event"] for t in transitions]

    ordered: List[str] = []
    seen = set()
    for name in list(controllable) + list(uncontrollable):
        if name in used_events and name not in seen:
            ordered.append(name)
            seen.add(name)
    for name in used_events:
        if name not in seen:
            ordered.append(name)
            seen.add(name)

    events: List[Dict[str, str]] = []
    for idx, name in enumerate(ordered):
        if name in ctrl_set:
            ctrl_flag = "True"
        elif name in unctrl_set:
            ctrl_flag = "False"
        else:
            ctrl_flag = "True"
        events.append(
            {
                "id": str(idx),
                "name": name,
                "controllable": ctrl_flag,
                "observable": "True",
            }
        )
    return events


def _build_xml(
    supervisor_id: str,
    states: Sequence[Dict[str, str]],
    events: Sequence[Dict[str, str]],
    transitions: Sequence[Dict[str, str]],
) -> bytes:
    model = ET.Element("model", version="0.0", type="FSA", id=supervisor_id)
    data = ET.SubElement(model, "data")

    state_ids = {s["name"]: s["id"] for s in states}
    event_ids = {e["name"]: e["id"] for e in events}

    for state in states:
        ET.SubElement(
            data,
            "state",
            id=state["id"],
            name=state["name"],
            initial=state["initial"],
            marked=state["marked"],
            x=state["x"],
            y=state["y"],
        )

    for event in events:
        ET.SubElement(
            data,
            "event",
            id=event["id"],
            name=event["name"],
            controllable=event["controllable"],
            observable=event["observable"],
        )

    for transition in transitions:
        src = state_ids.get(transition["source"], transition["source"])
        tgt = state_ids.get(transition["target"], transition["target"])
        evt = event_ids.get(transition["event"])
        if evt is None:
            raise ValueError(f"Missing event id for {transition['event']}")
        ET.SubElement(
            data,
            "transition",
            source=str(src),
            target=str(tgt),
            event=str(evt),
        )

    tree = ET.ElementTree(model)
    ET.indent(tree, space="  ", level=0)
    buf = io.BytesIO()
    tree.write(buf, encoding="utf-8", xml_declaration=True)
    return buf.getvalue() + b"\n"


def convert_json_to_xml(args: argparse.Namespace) -> None:
    payload = json.loads(Path(args.json).read_text(encoding="utf-8"))
    transition_lines = payload.get("transitions")
    if not transition_lines:
        raise SystemExit("JSON file does not contain a 'transitions' list.")

    transitions = [_parse_transition_line(line) for line in transition_lines]
    state_names = _collect_states(args.states or [], payload.get("new_states"), transitions)
    if not state_names:
        raise SystemExit("No states detected. Provide --states to seed the list.")

    initial = args.initial or state_names[:1]
    marked = args.marked or state_names
    missing_init = set(initial) - set(state_names)
    missing_marked = set(marked) - set(state_names)
    if missing_init:
        raise SystemExit(f"Initial states not found: {sorted(missing_init)}")
    if missing_marked:
        raise SystemExit(f"Marked states not found: {sorted(missing_marked)}")

    states = _states_with_flags(state_names, initial, marked)
    events = _event_entries(
        args.controllable or DEFAULT_CONTROLLABLE,
        args.uncontrollable or DEFAULT_UNCONTROLLABLE,
        transitions,
    )
    xml_payload = _build_xml(args.supervisor_id, states, events, transitions)
    Path(args.output).write_bytes(xml_payload)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Convert GPT-generated supervisor JSON into Nadzoru XML."
    )
    parser.add_argument("--json", required=True, help="Path to GPT response JSON.")
    parser.add_argument("--output", required=True, help="Destination XML file.")
    parser.add_argument(
        "--states",
        nargs="+",
        default=[],
        help="Base state names (existing DES modes).",
    )
    parser.add_argument(
        "--initial",
        nargs="+",
        help="Initial state names (default: first detected state).",
    )
    parser.add_argument(
        "--marked",
        nargs="+",
        help="Marked/accepting state names (default: all states).",
    )
    parser.add_argument(
        "--controllable",
        nargs="+",
        help="Controllable event names (default: standard list).",
    )
    parser.add_argument(
        "--uncontrollable",
        nargs="+",
        help="Uncontrollable event names (default: standard list).",
    )
    parser.add_argument(
        "--supervisor-id",
        default="Sup_reactive_motion",
        help="Identifier used in the XML <model id=...> attribute.",
    )
    args = parser.parse_args(argv)
    convert_json_to_xml(args)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
