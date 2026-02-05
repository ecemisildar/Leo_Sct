#!/usr/bin/env python3
"""
Convert robot_navigation.cpp supervisors into the Nadzoru XML format.

This script parses the controllable/uncontrollable event vectors, state
declarations, and K.SetTransition rules directly from the DES-oriented
robot_navigation.cpp file.  It then emits a deterministic finite-state
automaton description compatible with Nadzoru so we can stop relying on
libfaudes for serialization.
"""

from __future__ import annotations

import argparse
import io
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_CPP = THIS_DIR / "robot_navigation.cpp"
DEFAULT_OUTPUT = THIS_DIR / "robot_navigation.xml"


def _extract_vector(source: str, var_name: str) -> List[str]:
    """Return the string entries inside the std::vector definition."""
    pattern = rf"{var_name}\s*=\s*\{{(.*?)\}};"
    match = re.search(pattern, source, flags=re.DOTALL)
    if not match:
        raise ValueError(f"Unable to locate definition for {var_name}")
    return re.findall(r'"([^"]+)"', match.group(1))


def _extract_states(source: str) -> List[str]:
    """Return states in order of K.InsState declarations."""
    seen = set()
    ordered: List[str] = []
    for state in re.findall(r'K\.InsState\("([^"]+)"\)', source):
        if state not in seen:
            ordered.append(state)
            seen.add(state)
    if not ordered:
        raise ValueError("No K.InsState(...) declarations found.")
    return ordered


def _extract_initial_states(source: str) -> List[str]:
    states = re.findall(r'K\.SetInitState\("([^"]+)"\)', source)
    if not states:
        raise ValueError("No K.SetInitState(...) calls found.")
    return states


def _extract_marked_states(source: str) -> List[str]:
    states = re.findall(r'K\.SetMarkedState\("([^"]+)"\)', source)
    if not states:
        raise ValueError("No K.SetMarkedState(...) calls found.")
    return states


def _extract_transitions(source: str) -> List[Dict[str, str]]:
    transitions: List[Dict[str, str]] = []
    pattern = r'K\.SetTransition\("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)'
    for state, event, nxt in re.findall(pattern, source):
        transitions.append(
            {
                "source": state,
                "event": event,
                "target": nxt,
            }
        )
    if not transitions:
        raise ValueError("No K.SetTransition entries located.")
    return transitions


def _states_with_flags(
    ordered_states: Sequence[str],
    init_states: Iterable[str],
    marked_states: Iterable[str],
) -> List[Dict[str, str]]:
    init_set = set(init_states)
    marked_set = set(marked_states)
    states: List[Dict[str, str]] = []
    for idx, name in enumerate(ordered_states):
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
    """Return event dicts (id, name, controllable, observable)."""
    ctrl_set = set(controllable)
    unctrl_set = set(uncontrollable)

    # Only include events that appear anywhere in the model to keep XML compact.
    used_events = {t["event"] for t in transitions}
    ordered_names: List[str] = []
    seen = set()
    for name in list(controllable) + list(uncontrollable):
        if name in used_events and name not in seen:
            ordered_names.append(name)
            seen.add(name)
    # Include any stray event referenced in transitions but missing from vectors.
    for name in sorted(used_events):
        if name not in seen:
            ordered_names.append(name)
            seen.add(name)

    events: List[Dict[str, str]] = []
    for idx, name in enumerate(ordered_names):
        if name in ctrl_set:
            controllable_flag = "True"
        elif name in unctrl_set:
            controllable_flag = "False"
        else:
            controllable_flag = "True"  # best effort default
        events.append(
            {
                "id": str(idx),
                "name": name,
                "controllable": controllable_flag,
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
        event_id = event_ids.get(transition["event"])
        if event_id is None:
            raise ValueError(f"Missing event id for {transition['event']}")
        ET.SubElement(
            data,
            "transition",
            source=str(src),
            target=str(tgt),
            event=str(event_id),
        )

    tree = ET.ElementTree(model)
    ET.indent(tree, space="  ", level=0)
    buf = io.BytesIO()
    tree.write(buf, encoding="utf-8", xml_declaration=True)
    return buf.getvalue() + b"\n"


def convert_cpp_to_xml(cpp_path: Path, output_path: Path, supervisor_id: str) -> None:
    source = cpp_path.read_text(encoding="utf-8")
    controllable = _extract_vector(source, "kControllableEvents")
    uncontrollable = _extract_vector(source, "kUncontrollableEvents")
    ordered_states = _extract_states(source)
    init_states = _extract_initial_states(source)
    marked_states = _extract_marked_states(source)
    transitions = _extract_transitions(source)

    states = _states_with_flags(ordered_states, init_states, marked_states)
    events = _event_entries(controllable, uncontrollable, transitions)
    xml_payload = _build_xml(supervisor_id, states, events, transitions)
    output_path.write_bytes(xml_payload)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Convert robot_navigation.cpp supervisors to Nadzoru XML."
    )
    parser.add_argument(
        "--cpp",
        type=Path,
        default=DEFAULT_CPP,
        help="Path to the robot_navigation.cpp-style file to parse.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Destination XML file.",
    )
    parser.add_argument(
        "--supervisor-id",
        default="Sup_reactive_motion",
        help="Identifier stored in the XML's <model id=...> attribute.",
    )
    args = parser.parse_args(argv)

    convert_cpp_to_xml(args.cpp, args.output, args.supervisor_id)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
