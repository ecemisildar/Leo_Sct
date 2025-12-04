#!/usr/bin/env python3
import re
import xml.etree.ElementTree as ET
from pathlib import Path

# --- Input / Output paths ---
INPUT_TXT = Path("supervisors_xml/test.txt")
OUTPUT_XML = INPUT_TXT.with_suffix(".xml")

# --- Helper to parse lines like "name=0, initial=True, marked=True"
def parse_kv_line(line: str):
    pairs = [kv.strip() for kv in line.split(",") if "=" in kv]
    data = {}
    for kv in pairs:
        k, v = kv.split("=", 1)
        data[k.strip()] = v.strip()
    return data

def txt_to_xml(txt_path: Path, xml_path: Path):
    with open(txt_path, "r") as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    # --- Section splitters ---
    states, events, transitions = [], [], []
    section = None
    for ln in lines:
        if ln.startswith("States:"):
            section = "states"
            continue
        elif ln.startswith("Events:"):
            section = "events"
            continue
        elif ln.startswith("Transitions:"):
            section = "transitions"
            continue

        if section == "states" and ln.startswith("name="):
            states.append(parse_kv_line(ln))
        elif section == "events" and ln.startswith("name="):
            events.append(parse_kv_line(ln))
        elif section == "transitions" and ln.startswith("source="):
            transitions.append(parse_kv_line(ln))

    # --- Build XML ---
    model = ET.Element("model", version="0.0", type="FSA", id="Untitled")
    data = ET.SubElement(model, "data")

    # Add states
    for i, s in enumerate(states):
        # Give numeric id if name looks like string
        sid = s.get("name", str(i))
        ET.SubElement(
            data,
            "state",
            id=sid,
            name=sid,
            initial=s.get("initial", "False"),
            marked=s.get("marked", "True"),
            x=str(150 + 200 * i),  # simple layout coords
            y="260",
        )

    # Add events
    for i, e in enumerate(events):
        ET.SubElement(
            data,
            "event",
            id=str(i),
            name=e.get("name", f"E{i}"),
            controllable=e.get("controllable", "False"),
            observable=e.get("observable", "True"),
        )

    # Add transitions
    for t in transitions:
        ET.SubElement(
            data,
            "transition",
            source=t.get("source", "0"),
            target=t.get("target", "0"),
            event=t.get("event", ""),
        )

    # --- Write pretty XML ---
    def indent(elem, level=0):
        i = "\n" + level * "    "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "    "
            for e in elem:
                indent(e, level + 1)
            if not e.tail or not e.tail.strip():
                e.tail = i
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

    indent(model)
    tree = ET.ElementTree(model)
    xml_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    print(f"Converted {txt_path} → {xml_path}")

if __name__ == "__main__":
    txt_to_xml(INPUT_TXT, OUTPUT_XML)
