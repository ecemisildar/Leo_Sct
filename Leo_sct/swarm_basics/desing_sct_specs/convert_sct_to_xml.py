#!/usr/bin/env python3
"""
Convert generated supervisor text files into XML (Nadzoru FSA format).
Each text file can contain multiple supervisors (### Supervisor for ...).

Author: Ecem Isildar
"""

import re
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import io

# ==========================================================
#  1. Configuration
# ==========================================================
INPUT_DIR = Path("prompt_eval_results_new")   # where LLM-generated outputs are stored
OUTPUT_DIR = Path("converted_xml_new")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
FBD_DIR = Path("fbd")
CONTROLLABLE_FILE = FBD_DIR / "controllable_events.txt"
UNCONTROLLABLE_FILE = FBD_DIR / "uncontrollable_events.txt"


def load_event_vocab():
    """Read authoritative controllable/uncontrollable names from the FBD text files."""
    ctrl, unctrl = [], []
    if CONTROLLABLE_FILE.exists():
        with open(CONTROLLABLE_FILE) as f:
            for line in f:
                if "name=" in line and "controllable=True" in line:
                    ctrl.append(line.split("name=")[1].split(",")[0].strip())
    if UNCONTROLLABLE_FILE.exists():
        with open(UNCONTROLLABLE_FILE) as f:
            for line in f:
                if "name=" in line:
                    unctrl.append(line.split("name=")[1].split(",")[0].strip())
    return unctrl, ctrl


allowed_unctrl, allowed_ctrl = load_event_vocab()

# ==========================================================
#  2. Helper functions
# ==========================================================
def parse_supervisors_from_text(text):
    """Split one file into multiple supervisors."""
    pattern = r"### Supervisor for ([\w_]+)"
    blocks = re.split(pattern, text)
    supervisors = []

    # blocks = ['', name1, content1, name2, content2, ...]
    for i in range(1, len(blocks), 2):
        name = blocks[i].strip()
        body = blocks[i + 1].strip()
        supervisors.append((name, body))
    return supervisors


def parse_states(block_text):
    """Extract state lines."""
    pattern = r"name=([A-Za-z0-9_]+),\s*initial=(True|False),\s*marked=(True|False)"
    matches = re.findall(pattern, block_text)
    states = []
    for i, (name, initial, marked) in enumerate(matches):
        states.append({
            "id": str(i),       # internal numeric ID
            "name": name,       # human-readable state name
            "initial": initial,
            "marked": marked,
            "x": str(150 + 200 * (i % 3)),
            "y": str(200 + 60 * (i // 3)),
        })
    return states


def parse_transitions(block_text):
    """Extract transitions (source, target, event)."""
    pattern = r"source=([A-Za-z0-9_]+),\s*target=([A-Za-z0-9_]+),\s*event=([A-Za-z0-9_]+)"
    matches = re.findall(pattern, block_text)
    transitions = []
    for src, tgt, evt in matches:
        transitions.append({
            "source": src,
            "target": tgt,
            "event": evt
        })
    return transitions


def collect_events(transitions):
    """Build a list of unique events appearing in transitions."""
    events = sorted(set([t["event"] for t in transitions]))
    return events


def build_event_elements(events, ctrl_events, unctrl_events):
    """Build <event> entries respecting controllable/uncontrollable flags."""
    event_elements = []
    ctrl_set = set(ctrl_events)
    unctrl_set = set(unctrl_events)
    for idx, name in enumerate(events):
        if name in ctrl_set:
            ctrl_flag = "True"
        elif name in unctrl_set:
            ctrl_flag = "False"
        else:
            # default to controllable if the name is unknown
            ctrl_flag = "True"

        event_elements.append({
            "id": str(idx),
            "name": name,
            "controllable": ctrl_flag,
            "observable": "True"
        })
    return event_elements



# ==========================================================
#  3. XML construction
# ==========================================================
def build_xml(supervisor_name, states, events, transitions):
    model = ET.Element("model", version="0.0", type="FSA", id=str(supervisor_name))
    data = ET.SubElement(model, "data")

    # State name → numeric ID
    name_to_id = {s["name"]: s["id"] for s in states}

    # Event name → numeric ID
    event_name_to_id = {e["name"]: e["id"] for e in events}

    # ---- States ----
    for s in states:
        ET.SubElement(
            data, "state",
            id=str(s["id"]),
            name=str(s["name"]),
            initial=str(s["initial"]),
            marked=str(s["marked"]),
            x=str(s["x"]), 
            y=str(s["y"]),
        )

    # ---- Events ----
    for e in events:
        ET.SubElement(
            data, "event",
            id=str(e["id"]),
            name=str(e["name"]),
            controllable=str(e["controllable"]),
            observable=str(e["observable"]),
        )

    # ---- Transitions (use event IDs) ----
    for t in transitions:
        src = name_to_id.get(t["source"], t["source"])
        tgt = name_to_id.get(t["target"], t["target"])

        evt_name = t["event"]
        evt_id = event_name_to_id.get(evt_name)

        ET.SubElement(
            data, "transition",
            source=str(src),
            target=str(tgt),
            event=str(evt_id),
        )

    # ---- Pretty output ----
    tree = ET.ElementTree(model)
    ET.indent(tree, space="  ", level=0)
    buf = io.BytesIO()
    tree.write(buf, encoding="utf-8", xml_declaration=True)
    return buf.getvalue() + b"\n"




# ==========================================================
#  4. Main conversion loop
# ==========================================================
for txt_file in sorted(INPUT_DIR.glob("*.txt")):
    print(f"Processing {txt_file.name} ...")
    text = txt_file.read_text()

    supervisors = parse_supervisors_from_text(text)
    if not supervisors:
        print(f"No supervisors found in {txt_file.name}")
        continue

    for name, body in supervisors:
        states = parse_states(body)
        transitions = parse_transitions(body)
        events = build_event_elements(collect_events(transitions), allowed_ctrl, allowed_unctrl)
        xml_data = build_xml(name, states, events, transitions)

        out_path = OUTPUT_DIR / f"{txt_file.stem}_{name}.xml"
        with open(out_path, "wb") as f:
            f.write(xml_data)
        print(f"Wrote {out_path.name}")

print("\nAll supervisors converted successfully!")
