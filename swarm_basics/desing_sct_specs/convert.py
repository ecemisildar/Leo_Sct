#!/usr/bin/env python3
import xml.etree.ElementTree as ET
from pathlib import Path

# Path to your XML

FBD_DIR = Path("fbd")
xml_path = FBD_DIR / "uncontrollable_events.xml"  
output_path = xml_path.with_suffix(".txt")


def as_bool(value):
    return "True" if str(value).lower() in ("true", "1", "yes") else "False"

def main():
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # --- Parse states ---
    state_id_to_name = {}
    states = []
    for st in root.findall(".//state"):
        sid = st.get("id")
        name = st.get("name")
        state_id_to_name[sid] = name
        states.append({
            "name": name,
            "initial": as_bool(st.get("initial")),
            "marked": as_bool(st.get("marked")),
        })

    # --- Parse events ---
    event_id_to_name = {}
    events = []
    for ev in root.findall(".//event"):
        eid = ev.get("id")
        name = ev.get("name")
        event_id_to_name[eid] = name
        events.append({
            "name": name,
            "controllable": as_bool(ev.get("controllable")),
            "observable": as_bool(ev.get("observable")),
        })

    # --- Parse transitions ---
    transitions = []
    for tr in root.findall(".//transition"):
        src = state_id_to_name.get(tr.get("source"), tr.get("source"))
        tgt = state_id_to_name.get(tr.get("target"), tr.get("target"))
        ev = event_id_to_name.get(tr.get("event"), tr.get("event"))
        transitions.append({"source": src, "target": tgt, "event": ev})

    # --- Write output ---
    with open(output_path, "w") as f:
        f.write("States:\n")
        for s in states:
            f.write(f"    name={s['name']}, initial={s['initial']}, marked={s['marked']}\n")

        f.write("\nEvents:\n")
        for e in events:
            f.write(f"    name={e['name']}, controllable={e['controllable']}, observable={e['observable']}\n")

        f.write("\nTransitions:\n")
        for t in transitions:
            f.write(f"    source={t['source']}, target={t['target']}, event={t['event']}\n")

    print(f"Converted successfully — saved to: {output_path}")

if __name__ == "__main__":
    main()
