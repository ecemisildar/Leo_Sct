#!/usr/bin/env python3
import sys
import yaml

# Exact event order you used before
EVENTS_ORDER = [
    "EV_move_backward",
    "EV_obstacle_left",
    "EV_clockwise_turn",
    "EV_full_rotate",
    "EV_obstacle_right",
    "EV_random_walk",
    "EV_obstacle_front",
    "EV_counterclockwise_turn",
    "EV_path_clear",
    "EV_move_forward",
    "EV_speed_up",
    "EV_slow_down",
    "EV_sense_crowd",
]

# Map FAUDES event names (from .gen) -> EV_* names
FAUDES_TO_EV = {
    "move_backward":           "EV_move_backward",
    "obstacle_left":           "EV_obstacle_left",
    "rotate_clockwise":        "EV_clockwise_turn",
    "full_rotate":             "EV_full_rotate",
    "obstacle_right":          "EV_obstacle_right",
    "random_walk":             "EV_random_walk",
    "obstacle_front":          "EV_obstacle_front",
    "rotate_counterclockwise": "EV_counterclockwise_turn",
    "path_clear":              "EV_path_clear",
    "move_forward":            "EV_move_forward",
    "speed_up":                "EV_speed_up",
    "slow_down":               "EV_slow_down",
    "crowd_detected":          "EV_sense_crowd", 
}

# Controllability pattern in this fixed order
EV_CONTROLLABLE = [
    1,  # EV_move_backward
    0,  # EV_obstacle_left
    1,  # EV_clockwise_turn
    1,  # EV_full_rotate
    0,  # EV_obstacle_right
    1,  # EV_random_walk
    0,  # EV_obstacle_front
    1,  # EV_counterclockwise_turn
    0,  # EV_path_clear
    1,  # EV_move_forward
    1,  # EV_speed_up
    1,  # EV_slow_down
    0,  # EV_sense_crowd
]


def parse_gen(path):
    """Parse a FAUDES .gen file into a simple dict."""
    with open(path, "r") as f:
        lines = [l.strip() for l in f if l.strip() and not l.strip().startswith("%")]

    section = None
    name = None
    alphabet = []
    states = []
    transrel = []
    init_states = []
    marked_states = []

    for line in lines:
        # Generator header
        if line.startswith("<Generator"):
            if 'name="' in line:
                name = line.split('name="', 1)[1].split('"', 1)[0]
            continue

        # Section tags
        if line.startswith("<") and line.endswith(">"):
            tag = line[1:-1]
            if tag in ["Alphabet", "States", "TransRel", "InitStates", "MarkedStates"]:
                section = tag
            elif tag.startswith("/"):
                section = None
            else:
                section = None
            continue

        # Section contents
        if section == "Alphabet":
            alphabet.extend(line.split())
        elif section == "States":
            states.extend(line.split())
        elif section == "TransRel":
            parts = line.split()
            if len(parts) == 3:
                src, ev, tgt = parts
                transrel.append((src, ev, tgt))
        elif section == "InitStates":
            init_states.extend(line.split())
        elif section == "MarkedStates":
            marked_states.extend(line.split())

    return {
        "name": name or "Supervisor",
        "alphabet": alphabet,
        "states": states,
        "transrel": transrel,
        "init_states": init_states,
        "marked_states": marked_states,
    }


def build_yaml(gen_info):
    name = gen_info["name"]
    states = gen_info["states"]
    transrel = gen_info["transrel"]
    init_states = gen_info["init_states"]

    # Check that all FAUDES events are mappable
    for _, fev, _ in transrel:
        if fev not in FAUDES_TO_EV:
            raise ValueError(f"No EV_* mapping for FAUDES event '{fev}'")

    # ---- States and indices ----
    state_index = {s: i for i, s in enumerate(states)}

    # ---- Events section ----
    num_events = len(EVENTS_ORDER)
    events = EVENTS_ORDER[:]  # copy

    ev_controllable = EV_CONTROLLABLE[:]
    ev_public = [1] * num_events  # all public by default

    # Map EV_* names to indices 0..9
    ev_to_idx = {ev: i for i, ev in enumerate(events)}

    # ---- Supervisors ----
    num_supervisors = 1
    # this single supervisor uses all events
    sup_events = [[1] * num_events]

    # initial/current state
    if not init_states:
        raise ValueError("No initial state in .gen")
    init_state_name = init_states[0]
    init_idx = state_index[init_state_name]
    sup_init_state = [init_idx]
    sup_current_state = [init_idx]

    # ---- sup_data & sup_data_pos ----
    sup_data_pos = [0]
    sup_data = []

    # group transitions by source state
    trans_by_src = {s: [] for s in states}
    for src, fev, tgt in transrel:
        ev_name = FAUDES_TO_EV[fev]          # EV_* name
        trans_by_src[src].append((ev_name, tgt))

    # encode each state in order
    for s_name in states:
        triples = []
        for ev_name, tgt_name in trans_by_src[s_name]:
            e_idx = ev_to_idx[ev_name]        # index in EVENTS_ORDER
            tgt_idx = state_index[tgt_name]   # state index
            hi = tgt_idx // 256
            lo = tgt_idx % 256
            # We can store the EV_* string; SCT.get_value() will map it to e_idx
            triples.append((ev_name, hi, lo))

        # number of transitions
        sup_data.append(len(triples))
        # transitions: (event, hi, lo)*
        for ev_name, hi, lo in triples:
            sup_data.append(ev_name)
            sup_data.append(hi)
            sup_data.append(lo)

    yaml_data = {
        "num_events": num_events,
        "num_supervisors": num_supervisors,
        "events": events,
        "ev_controllable": ev_controllable,
        "sup_events": sup_events,
        "sup_init_state": sup_init_state,
        "sup_current_state": sup_current_state,
        "sup_data_pos": sup_data_pos,
        "sup_data": sup_data,
        "ev_public": ev_public,
    }

    return yaml_data


def main():
    if len(sys.argv) != 3:
        print("Usage: gen_to_sct_yaml.py Sup_reactive_motion.gen Sup_reactive_motion.yaml")
        sys.exit(1)

    gen_path = sys.argv[1]
    yaml_path = sys.argv[2]

    gen_info = parse_gen(gen_path)
    yaml_data = build_yaml(gen_info)

    with open(yaml_path, "w") as f:
        yaml.safe_dump(
            yaml_data,
            f,
            sort_keys=False,
            default_flow_style=True  # <– this makes lists use [ ... ]
        )


    print(f"Converted {gen_path} -> {yaml_path}")


if __name__ == "__main__":
    main()
