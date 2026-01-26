#!/usr/bin/env python3
"""
End-to-end pipeline:
  1) (Optional) call request_supervisor_updates_manual.py to produce JSON.
  2) Convert JSON -> Nadzoru XML.
  3) Run Nadzoru script operations headlessly to produce Sloc*.xml.
  4) Convert one or more Sloc XMLs into SCT YAML for the ROS node.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import yaml

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
DEFAULT_NADZORU_ROOT = Path.home() / "Documents" / "Nadzoru2"
DEFAULT_SOURCE_AUTOMATA_DIR = THIS_DIR / "nadzoru_files" / "hardcoded_automata"
DEFAULT_OUTPUT_DIR = THIS_DIR / "full_pipeline"
DEFAULT_LLM_DIR = THIS_DIR
DEFAULT_LLM_PREFIX = "latest_"
DEFAULT_LLM_SUFFIX = "_nadzoru.json"
DEFAULT_YAML_OUT_DIR = Path("/home/ecem/ros2_ws/src/Leo_sct/swarm_basics/config")
DEFAULT_YAML_PREFIX = "sup_gpt_"
DEFAULT_SUPERVISOR_ID = "Sup_reactive_motion"

# Hardcoded LLM options (used only if RUN_LLM is True).
RUN_LLM = True
LLM_ARGS = [
    "--profile",
    "find_blue",
]

# Optional seeds for JSON -> XML conversion (leave empty to infer).
DEFAULT_STATES: List[str] = []
DEFAULT_INITIAL: List[str] = []
DEFAULT_MARKED: List[str] = []
DEFAULT_CONTROLLABLE: List[str] = [
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    "move_to_blue",
    "stop",
]
DEFAULT_UNCONTROLLABLE: List[str] = [
    "path_clear",
    "obstacle_front",
    "obstacle_left",
    "obstacle_right",
    "blue_seen",
    "blue_close",
]

# Skip JSON->XML and use existing E1.xml in full_pipeline.
START_FROM_XML_ONLY = False
START_FROM_XML_INDEX = "1"


def _ensure_nadzoru_imports(nadzoru_root: Path) -> None:
    if not nadzoru_root.exists():
        raise SystemExit(f"Nadzoru2 root not found: {nadzoru_root}")
    sys.path.insert(0, str(nadzoru_root))


def _infer_index(json_path: Path) -> str:
    name = json_path.stem
    if "latest_" in name and name.endswith("_nadzoru"):
        return name.split("latest_", 1)[1].split("_nadzoru", 1)[0]
    if name.startswith("latest_"):
        return name.split("latest_", 1)[1]
    return "1"


def _next_llm_json_path(output_dir: Path) -> Path:
    existing = list(output_dir.glob(f"{DEFAULT_LLM_PREFIX}*{DEFAULT_LLM_SUFFIX}"))
    max_idx = 0
    for path in existing:
        idx = _infer_index(path)
        if idx.isdigit():
            max_idx = max(max_idx, int(idx))
    next_idx = max_idx + 1
    return output_dir / f"{DEFAULT_LLM_PREFIX}{next_idx}{DEFAULT_LLM_SUFFIX}"


def _latest_llm_json_path(output_dir: Path) -> Path:
    existing = list(output_dir.glob(f"{DEFAULT_LLM_PREFIX}*{DEFAULT_LLM_SUFFIX}"))
    max_idx = 0
    for path in existing:
        idx = _infer_index(path)
        if idx.isdigit():
            max_idx = max(max_idx, int(idx))
    return output_dir / f"{DEFAULT_LLM_PREFIX}{max_idx}{DEFAULT_LLM_SUFFIX}"


def _prepare_output_dir(
    output_dir: Path,
    source_dir: Path,
    index: str,
    keep_existing_e: bool,
) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    g1_src = source_dir / "G1.xml"
    g2_src = source_dir / "G2.xml"
    if not g1_src.exists() or not g2_src.exists():
        raise SystemExit("G1.xml or G2.xml not found in hardcoded automata folder.")
    g1_dst = output_dir / "G1.xml"
    g2_dst = output_dir / "G2.xml"
    if not g1_dst.exists():
        g1_dst.write_bytes(g1_src.read_bytes())
    if not g2_dst.exists():
        g2_dst.write_bytes(g2_src.read_bytes())

    e_xml = output_dir / f"E{index}.xml"
    sloc_xml = output_dir / f"Sloc{index}.xml"
    script_path = output_dir / "script.txt"
    script_path.write_text(
        "\n".join(
            [
                "# Auto-generated script",
                "",
                "Gloc = Sync(G1,G2)",
                f"Kloc = Sync(Gloc, E{index})",
                f"Sloc{index} = SupC(Gloc, Kloc)",
                "",
            ]
        ),
        encoding="utf-8",
    )
    if keep_existing_e and e_xml.exists():
        e_xml = e_xml
    return {
        "e_xml": e_xml,
        "sloc_xml": sloc_xml,
        "script": script_path,
        "g1": g1_dst,
        "g2": g2_dst,
    }


def _load_automatons(xml_dir: Path) -> Dict[str, "Automaton"]:
    from machine.automaton import Automaton

    automatons: Dict[str, Automaton] = {}
    for xml_path in sorted(xml_dir.glob("*.xml")):
        automaton = Automaton()
        automaton.load(str(xml_path))
        automatons[automaton.get_id_name()] = automaton
    return automatons


def _run_nadzoru_script(script_path: Path, automata: Dict[str, "Automaton"], output_dir: Path) -> List[Path]:
    from machine.automaton import Automaton

    loc = {
        "Sync": Automaton.synchronization,
        "SupC": Automaton.sup_c,
        "Observer": Automaton.observer,
        "Accessible": Automaton.accessible,
        "Coaccessible": Automaton.coaccessible,
        "Trim": Automaton.trim,
        "Minimize": Automaton.minimize,
        "Supervisor Reduction": Automaton.supervisor_reduction,
        "Labeller": Automaton.labeller,
        "Diagnoser": Automaton.diagnoser,
    }
    loc.update(automata)

    script = script_path.read_text(encoding="utf-8")
    exec(script, {}, loc)

    generated: List[Path] = []
    for name, automaton in loc.items():
        if not isinstance(automaton, Automaton):
            continue
        if name in automata:
            continue
        output_path = output_dir / f"{name}.xml"
        automaton.set_name(name)
        automaton.set_file_path_name(str(output_path))
        automaton.arrange_states_position()
        automaton.save()
        generated.append(output_path)
    return generated


def _build_sct_yaml(automatons: Sequence["Automaton"]) -> Dict[str, object]:
    if not automatons:
        raise ValueError("No automatons provided for YAML generation.")

    event_by_name = {}
    for automaton in automatons:
        for event in automaton.events:
            if event.name not in event_by_name:
                event_by_name[event.name] = event
            else:
                existing = event_by_name[event.name]
                if existing.controllable != event.controllable:
                    raise ValueError(f"Conflicting controllability for event: {event.name}")

    event_names = sorted(event_by_name.keys())
    events = [f"EV_{name}" for name in event_names]
    ev_controllable = [1 if event_by_name[name].controllable else 0 for name in event_names]

    sup_events = []
    for automaton in automatons:
        automaton_events = {event.name for event in automaton.events}
        sup_events.append([1 if name in automaton_events else 0 for name in event_names])

    sup_data_pos: List[int] = []
    sup_data: List[object] = []
    sup_init_state: List[int] = []

    for automaton in automatons:
        states = sorted(automaton.states, key=lambda s: s.name)
        state_index = {state: idx for idx, state in enumerate(states)}

        if automaton.initial_state is None:
            raise ValueError(f"Automaton '{automaton.get_id_name()}' has no initial state.")

        sup_data_pos.append(len(sup_data))
        sup_init_state.append(state_index[automaton.initial_state])

        for state in states:
            transitions = sorted(
                state.out_transitions,
                key=lambda t: (t.event.name, t.to_state.name),
            )
            sup_data.append(len(transitions))
            for transition in transitions:
                sup_data.append(f"EV_{transition.event.name}")
                tgt_idx = state_index[transition.to_state]
                sup_data.append(tgt_idx // 256)
                sup_data.append(tgt_idx % 256)

    return {
        "num_events": len(events),
        "num_supervisors": len(automatons),
        "events": events,
        "ev_controllable": ev_controllable,
        "sup_events": sup_events,
        "sup_init_state": sup_init_state,
        "sup_current_state": list(sup_init_state),
        "sup_data_pos": sup_data_pos,
        "sup_data": sup_data,
    }


def _convert_json_to_xml(
    json_path: Path,
    output_path: Path,
    states: Sequence[str],
    initial: Sequence[str],
    marked: Sequence[str],
    controllable: Sequence[str],
    uncontrollable: Sequence[str],
    supervisor_id: str,
) -> None:
    import json_to_nadzoru_xml

    args = argparse.Namespace(
        json=str(json_path),
        output=str(output_path),
        states=list(states),
        initial=list(initial) if initial else None,
        marked=list(marked) if marked else None,
        controllable=list(controllable) if controllable else None,
        uncontrollable=list(uncontrollable) if uncontrollable else None,
        supervisor_id=supervisor_id,
    )
    json_to_nadzoru_xml.convert_json_to_xml(args)


def _run_llm(llm_args: List[str], llm_json: Path) -> None:
    import request_supervisor_updates_manual as llm

    args = list(llm_args)
    if "--save" not in args:
        args.extend(["--save", str(llm_json)])
    result = llm.main(args)
    if result != 0:
        raise SystemExit(result)


def _build_llm_args(
    base_args: Sequence[str],
    profile: str | None,
    goal: str | None,
    constraints: Sequence[str] | None,
    no_default_constraints: bool,
) -> List[str]:
    args = list(base_args)
    if profile:
        args.extend(["--profile", profile])
    if goal:
        args.extend(["--goal", goal])
    if constraints:
        for item in constraints:
            args.extend(["--constraint", item])
    if no_default_constraints:
        args.append("--no-default-constraints")
    return args


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the full DES->Nadzoru->SCT pipeline."
    )
    parser.add_argument(
        "--run-llm",
        action="store_true",
        default=RUN_LLM,
        help="Generate a new supervisor JSON via the LLM step.",
    )
    parser.add_argument(
        "--skip-llm",
        action="store_true",
        help="Skip the LLM step and use the latest JSON output.",
    )
    parser.add_argument(
        "--llm-profile",
        help="Select the LLM goal profile to use (passed to request_supervisor_updates_manual.py).",
    )
    parser.add_argument(
        "--llm-goal",
        help="Override the LLM goal text.",
    )
    parser.add_argument(
        "--llm-constraint",
        action="append",
        default=[],
        help="Add one or more LLM constraint lines (repeatable).",
    )
    parser.add_argument(
        "--llm-no-default-constraints",
        action="store_true",
        help="Do not include default LLM constraints.",
    )
    args = parser.parse_args()

    run_llm = args.run_llm and not args.skip_llm
    llm_args = _build_llm_args(
        LLM_ARGS,
        profile=args.llm_profile,
        goal=args.llm_goal,
        constraints=args.llm_constraint,
        no_default_constraints=args.llm_no_default_constraints,
    )

    if run_llm:
        DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        llm_json_path = _next_llm_json_path(DEFAULT_OUTPUT_DIR)
        _run_llm(llm_args, llm_json_path)
    else:
        llm_json_path = _latest_llm_json_path(DEFAULT_OUTPUT_DIR)

    if not run_llm and not START_FROM_XML_ONLY and not llm_json_path.exists():
        raise SystemExit(f"Expected input not found: {llm_json_path}")

    index = START_FROM_XML_INDEX if START_FROM_XML_ONLY else _infer_index(llm_json_path)
    paths = _prepare_output_dir(
        DEFAULT_OUTPUT_DIR,
        DEFAULT_SOURCE_AUTOMATA_DIR,
        index,
        START_FROM_XML_ONLY,
    )
    if not START_FROM_XML_ONLY:
        _convert_json_to_xml(
            llm_json_path,
            paths["e_xml"],
            DEFAULT_STATES,
            DEFAULT_INITIAL,
            DEFAULT_MARKED,
            DEFAULT_CONTROLLABLE,
            DEFAULT_UNCONTROLLABLE,
            DEFAULT_SUPERVISOR_ID,
        )
    elif not paths["e_xml"].exists():
        raise SystemExit(f"Expected input not found: {paths['e_xml']}")

    _ensure_nadzoru_imports(DEFAULT_NADZORU_ROOT)
    automata = _load_automatons(DEFAULT_OUTPUT_DIR)
    _run_nadzoru_script(paths["script"], automata, DEFAULT_OUTPUT_DIR)

    if not paths["sloc_xml"].exists():
        raise SystemExit(f"Expected output not found: {paths['sloc_xml']}")

    from machine.automaton import Automaton

    sloc_automatons = []
    automaton = Automaton()
    automaton.load(str(paths["sloc_xml"]))
    sloc_automatons.append(automaton)

    yaml_payload = _build_sct_yaml(sloc_automatons)
    yaml_out = DEFAULT_YAML_OUT_DIR / f"{DEFAULT_YAML_PREFIX}{index}.yaml"
    yaml_latest = DEFAULT_YAML_OUT_DIR / "sup_gpt.yaml"
    yaml_out.parent.mkdir(parents=True, exist_ok=True)
    for path in (yaml_out, yaml_latest):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(yaml_payload, f, sort_keys=False, default_flow_style=True)

    print(f"Wrote YAML to {yaml_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
