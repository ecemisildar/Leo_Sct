#!/usr/bin/env python3
"""
End-to-end pipeline:
  1) (Optional) call request_supervisor_updates_manual.py to produce JSON.
  2) Convert JSON -> Nadzoru XML.
  3) Run Nadzoru script operations headlessly to produce Sloc*.xml.
  4) Convert one or more Sloc XMLs into SCT YAML for the ROS node.

Quick run:
  python3 run_pipeline.py --task explore --run-llm
  python3 run_pipeline.py --task find_marker --skip-llm
  python3 run_pipeline.py --task wall_follow --run-llm
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import yaml

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
DEFAULT_NADZORU_ROOT = Path.home() / "Nadzoru2"
DEFAULT_SOURCE_AUTOMATA_DIR = THIS_DIR / "hardcoded_find_obj"
DEFAULT_SOURCE_AUTOMATA_DIR_EXPLORE = THIS_DIR / "hardcoded_coverage"
DEFAULT_SOURCE_AUTOMATA_DIR_WALL_FOLLOW = THIS_DIR / "hardcoded_wall_follow"
DEFAULT_SOURCE_AUTOMATA_DIR_ZIGZAG = THIS_DIR / "hardcoded_zigzag"

TASK_AUTOMATA_DIRS = {
    "find_marker": DEFAULT_SOURCE_AUTOMATA_DIR,
    "explore": DEFAULT_SOURCE_AUTOMATA_DIR_EXPLORE,
    "wall_follow": DEFAULT_SOURCE_AUTOMATA_DIR_WALL_FOLLOW,
    "zigzag": DEFAULT_SOURCE_AUTOMATA_DIR_ZIGZAG,
}
DEFAULT_PROFILE_PATH = THIS_DIR / "task_profiles.json"
DEFAULT_OUTPUT_DIR = THIS_DIR / "full_pipeline"
DEFAULT_LLM_DIR = THIS_DIR
DEFAULT_LLM_PREFIX = "latest_"
DEFAULT_LLM_SUFFIX = "_nadzoru.json"
DEFAULT_REAL_YAML_OUT_DIR = Path.home() / "ros_ws/src/Leo_Sct/Leo_Sct/leo_real/config"
DEFAULT_YAML_PREFIX = "sup_gpt_"
DEFAULT_SUPERVISOR_ID = "Sup_reactive_motion"

# Hardcoded LLM options (used only if RUN_LLM is True).
RUN_LLM = True
LLM_ARGS: List[str] = []

# Optional seeds for JSON -> XML conversion (leave empty to infer).
DEFAULT_STATES: List[str] = []
DEFAULT_INITIAL: List[str] = []
DEFAULT_MARKED: List[str] = []

# Skip JSON->XML and use existing E1.xml in full_pipeline.
START_FROM_XML_ONLY = False
START_FROM_XML_INDEX = "1"


def _ensure_nadzoru_imports(nadzoru_root: Path) -> None:
    if not nadzoru_root.exists():
        raise SystemExit(f"Nadzoru2 root not found: {nadzoru_root}")
    print(f"[nadzoru] Using root: {nadzoru_root}")    
    sys.path.insert(0, str(nadzoru_root))
    try:
        from machine.automaton import Automaton  # noqa: F401
    except Exception as e:
        raise SystemExit(f"Failed to import Nadzoru Automaton from {nadzoru_root}: {e}")

def _infer_index(json_path: Path, task: str) -> str:
    name = json_path.stem
    prefix = f"{task}_"
    if name.startswith(prefix) and name.endswith("_nadzoru"):
        return name[len(prefix):].split("_nadzoru", 1)[0]
    if name.startswith(prefix):
        return name[len(prefix):]
    return "1"


def _next_llm_json_path(output_dir: Path, task: str) -> Path:
    existing = list(output_dir.glob(f"{task}_*{DEFAULT_LLM_SUFFIX}"))
    max_idx = 0
    for path in existing:
        idx = _infer_index(path, task)
        if idx.isdigit():
            max_idx = max(max_idx, int(idx))
    next_idx = max_idx + 1
    return output_dir / f"{task}_{next_idx}{DEFAULT_LLM_SUFFIX}"


def _latest_llm_json_path(output_dir: Path, task: str) -> Path:
    existing = list(output_dir.glob(f"{task}_*{DEFAULT_LLM_SUFFIX}"))
    max_idx = 0
    for path in existing:
        idx = _infer_index(path, task)
        if idx.isdigit():
            max_idx = max(max_idx, int(idx))
    return output_dir / f"{task}_{max_idx}{DEFAULT_LLM_SUFFIX}"


def _prepare_output_dir(
    output_dir: Path,
    source_dir: Path,
    index: str,
    keep_existing_e: bool,
) -> Dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale in ("G1.xml", "G2.xml"):
        stale_path = output_dir / stale
        if stale_path.exists():
            stale_path.unlink()
    g1_src = source_dir / "G1.xml"
    g2_src = source_dir / "G2.xml"
    if not g1_src.exists() or not g2_src.exists():
        raise SystemExit("G1.xml or G2.xml not found in hardcoded automata folder.")

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
        "g1": g1_src,
        "g2": g2_src,
    }


def _load_automatons(xml_dirs: Sequence[Path]) -> Dict[str, "Automaton"]:
    from machine.automaton import Automaton

    automatons: Dict[str, Automaton] = {}
    for xml_dir in xml_dirs:
        for xml_path in sorted(xml_dir.glob("*.xml")):
            automaton = Automaton()
            automaton.load(str(xml_path))
            automatons[automaton.get_id_name()] = automaton
    return automatons


def _run_nadzoru_script(
    script_path: Path,
    automata: Dict[str, "Automaton"],
    output_dir: Path,
    save_names: List[str],
) -> List[Path]:
    from machine.automaton import Automaton

    loc = {
        "Sync": Automaton.synchronization,
        "SupC": Automaton.sup_c,
        "Observer": Automaton.observer,
        "Accessible": Automaton.accessible,
        "Coaccessible": Automaton.coaccessible,
        "Trim": Automaton.trim,
        "Minimize": Automaton.minimize,
        "SupervisorReduction": Automaton.supervisor_reduction,
        "Labeller": Automaton.labeller,
        "Diagnoser": Automaton.diagnoser,
    }
    loc.update(automata)

    script = script_path.read_text(encoding="utf-8")
    exec(script, {}, loc)

    generated: List[Path] = []
    for name in save_names:
        obj = loc.get(name)
        if not isinstance(obj, Automaton):
            raise SystemExit(f"[nadzoru] '{name}' not created by script (or not an Automaton).")

        out = output_dir / f"{name}.xml"
        obj.set_name(name)
        obj.set_file_path_name(str(out))
        obj.arrange_states_position()
        obj.save()
        generated.append(out)

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


def _infer_llm_profile(llm_args: Sequence[str]) -> str | None:
    if "--profile" not in llm_args:
        return None
    try:
        idx = llm_args.index("--profile")
        if idx + 1 < len(llm_args):
            return llm_args[idx + 1]
    except ValueError:
        return None
    return None


def _load_profiles(path: Path) -> Dict[str, Dict[str, object]]:
    if not path.exists():
        raise SystemExit(f"Profile file not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _select_event_sets(profile: str | None, profiles: Dict[str, Dict[str, object]]) -> tuple[List[str], List[str]]:
    if not profile:
        raise SystemExit("Profile name is required to select event sets.")
    if profile not in profiles:
        raise SystemExit(f"Unknown profile: {profile}")
    data = profiles[profile]
    controllable = list(data.get("controllable_events", []))
    uncontrollable = list(data.get("uncontrollable_events", []))
    if not controllable or not uncontrollable:
        raise SystemExit(f"Profile '{profile}' is missing event lists.")
    return controllable, uncontrollable


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the full DES->Nadzoru->SCT pipeline."
    )
    parser.add_argument(
        "--task",
        required=True,
        help="Task profile to run (e.g., explore, find_marker, wall_follow).",
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

    profiles = _load_profiles(DEFAULT_PROFILE_PATH)
    if args.task not in profiles:
        raise SystemExit(f"Unknown task: {args.task}")

    run_llm = args.run_llm and not args.skip_llm
    llm_profile_arg = args.llm_profile or args.task
    llm_args = _build_llm_args(
        LLM_ARGS,
        profile=llm_profile_arg,
        goal=args.llm_goal,
        constraints=args.llm_constraint,
        no_default_constraints=args.llm_no_default_constraints,
    )
    llm_profile = args.task
    if llm_profile not in TASK_AUTOMATA_DIRS:
        raise SystemExit(f"No automata folder configured for task: {llm_profile}")
    source_automata_dir = TASK_AUTOMATA_DIRS[llm_profile]
    controllable_events, uncontrollable_events = _select_event_sets(llm_profile, profiles)

    if run_llm:
        DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        llm_json_path = _next_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)
        _run_llm(llm_args, llm_json_path)
    else:
        llm_json_path = _latest_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)

    if not run_llm and not START_FROM_XML_ONLY and not llm_json_path.exists():
        raise SystemExit(f"Expected input not found: {llm_json_path}")

    index = START_FROM_XML_INDEX if START_FROM_XML_ONLY else _infer_index(llm_json_path, args.task)
    paths = _prepare_output_dir(
        DEFAULT_OUTPUT_DIR,
        source_automata_dir,
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
            controllable_events,
            uncontrollable_events,
            DEFAULT_SUPERVISOR_ID,
        )
    elif not paths["e_xml"].exists():
        raise SystemExit(f"Expected input not found: {paths['e_xml']}")

    _ensure_nadzoru_imports(DEFAULT_NADZORU_ROOT)
    automata = _load_automatons([DEFAULT_OUTPUT_DIR, source_automata_dir])
    target = f"Sloc{index}"
    _run_nadzoru_script(paths["script"], automata, DEFAULT_OUTPUT_DIR, save_names=["Gloc", "Kloc", target])

    #_run_nadzoru_script(paths["script"], automata, DEFAULT_OUTPUT_DIR)

    if not paths["sloc_xml"].exists():
        raise SystemExit(f"Expected output not found: {paths['sloc_xml']}")

    from machine.automaton import Automaton

    sloc_automatons = []
    automaton = Automaton()
    automaton.load(str(paths["sloc_xml"]))
    sloc_automatons.append(automaton)

    yaml_payload = _build_sct_yaml(sloc_automatons)
    yaml_real_out = DEFAULT_REAL_YAML_OUT_DIR / f"{args.task}_{DEFAULT_YAML_PREFIX}{index}.yaml"
    yaml_real_latest = DEFAULT_REAL_YAML_OUT_DIR / f"{args.task}_{DEFAULT_YAML_PREFIX}latest.yaml"
    yaml_real_default = DEFAULT_REAL_YAML_OUT_DIR / "sup_gpt.yaml"
    yaml_real_out.parent.mkdir(parents=True, exist_ok=True)
    for path in (
        yaml_real_out,
        yaml_real_latest,
        yaml_real_default,
    ):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(yaml_payload, f, sort_keys=False, default_flow_style=True)

    print(f"Wrote YAML to {yaml_real_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
