#!/usr/bin/env python3
"""
End-to-end pipeline:
  1) (Optional) run the built-in LLM step to produce JSON.
  2) Convert JSON -> Nadzoru XML.
  3) Run Nadzoru script operations headlessly to produce Sloc*.xml.
  4) Convert one or more Sloc XMLs into SCT YAML for the ROS node.

Quick run:
  python3 run_pipeline.py --task explore --run-llm
  python3 run_pipeline.py --task find_marker --skip-llm
  python3 run_pipeline.py --task wall_follow --run-llm
  python3 run_pipeline.py --task zigzag --run-llm
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

from collections import defaultdict
from dataclasses import dataclass

import requests
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
DEFAULT_LLM_SUFFIX = "_nadzoru.json"
DEFAULT_REAL_YAML_OUT_DIR = Path("/home/ecem/ros2_ws/src/Leo_sct/leo_real/config")
DEFAULT_YAML_PREFIX = "sup_gpt_"
DEFAULT_SUPERVISOR_ID = "Sup_reactive_motion"

# Hardcoded LLM options (used only if RUN_LLM is True).
RUN_LLM = True

# LLM defaults (shared with the LLM-only flow).
DEFAULT_KEY_FILE = THIS_DIR.parent / "api_key.txt"
DEFAULT_MODEL = "gpt-4.1"
DEFAULT_GOAL = "Find the marker object and get close to it and stop while staying safe"
DEFAULT_TIMEOUT_S = 120.0
DEFAULT_RETRIES = 2

DEFAULT_STATES = ["clear", "obs_left", "obs_right", "obs_front"]
DEFAULT_CONSTRAINTS = [
    # Alphabet / validity
    "Do NOT introduce any new controllable or uncontrollable events. Use ONLY the provided event lists.",
    "Do NOT invent sensor events beyond the given uncontrollable list (e.g., no 'battery_low', etc.).",
    "All transitions must be formatted exactly as: (\"state\", \"event\", \"next\").",

    # Determinism (critical)
    "Determinism required: for each (state, event) pair, specify EXACTLY ONE next state. No duplicates.",
    "No epsilon transitions. Every transition must be labeled by one event from the lists.",

    # Uncontrollable coverage (your requirement)
    "For EVERY state, include an outgoing transition for ALL uncontrollable events (total w.r.t. uncontrollables).",

    # Controllable presence / progress
    "For EVERY non-terminal state, include at least ONE controllable transition (otherwise the robot may stall).",
    "Avoid enabling multiple controllable actions in the same state (runtime may choose randomly among them).",
    "Exception: obs_front may enable BOTH move_backward and full_rotate if you strongly need it.",

    # Runtime semantics: uncontrollable first
    "Runtime semantics: uncontrollable events are processed BEFORE a controllable action each step.",
    "Therefore, in action/commit/scan/recovery states, non-critical uncontrollables (e.g., path_clear, marker_seen/marker_lost) should usually SELF-LOOP to avoid preempting the intended controllable in the same step.",
    "Only safety-critical uncontrollables (obstacle_*, marker_close) should force leaving an action/commit state.",

    # Safety rules
    "Never allow move_forward from obs_front (obstacle_front).",
    "After obstacle_front, do NOT return directly to move_forward; pass through a recovery/escape step (e.g., move_backward and/or rotate/full_rotate).",
    "If marker_close occurs in ANY state, transition to a terminal goal/stop state where stop is the only controllable action.",

    # Anti-oscillation
    "Avoid oscillations: do NOT enable both rotate_clockwise and rotate_counterclockwise as controllables in the same state.",
    "Keep the behavior consistent: obs_left should prefer rotate_clockwise OR a clear escape policy; obs_right should prefer rotate_counterclockwise (or vice versa), but do not alternate rapidly.",

    # Commit-state discipline (fixed)
    "You MAY introduce helper states such as forward_commit, rotate_commit_cw, rotate_commit_ccw, scan_full, recover_back, marker_track, marker_approach.",
    "Commit/scan/recovery states must be ONE-SHOT on their primary controllable action: the primary controllable must transition OUT to a decision/perception state (e.g., clear), NOT self-loop.",
    "Examples (illustrative only): forward_commit: move_forward->clear; rotate_commit_cw: rotate_clockwise->clear; rotate_commit_ccw: rotate_counterclockwise->clear; scan_full: full_rotate->clear; recover_back: move_backward->scan_full or ->clear.",
    "Do NOT create infinite controllable loops like full_rotate->scan_full, move_backward->recover_back, or move_forward->forward_commit.",

    # Marker priority
    "When marker_seen occurs (and marker_close has not occurred), prefer switching to a marker-tracking mode/state where the primary controllable is move_to_marker.",
    "When marker_lost occurs during marker tracking, transition to a search behavior (e.g., scan_full) rather than continuing move_to_marker blindly.",
]

STATE_SEMANTICS = {
    "clear": "no obstacle in front/left/right; safe to advance.",
    "obs_front": "obstacle detected in front region; forward is unsafe.",
    "obs_left": "obstacle close on left; left turn is unsafe; right turn may be preferred.",
    "obs_right": "obstacle close on right; right turn is unsafe; left turn may be preferred.",
}

EVENT_SEMANTICS = {
    "path_clear": "sensors say forward corridor is clear (no front obstacle).",
    "obstacle_front": "front sensor region is blocked.",
    "obstacle_left": "left sensor region is blocked.",
    "obstacle_right": "right sensor region is blocked.",
    "rotate_clockwise": "robot rotates clockwise.",
    "rotate_counterclockwise": "robot rotates counterclockwise.",
    "marker_seen": "marker currently detected in camera.",
    "marker_lost": "marker currently not detected in camera.",
    "marker_close": "marker distance < threshold (goal condition).",
    "move_forward": "apply forward motion for 0.2 s (one pulse).",
    "move_backward": "apply backward motion for 0.2 s (one pulse).",
    "rotate_clockwise": "apply clockwise rotation for 0.2 s (one pulse).",
    "rotate_counterclockwise": "apply counterclockwise rotation for 0.2 s (one pulse).",
    "full_rotate": "rotate 360 degrees (atomic action; completes a full scan).",
    "move_to_marker": "move toward marker for 0.2 s (one pulse) while tracking.",
    "stop": "set velocity to zero (stop).",
}

# Skip JSON->XML and use existing E1.xml in full_pipeline.
START_FROM_XML_ONLY = False
START_FROM_XML_INDEX = "1"

_TRANS_RE = re.compile(r'^\(\s*"(.*?)"\s*,\s*"(.*?)"\s*,\s*"(.*?)"\s*\)\s*$')

def _parse_transition(t: str) -> tuple[str, str, str]:
    m = _TRANS_RE.match(t.strip())
    if not m:
        raise ValueError(f"Bad transition format: {t!r}")
    return m.group(1), m.group(2), m.group(3)

def _validate_llm_json(
    llm_json_path: Path,
    controllable: Sequence[str],
    uncontrollable: Sequence[str],
) -> None:
    data = json.loads(llm_json_path.read_text(encoding="utf-8"))

    if "transitions" not in data or not isinstance(data["transitions"], list):
        raise ValueError("LLM JSON must contain a list field: 'transitions'")

    allowed_events = set(controllable) | set(uncontrollable)

    transitions = [_parse_transition(t) for t in data["transitions"]]

    # 1) Event alphabet check
    bad_events = sorted({ev for _, ev, _ in transitions if ev not in allowed_events})
    if bad_events:
        raise ValueError(f"Unknown events in LLM JSON: {bad_events}")

    # Collect states (from source/target of transitions)
    states: Set[str] = set()
    for s, _, t in transitions:
        states.add(s)
        states.add(t)

    # Build next-state map: (state,event) -> {next}
    nexts: dict[tuple[str, str], Set[str]] = defaultdict(set)
    for s, ev, t in transitions:
        nexts[(s, ev)].add(t)

    # 2) Determinism: exactly one next state per (state,event)
    nondet = [(k, sorted(v)) for k, v in nexts.items() if len(v) > 1]
    if nondet:
        msg = "\n".join([f"  {k} -> {v}" for k, v in nondet[:30]])
        raise ValueError(f"Nondeterministic transitions (same state+event):\n{msg}")

    # 3) Totality on uncontrollables: every state must define all uncontrollable events
    missing = []
    for s in sorted(states):
        for ev in uncontrollable:
            if (s, ev) not in nexts:
                missing.append((s, ev))
    if missing:
        msg = "\n".join([f"  missing ({s}, {ev})" for s, ev in missing[:50]])
        raise ValueError(f"Missing uncontrollable transitions (totality violated):\n{msg}")

    # 4) Must have at least one controllable per state (prevents stalling)
    for s in sorted(states):
        outs_ctrl = [ev for ev in controllable if (s, ev) in nexts]
        if not outs_ctrl:
            raise ValueError(f"State '{s}' has no controllable transitions (may stall).")

    # 5) Prevent infinite controllable self-loops in action-like states
    # With your 0.5s tick / 0.2s hold, controllable self-loops are dangerous.
    trap_prefixes = ("forward_commit", "rotate_commit", "recover", "scan")
    for (s, ev), tgt_set in nexts.items():
        tgt = next(iter(tgt_set))
        if ev in controllable and tgt == s:
            if any(s.startswith(p) for p in trap_prefixes) or s.endswith("_commit"):
                raise ValueError(
                    f"Controllable self-loop in action-like state '{s}' on '{ev}'. "
                    f"This tends to cause repeated motion pulses and collisions."
                )


def _ensure_nadzoru_imports(nadzoru_root: Path) -> None:
    if not nadzoru_root.exists():
        raise SystemExit(f"Nadzoru2 root not found: {nadzoru_root}")
    sys.path.insert(0, str(nadzoru_root))


def read_api_key() -> str:
    env_key = os.getenv("OPENAI_API_KEY")
    if env_key:
        return env_key.strip()
    if DEFAULT_KEY_FILE.exists():
        return DEFAULT_KEY_FILE.read_text(encoding="utf-8").strip()
    raise SystemExit(
        "OpenAI API key not found. Set OPENAI_API_KEY or create "
        f"{DEFAULT_KEY_FILE}"
    )


def _merge_semantics(
    base: Dict[str, str],
    overrides: Dict[str, str] | None,
) -> Dict[str, str]:
    merged = dict(base)
    if overrides:
        merged.update(overrides)
    return merged


def build_prompt(
    goal: str,
    states: List[str],
    controllable: List[str],
    uncontrollable: List[str],
    scenario: str | None,
    guidance: List[str] | None,
    state_semantics: Dict[str, str],
    event_semantics: Dict[str, str],
) -> str:
    states = sorted(states)
    lines: List[str] = []
    lines.append(
        "We operate a DES-based supervisor for a mobile robot with multiple "
        "sensing modes. Each entry describes a sensor switch "
        "(uncontrollable event) or an allowed control action."
    )
    lines.append("Timing: supervisor tick = 0.5 s. If a controllable is chosen, its motion command is applied for 0.2 s (one pulse), then released until the next tick.")
    lines.append("Exception: full_rotate is an atomic 360-degree rotation (complete scan action).")
    lines.append("")
    lines.append("Controllable events: " + ", ".join(controllable))
    lines.append("Uncontrollable events: " + ", ".join(uncontrollable))
    lines.append("Current states: " + ", ".join(states))
    lines.append("")
    state_semantics_lines = [
        f"- {name}: {state_semantics[name]}"
        for name in states
        if name in state_semantics
    ]
    if state_semantics_lines:
        lines.append("State semantics:")
        lines.extend(state_semantics_lines)
        lines.append("")

    event_semantics_lines = [
        f"- {name}: {event_semantics[name]}"
        for name in controllable + uncontrollable
        if name in event_semantics
    ]
    if event_semantics_lines:
        lines.append("Event semantics:")
        lines.extend(event_semantics_lines)
        lines.append("")
    if scenario:
        lines.append("Scenario context: " + scenario)
        lines.append("")
    if guidance:
        lines.append("Behaviour guidance:")
        for item in guidance:
            lines.append(f"- {item}")
        lines.append("")
    lines.append(
        "Design a new transition structure from scratch using the states "
        "above. You may propose additional helper states if justified."
    )
    lines.append("")
    lines.append(f"Goal: {goal}. Avoid oscillations and ensure safe reactions to obstacles.")
    lines.append("")
    lines.append(
        "Return JSON with this schema:\n"
        "{\n"
        '  "rationale": "<short reasoning>",\n'
        '  "new_states": ["optional", "state", "names"],\n'
        '  "transitions": [\n'
        '     "(\\"state\\", \\"event\\", \\"next\\")",\n'
        '     "... additional lines ..."\n'
        '  ],\n'
        '  "strategy": ["bullet list of goal-seeking ideas"]\n'
        "}"
    )

    return "\n".join(lines)


def parse_json_response(content: str) -> Dict[str, str]:
    def strip_line_comments(text: str) -> str:
        out = []
        in_string = False
        escape = False
        i = 0
        while i < len(text):
            ch = text[i]
            if in_string:
                out.append(ch)
                if escape:
                    escape = False
                elif ch == "\\":
                    escape = True
                elif ch == '"':
                    in_string = False
                i += 1
                continue
            if ch == '"':
                in_string = True
                out.append(ch)
                i += 1
                continue
            if ch == "/" and i + 1 < len(text) and text[i + 1] == "/":
                while i < len(text) and text[i] != "\n":
                    i += 1
                continue
            out.append(ch)
            i += 1
        return "".join(out)

    try:
        return json.loads(content)
    except json.JSONDecodeError:
        pass
    fence_match = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", content, re.DOTALL)
    if fence_match:
        snippet = fence_match.group(1)
        try:
            return json.loads(strip_line_comments(snippet))
        except json.JSONDecodeError:
            pass
    start = content.find("{")
    end = content.rfind("}")
    if start != -1 and end != -1 and end > start:
        snippet = content[start : end + 1]
        try:
            return json.loads(strip_line_comments(snippet))
        except json.JSONDecodeError:
            pass
    raise ValueError("Unable to parse JSON response from API:\n" + content)


def call_chat_completion(
    api_key: str,
    prompt: str,
    model: str,
    temperature: float,
    timeout_s: float,
    retries: int,
) -> Dict[str, str]:
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": model,
        "temperature": temperature,
        "messages": [
            {
                "role": "system",
                "content": (
                    "You are a DES/control expert. Respond ONLY with valid JSON "
                    "describing improved supervisor transitions."
                ),
            },
            {"role": "user", "content": prompt},
        ],
    }
    last_error: Exception | None = None
    for attempt in range(1, retries + 2):
        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers=headers,
                data=json.dumps(payload),
                timeout=timeout_s,
            )
            response.raise_for_status()
            content = response.json()["choices"][0]["message"]["content"]
            return parse_json_response(content)
        except requests.exceptions.ReadTimeout as exc:
            last_error = exc
        except requests.exceptions.RequestException as exc:
            last_error = exc
        if attempt <= retries:
            time.sleep(min(2 ** attempt, 8))
    assert last_error is not None
    raise last_error


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


def _next_output_index(output_dir: Path) -> str:
    max_idx = 0
    for path in output_dir.glob("E*.xml"):
        stem = path.stem  # e.g., E12
        if stem.startswith("E"):
            idx = stem[1:]
            if idx.isdigit():
                max_idx = max(max_idx, int(idx))
    for path in output_dir.glob("Sloc*.xml"):
        stem = path.stem  # e.g., Sloc12
        if stem.startswith("Sloc"):
            idx = stem[4:]
            if idx.isdigit():
                max_idx = max(max_idx, int(idx))
    return str(max_idx + 1)


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
    sloc_name = f"Sloc{index}"
    script_path.write_text(
        "\n".join(
            [
                "# Auto-generated script",
                "",
                "Gloc = Sync(G1,G2)",
                f"Kloc = Sync(Gloc, E{index})",
                f"{sloc_name} = SupC(Gloc, Kloc)",
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
        "sloc_name": sloc_name,
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


def _load_automaton(xml_path: Path) -> "Automaton":
    from machine.automaton import Automaton

    automaton = Automaton()
    automaton.load(str(xml_path))
    return automaton


def _run_nadzoru_script(
    script_path: Path,
    automata: Dict[str, "Automaton"],
    output_dir: Path,
    expected_name: str | None = None,
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
        "Supervisor Reduction": Automaton.supervisor_reduction,
        "Labeller": Automaton.labeller,
        "Diagnoser": Automaton.diagnoser,
    }
    loc.update(automata)

    script = script_path.read_text(encoding="utf-8")
    exec(script, {}, loc)

    if expected_name:
        obj = loc.get(expected_name)
        if obj is None:
            raise RuntimeError(f"Script did not define {expected_name}.")
        if not isinstance(obj, Automaton):
            raise RuntimeError(
                f"{expected_name} is not an Automaton (got {type(obj).__name__})."
            )

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
    profile: str,
    supervisor_id: str,
) -> None:
    import json_to_nadzoru_xml

    args = argparse.Namespace(
        json=str(json_path),
        output=str(output_path),
        states=None,
        initial=["clear"],
        marked=None,
        controllable=None,
        uncontrollable=None,
        profile=profile,
        profile_path=str(DEFAULT_PROFILE_PATH),
        supervisor_id=supervisor_id,
    )
    json_to_nadzoru_xml.convert_json_to_xml(args)


def _run_llm(task: str, llm_json: Path) -> None:
    profiles = _load_profiles(DEFAULT_PROFILE_PATH)
    if task not in profiles:
        raise SystemExit(f"Unknown task: {task}")
    profile = profiles[task]

    states = list(DEFAULT_STATES)
    controllable = profile.get("controllable_events")
    uncontrollable = profile.get("uncontrollable_events")
    if not isinstance(controllable, list) or not all(isinstance(x, str) for x in controllable):
        raise SystemExit(f"Profile '{task}' missing valid controllable_events list.")
    if not isinstance(uncontrollable, list) or not all(isinstance(x, str) for x in uncontrollable):
        raise SystemExit(f"Profile '{task}' missing valid uncontrollable_events list.")

    guidance_lines: List[str] = []
    include_default_constraints = profile.get("include_default_constraints", True)
    if include_default_constraints:
        guidance_lines.extend(DEFAULT_CONSTRAINTS)
    profile_constraints = profile.get("constraints")
    if profile_constraints:
        guidance_lines.extend(profile_constraints)

    goal = profile.get("goal") or DEFAULT_GOAL

    profile_state_semantics = profile.get("state_semantics")
    if profile_state_semantics is not None and not isinstance(profile_state_semantics, dict):
        raise SystemExit("Profile state_semantics must be an object mapping names to strings.")
    profile_event_semantics = profile.get("event_semantics")
    if profile_event_semantics is not None and not isinstance(profile_event_semantics, dict):
        raise SystemExit("Profile event_semantics must be an object mapping names to strings.")
    merged_state_semantics = _merge_semantics(STATE_SEMANTICS, profile_state_semantics)
    merged_event_semantics = _merge_semantics(EVENT_SEMANTICS, profile_event_semantics)

    prompt = build_prompt(
        goal=goal,
        states=states,
        controllable=controllable,
        uncontrollable=uncontrollable,
        scenario=None,
        guidance=guidance_lines or None,
        state_semantics=merged_state_semantics,
        event_semantics=merged_event_semantics,
    )

    api_key = read_api_key()
    result = call_chat_completion(
        api_key,
        prompt,
        DEFAULT_MODEL,
        0.2,
        DEFAULT_TIMEOUT_S,
        DEFAULT_RETRIES,
    )
    llm_json.write_text(json.dumps(result, indent=2), encoding="utf-8")


def _load_profiles(path: Path) -> Dict[str, Dict[str, object]]:
    if not path.exists():
        raise SystemExit(f"Profile file not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the full DES->Nadzoru->SCT pipeline."
    )
    parser.add_argument(
        "--task",
        required=True,
        help="Task profile to run (e.g., explore, find_marker, zigzag, wall_follow).",
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
    args = parser.parse_args()

    profiles = _load_profiles(DEFAULT_PROFILE_PATH)
    if args.task not in profiles:
        raise SystemExit(f"Unknown task: {args.task}")

    task_profile = profiles[args.task]
    controllable_events = task_profile.get("controllable_events")
    uncontrollable_events = task_profile.get("uncontrollable_events")

    if not isinstance(controllable_events, list) or not all(isinstance(x, str) for x in controllable_events):
        raise SystemExit(f"Profile '{args.task}' missing valid controllable_events list.")
    if not isinstance(uncontrollable_events, list) or not all(isinstance(x, str) for x in uncontrollable_events):
        raise SystemExit(f"Profile '{args.task}' missing valid uncontrollable_events list.")

    run_llm = args.run_llm and not args.skip_llm
    if args.task not in TASK_AUTOMATA_DIRS:
        raise SystemExit(f"No automata folder configured for task: {args.task}")
    source_automata_dir = TASK_AUTOMATA_DIRS[args.task]


    if run_llm:
        DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        llm_json_path = _next_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)
        _run_llm(args.task, llm_json_path)
    else:
        llm_json_path = _latest_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)

    if not run_llm and not START_FROM_XML_ONLY and not llm_json_path.exists():
        raise SystemExit(f"Expected input not found: {llm_json_path}")

    index = START_FROM_XML_INDEX if START_FROM_XML_ONLY else _next_output_index(DEFAULT_OUTPUT_DIR)
    paths = _prepare_output_dir(
        DEFAULT_OUTPUT_DIR,
        source_automata_dir,
        index,
        START_FROM_XML_ONLY,
    )
    if not START_FROM_XML_ONLY:
        # Hard gate: reject bad supervisors early
        _validate_llm_json(llm_json_path, controllable_events, uncontrollable_events)

        _convert_json_to_xml(
            llm_json_path,
            paths["e_xml"],
            args.task,
            DEFAULT_SUPERVISOR_ID,
        )
    elif not paths["e_xml"].exists():
        raise SystemExit(f"Expected input not found: {paths['e_xml']}")

    _ensure_nadzoru_imports(DEFAULT_NADZORU_ROOT)
    # Load previously generated automatons from the output dir only.
    # Explicitly load G1/G2 from the task source dir to avoid name collisions
    # (e.g., preexisting Sloc*.xml in hardcoded folders).
    automata = _load_automatons([DEFAULT_OUTPUT_DIR])
    automata["G1"] = _load_automaton(paths["g1"])
    automata["G2"] = _load_automaton(paths["g2"])
    try:
        _run_nadzoru_script(
            paths["script"],
            automata,
            DEFAULT_OUTPUT_DIR,
            expected_name=paths["sloc_name"],
        )
    except Exception as exc:
        raise SystemExit(
            "Nadzoru script failed. "
            f"Script: {paths['script']} "
            f"E XML: {paths['e_xml']} "
            f"Source automata: {source_automata_dir} "
            f"Error: {exc}"
        ) from exc

    if not paths["sloc_xml"].exists():
        raise SystemExit(
            "Expected output not found. "
            f"Missing: {paths['sloc_xml']} "
            f"Inputs: E={paths['e_xml']} G1={paths['g1']} G2={paths['g2']} "
            f"Script={paths['script']}"
        )

    from machine.automaton import Automaton

    sloc_automatons = []
    automaton = Automaton()
    automaton.load(str(paths["sloc_xml"]))
    sloc_automatons.append(automaton)

    yaml_payload = _build_sct_yaml(sloc_automatons)
    yaml_real_out = DEFAULT_REAL_YAML_OUT_DIR / f"{args.task}_{DEFAULT_YAML_PREFIX}{index}.yaml"
    yaml_real_default = DEFAULT_REAL_YAML_OUT_DIR / "sup_gpt.yaml"
    yaml_out.parent.mkdir(parents=True, exist_ok=True)
    yaml_real_out.parent.mkdir(parents=True, exist_ok=True)
    for path in (
        yaml_real_out,
        yaml_real_default,
    ):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(yaml_payload, f, sort_keys=False, default_flow_style=True)

    print(f"Wrote YAML to {yaml_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
