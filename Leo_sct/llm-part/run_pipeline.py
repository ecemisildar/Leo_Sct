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
import ast
import json
import os
import re
import sys
import time
import warnings
from pathlib import Path
from typing import Dict, List, Sequence, Set

from collections import defaultdict

import requests
import yaml
from pipeline_prompts import (
    DEFAULT_CONSTRAINTS,
    SYSTEM_PROMPT,
    build_pipeline_prompt,
    load_example_tasks,
    merge_prompt_semantics,
)

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
DEFAULT_NADZORU_ROOT = Path.home() / "Documents/Nadzoru2"


DEFAULT_REAL_YAML_OUT_DIR = (
    Path.home()
    / "ros2_ws/src/Leo_sct/swarm_basics/config"
)

DEFAULT_PROFILE_PATH = THIS_DIR / "task_profiles.json"
DEFAULT_OUTPUT_DIR = THIS_DIR / "generated_pipeline_outputs"
DEFAULT_LLM_DIR = THIS_DIR
DEFAULT_LLM_SUFFIX = "_nadzoru.json"
DEFAULT_YAML_PREFIX = "sup_gpt_"

# Hardcoded LLM options (used only if RUN_LLM is True).
RUN_LLM = True

# LLM defaults (shared with the LLM-only flow).
DEFAULT_KEY_FILE = THIS_DIR.parent / "api_key.txt"
DEFAULT_MODEL = "gpt-4.1"
DEFAULT_GOAL = "Find the marker object and get close to it and stop while staying safe"
DEFAULT_TIMEOUT_S = 120.0
DEFAULT_RETRIES = 2

# Skip JSON->XML and use an existing E1.xml in the generated output folder.
START_FROM_XML_ONLY = False
START_FROM_XML_INDEX = "1"

_TRANS_RE = re.compile(r'^\(\s*"(.*?)"\s*,\s*"(.*?)"\s*,\s*"(.*?)"\s*\)\s*$')

def _parse_transition(t: str) -> tuple[str, str, str]:
    m = _TRANS_RE.match(t.strip())
    if not m:
        raise ValueError(f"Bad transition format: {t!r}")
    return m.group(1), m.group(2), m.group(3)


def _add_missing_uncontrollable_self_loops(
    transitions: List[tuple[str, str, str]],
    uncontrollable: Sequence[str],
) -> List[tuple[str, str, str]]:
    states: Set[str] = set()
    for s, _, t in transitions:
        states.add(s)
        states.add(t)

    existing = {(s, ev) for s, ev, _ in transitions}
    completed = list(transitions)
    for s in sorted(states):
        for ev in uncontrollable:
            if (s, ev) not in existing:
                completed.append((s, ev, s))
    return completed

def _validate_llm_json(
    llm_json_path: Path,
    controllable: Sequence[str],
    uncontrollable: Sequence[str],
    task: str | None = None,
) -> None:
    data = json.loads(llm_json_path.read_text(encoding="utf-8"))

    if "transitions" not in data or not isinstance(data["transitions"], list):
        raise ValueError("LLM JSON must contain a list field: 'transitions'")

    allowed_events = set(controllable) | set(uncontrollable)

    transitions = [_parse_transition(t) for t in data["transitions"]]
    transitions = _add_missing_uncontrollable_self_loops(transitions, uncontrollable)

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

    # 4) Must have at least one controllable per non-terminal state.
    # Allow explicit terminal sinks with no controllables if they self-loop on
    # all uncontrollables.
    for s in sorted(states):
        outs_ctrl = [ev for ev in controllable if (s, ev) in nexts]
        if not outs_ctrl:
            terminal_sink = True
            for ev in uncontrollable:
                tgt = next(iter(nexts[(s, ev)]))
                if tgt != s:
                    terminal_sink = False
                    break
            if not terminal_sink:
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

    # 6) marker_close safety invariant:
    # marker_close must always lead to a terminal state where stop is the ONLY controllable
    if "marker_close" in uncontrollable:
        for (s, ev), tgt_set in nexts.items():
            if ev != "marker_close":
                continue
            tgt = next(iter(tgt_set))
            tgt_ctrls = [e for e in controllable if (tgt, e) in nexts]
            if tgt_ctrls != ["stop"]:
                raise ValueError(
                    f"Safety violation: marker_close from '{s}' leads to '{tgt}', "
                    f"which has controllables {tgt_ctrls} — must be exactly ['stop']. "
                    f"The robot must stop when the marker is reached."
                )
            # Also check the terminal state doesn't transition away on uncontrollables
            # (other than self-loops) — prevents escaping the goal state
            for uc in uncontrollable:
                if (tgt, uc) in nexts:
                    uc_tgt = next(iter(nexts[(tgt, uc)]))
                    if uc_tgt != tgt:
                        raise ValueError(
                            f"Safety violation: terminal goal state '{tgt}' transitions "
                            f"away on uncontrollable '{uc}' to '{uc_tgt}'. "
                            f"Goal state must self-loop on all uncontrollables."
                        )            

    if task == "explore":
        ctrl_by_state = {
            s: [ev for ev in controllable if (s, ev) in nexts]
            for s in sorted(states)
        }
        explore_warnings: List[str] = []

        def single_ctrl_for_next_state(src_state: str, obstacle_event: str) -> tuple[str, List[str]]:
            target = next(iter(nexts[(src_state, obstacle_event)]))
            ctrls = ctrl_by_state.get(target, [])
            return target, ctrls

        for s in sorted(states):
            front_target, front_ctrls = single_ctrl_for_next_state(s, "obstacle_front")
            if not front_ctrls:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_front from '{s}' leads to '{front_target}' "
                    "with no controllable recovery action."
                )
                continue
            recovery_front = [
                ev for ev in front_ctrls
                if ev in {"full_rotate", "rotate_clockwise", "rotate_counterclockwise"}
            ]
            if not recovery_front:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_front from '{s}' leads to '{front_target}' "
                    f"with controllables {front_ctrls}. Expected at least one recovery action."
                )

            left_target, left_ctrls = single_ctrl_for_next_state(s, "obstacle_left")
            if not left_ctrls:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_left from '{s}' leads to '{left_target}' "
                    "with no controllable recovery action."
                )
                continue
            left_recovery = [
                ev for ev in left_ctrls
                if ev in {"rotate_clockwise", "full_rotate"}
            ]
            if not left_recovery:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_left from '{s}' leads to '{left_target}' "
                    f"with controllables {left_ctrls}. Expected a left-side recovery action."
                )

            right_target, right_ctrls = single_ctrl_for_next_state(s, "obstacle_right")
            if not right_ctrls:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_right from '{s}' leads to '{right_target}' "
                    "with no controllable recovery action."
                )
                continue
            right_recovery = [
                ev for ev in right_ctrls
                if ev in {"rotate_counterclockwise", "full_rotate"}
            ]
            if not right_recovery:
                explore_warnings.append(
                    f"Explore safety violation: obstacle_right from '{s}' leads to '{right_target}' "
                    f"with controllables {right_ctrls}. Expected a right-side recovery action."
                )

            for target, ctrls, event_name in (
                (front_target, front_ctrls, "obstacle_front"),
                (left_target, left_ctrls, "obstacle_left"),
                (right_target, right_ctrls, "obstacle_right"),
            ):
                bad = [ev for ev in ctrls if ev in {"move_forward", "random_walk"}]
                if event_name == "obstacle_front" and recovery_front:
                    bad = [ev for ev in bad if ev == "random_walk"]
                if bad:
                    explore_warnings.append(
                        f"Explore safety violation: {event_name} from '{s}' leads to '{target}' "
                        f"with forbidden controllables {bad}."
                    )
        for msg in explore_warnings:
            warnings.warn(msg)


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

def parse_json_response(content: str) -> Dict[str, object]:
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

    def normalize_payload(data: object) -> Dict[str, object]:
        if not isinstance(data, dict):
            raise ValueError(f"Expected a JSON object, got {type(data).__name__}.")
        data = dict(data)

        def normalize_transition_list(transitions: object) -> List[str]:
            if not isinstance(transitions, list):
                raise ValueError("Transitions must be a list.")
            normalized = []
            for item in transitions:
                if isinstance(item, str):
                    normalized.append(item)
                elif isinstance(item, (tuple, list)) and len(item) == 3 and all(isinstance(x, str) for x in item):
                    s, ev, t = item
                    normalized.append(f'(\"{s}\", \"{ev}\", \"{t}\")')
                else:
                    raise ValueError(f"Unsupported transition entry: {item!r}")
            return normalized

        transitions = data.get("transitions")
        if isinstance(transitions, list):
            data["transitions"] = normalize_transition_list(transitions)

        for section_name in ("G", "E"):
            section = data.get(section_name)
            if not isinstance(section, list):
                continue
            normalized_section = []
            for automaton in section:
                if not isinstance(automaton, dict):
                    raise ValueError(f"{section_name} entries must be objects.")
                automaton = dict(automaton)
                if "transitions" in automaton:
                    automaton["transitions"] = normalize_transition_list(automaton["transitions"])
                normalized_section.append(automaton)
            data[section_name] = normalized_section
        return data

    def try_json(text: str):
        try:
            return normalize_payload(json.loads(text))
        except json.JSONDecodeError:
            return None

    def try_python_literal(text: str):
        try:
            return normalize_payload(ast.literal_eval(text))
        except (ValueError, SyntaxError):
            return None

    stripped = strip_line_comments(content)
    parsed = try_json(stripped)
    if parsed is not None:
        return parsed

    fence_match = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", content, re.DOTALL)
    if fence_match:
        snippet = strip_line_comments(fence_match.group(1))
        parsed = try_json(snippet) or try_python_literal(snippet)
        if parsed is not None:
            return parsed

    start = content.find("{")
    end = content.rfind("}")
    if start != -1 and end != -1 and end > start:
        snippet = strip_line_comments(content[start : end + 1])
        parsed = try_json(snippet) or try_python_literal(snippet)
        if parsed is not None:
            return parsed

    parsed = try_python_literal(stripped)
    if parsed is not None:
        return parsed

    raise ValueError("Unable to parse JSON response from API:\n" + content)


def call_chat_completion(
    api_key: str,
    prompt: str,
    model: str,
    temperature: float,
    timeout_s: float,
    retries: int,
) -> Dict[str, object]:
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
                "content": SYSTEM_PROMPT,
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


def _normalize_name_list(values: object, field_name: str) -> List[str]:
    if values is None:
        return []
    if not isinstance(values, list) or not all(isinstance(item, str) for item in values):
        raise ValueError(f"{field_name} must be a list of strings.")
    return list(values)


def _normalize_automaton_payload(
    section_name: str,
    position: int,
    payload: object,
) -> Dict[str, object]:
    if not isinstance(payload, dict):
        raise ValueError(f"{section_name}[{position}] must be an object.")
    normalized = dict(payload)
    transitions = normalized.get("transitions")
    if not isinstance(transitions, list) or not transitions:
        raise ValueError(f"{section_name}[{position}] must include a non-empty transitions list.")
    parsed_transitions = [_parse_transition(transition) for transition in transitions]

    normalized["name"] = str(normalized.get("name") or f"{section_name}{position + 1}")
    normalized["purpose"] = str(normalized.get("purpose") or "")
    normalized["states"] = _normalize_name_list(normalized.get("states"), f"{section_name}[{position}].states")
    normalized["initial_states"] = _normalize_name_list(
        normalized.get("initial_states"),
        f"{section_name}[{position}].initial_states",
    )
    normalized["marked_states"] = _normalize_name_list(
        normalized.get("marked_states"),
        f"{section_name}[{position}].marked_states",
    )
    normalized["controllable_events"] = _normalize_name_list(
        normalized.get("controllable_events"),
        f"{section_name}[{position}].controllable_events",
    )
    normalized["uncontrollable_events"] = _normalize_name_list(
        normalized.get("uncontrollable_events"),
        f"{section_name}[{position}].uncontrollable_events",
    )
    parsed_transitions = _add_missing_uncontrollable_self_loops(
        parsed_transitions,
        normalized["uncontrollable_events"],
    )
    normalized["transitions"] = [
        f'("{src}", "{ev}", "{dst}")'
        for src, ev, dst in parsed_transitions
    ]
    return normalized


def _is_empty_automaton_payload(payload: object) -> bool:
    if not isinstance(payload, dict):
        return False
    transitions = payload.get("transitions")
    if transitions not in (None, []):
        return False
    for key in (
        "states",
        "initial_states",
        "marked_states",
        "controllable_events",
        "uncontrollable_events",
    ):
        value = payload.get(key)
        if value not in (None, []):
            return False
    return not str(payload.get("name") or "").strip() and not str(payload.get("purpose") or "").strip()


def _normalize_pipeline_payload(payload: Dict[str, object]) -> Dict[str, object]:
    normalized = dict(payload)
    g_list = normalized.get("G")
    e_list = normalized.get("E")
    if not isinstance(g_list, list) or not g_list:
        raise ValueError("API response must contain a non-empty 'G' list.")
    if not isinstance(e_list, list) or not e_list:
        raise ValueError("API response must contain a non-empty 'E' list.")

    normalized["G"] = [
        _normalize_automaton_payload("G", index, entry)
        for index, entry in enumerate(g_list)
        if not _is_empty_automaton_payload(entry)
    ]
    normalized["E"] = [
        _normalize_automaton_payload("E", index, entry)
        for index, entry in enumerate(e_list)
        if not _is_empty_automaton_payload(entry)
    ]
    if not normalized["G"]:
        raise ValueError("API response 'G' list only contained empty automata.")
    if not normalized["E"]:
        raise ValueError("API response 'E' list only contained empty automata.")

    event_controllability: Dict[str, bool] = {}
    used_names: Set[str] = set()
    for section_name in ("G", "E"):
        for automaton in normalized[section_name]:
            name = str(automaton["name"])
            if name in used_names:
                raise ValueError(f"Duplicate automaton name in API response: {name}")
            used_names.add(name)

            for event in automaton["controllable_events"]:
                prev = event_controllability.setdefault(event, True)
                if prev is not True:
                    raise ValueError(f"Event '{event}' has conflicting controllability across automata.")
            for event in automaton["uncontrollable_events"]:
                prev = event_controllability.setdefault(event, False)
                if prev is not False:
                    raise ValueError(f"Event '{event}' has conflicting controllability across automata.")
    return normalized


def _write_automaton_xml(automaton_payload: Dict[str, object], output_path: Path) -> None:
    import json_to_nadzoru_xml

    transitions = automaton_payload["transitions"]
    transition_dicts = [json_to_nadzoru_xml._parse_transition_line(line) for line in transitions]
    state_names = json_to_nadzoru_xml._collect_states(
        automaton_payload["states"],
        None,
        transition_dicts,
    )
    if not state_names:
        raise ValueError(f"No states detected for automaton {automaton_payload['name']}.")

    initial_states = automaton_payload["initial_states"] or state_names[:1]
    marked_states = automaton_payload["marked_states"] or state_names
    states = json_to_nadzoru_xml._states_with_flags(state_names, initial_states, marked_states)
    events = json_to_nadzoru_xml._event_entries(
        automaton_payload["controllable_events"],
        automaton_payload["uncontrollable_events"],
        transition_dicts,
    )
    xml_payload = json_to_nadzoru_xml._build_xml(
        str(automaton_payload["name"]),
        states,
        events,
        transition_dicts,
    )
    output_path.write_bytes(xml_payload)


def _prepare_output_dir(
    output_dir: Path,
    index: str,
    llm_payload: Dict[str, object] | None,
    keep_existing_e: bool,
) -> Dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    sloc_xml = output_dir / f"Sloc{index}.xml"
    gloc_xml = output_dir / f"Gloc{index}.xml"
    kloc_xml = output_dir / f"Kloc{index}.xml"
    script_path = output_dir / "script.txt"
    disabled_report = output_dir / f"disabled_transitions_{index}.json"
    sloc_name = f"Sloc{index}"
    gloc_name = f"Gloc{index}"
    kloc_name = f"Kloc{index}"
    g_xml_paths: List[Path] = []
    e_xml_paths: List[Path] = []

    if llm_payload is not None:
        g_names: List[str] = []
        e_names: List[str] = []
        for pos, automaton in enumerate(llm_payload["G"], start=1):
            path = output_dir / f"G{index}_{pos}.xml"
            _write_automaton_xml(automaton, path)
            g_xml_paths.append(path)
            g_names.append(str(automaton["name"]))
        for pos, automaton in enumerate(llm_payload["E"], start=1):
            path = output_dir / f"E{index}_{pos}.xml"
            _write_automaton_xml(automaton, path)
            e_xml_paths.append(path)
            e_names.append(str(automaton["name"]))

        plant_expr = g_names[0]
        for name in g_names[1:]:
            plant_expr = f"Sync({plant_expr}, {name})"

        spec_expr = plant_expr
        for name in e_names:
            spec_expr = f"Sync({spec_expr}, {name})"

        script_path.write_text(
            "\n".join(
                [
                    "# Auto-generated script",
                    "",
                    f"{gloc_name} = {plant_expr}",
                    f"{kloc_name} = {spec_expr}",
                    f"{sloc_name} = SupC({gloc_name}, {kloc_name})",
                    "",
                ]
            ),
            encoding="utf-8",
        )
    else:
        e_xml = output_dir / f"E{index}.xml"
        if not keep_existing_e or not e_xml.exists():
            raise SystemExit("START_FROM_XML_ONLY requires an existing E XML file.")
        e_xml_paths.append(e_xml)
    return {
        "e_xml": e_xml_paths[0] if len(e_xml_paths) == 1 else None,
        "e_xml_paths": e_xml_paths,
        "sloc_xml": sloc_xml,
        "gloc_xml": gloc_xml,
        "kloc_xml": kloc_xml,
        "script": script_path,
        "disabled_report": disabled_report,
        "g_xml_paths": g_xml_paths,
        "g_names": g_names if llm_payload is not None else [],
        "e_names": e_names if llm_payload is not None else [],
        "gloc_name": gloc_name,
        "kloc_name": kloc_name,
        "sloc_name": sloc_name,
    }

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


def _save_disabled_transition_report(
    gloc: "Automaton",
    sloc: "Automaton",
    report_path: Path,
) -> None:
    gloc_states = list(gloc.states)
    gloc_by_name = {state.name: state for state in gloc_states}
    sorted_gloc_names = sorted(gloc_by_name.keys(), key=len, reverse=True)

    def match_gloc_state_name(sloc_state_name: str) -> str | None:
        for name in sorted_gloc_names:
            if sloc_state_name == name or sloc_state_name.startswith(name + ","):
                return name
        return None

    report = {
        "gloc_name": gloc.get_id_name(),
        "sloc_name": sloc.get_id_name(),
        "states": [],
    }

    for sloc_state in sorted(sloc.states, key=lambda s: s.name):
        gloc_state_name = match_gloc_state_name(sloc_state.name)
        if gloc_state_name is None:
            report["states"].append(
                {
                    "sloc_state": sloc_state.name,
                    "gloc_state": None,
                    "enabled_transitions": [
                        {
                            "event": t.event.name,
                            "target": t.to_state.name,
                            "controllable": bool(t.event.controllable),
                        }
                        for t in sorted(
                            sloc_state.out_transitions,
                            key=lambda t: (t.event.name, t.to_state.name),
                        )
                    ],
                    "disabled_transitions": [],
                    "note": "No matching Gloc state name prefix found.",
                }
            )
            continue

        gloc_state = gloc_by_name[gloc_state_name]
        enabled_events = {t.event.name for t in sloc_state.out_transitions}
        disabled = []
        for transition in sorted(
            gloc_state.out_transitions,
            key=lambda t: (t.event.name, t.to_state.name),
        ):
            if transition.event.name in enabled_events:
                continue
            disabled.append(
                {
                    "event": transition.event.name,
                    "target": transition.to_state.name,
                    "controllable": bool(transition.event.controllable),
                }
            )

        report["states"].append(
            {
                "sloc_state": sloc_state.name,
                "gloc_state": gloc_state_name,
                "enabled_transitions": [
                    {
                        "event": t.event.name,
                        "target": t.to_state.name,
                        "controllable": bool(t.event.controllable),
                    }
                    for t in sorted(
                        sloc_state.out_transitions,
                        key=lambda t: (t.event.name, t.to_state.name),
                    )
                ],
                "disabled_transitions": disabled,
            }
        )

    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

def _check_sloc_valid(sloc_path: Path, sloc_name: str) -> None:
    from machine.automaton import Automaton

    a = Automaton()
    a.load(str(sloc_path))

    states = list(a.states)
    if len(states) == 0:
        raise SystemExit(
            f"SupC produced an empty supervisor '{sloc_name}' — "
            f"the specification is likely not controllable w.r.t. the plant. "
            f"Check {sloc_path} and review the LLM-generated transitions."
        )

    if a.initial_state is None:
        raise SystemExit(
            f"Supervisor '{sloc_name}' has no initial state — "
            f"it cannot be executed. Check {sloc_path}."
        )

    # Check for blocking: every state should have at least one outgoing transition
    blocking_states = [
        s.name for s in states
        if len(list(s.out_transitions)) == 0 and s != a.initial_state
    ]
    if blocking_states:
        raise SystemExit(
            f"Supervisor '{sloc_name}' has blocking (dead-end) states with no "
            f"outgoing transitions: {blocking_states}. The robot may get permanently stuck."
        )


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

def _run_llm(
    task: str,
    llm_json: Path,
    user_task: str | None = None,
    print_prompt: bool = False,
) -> None:
    profiles = _load_profiles(DEFAULT_PROFILE_PATH)
    if task not in profiles:
        raise SystemExit(f"Unknown task: {task}")
    profile = profiles[task]

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
    merged_state_semantics, merged_event_semantics = merge_prompt_semantics(
        profile_state_semantics,
        profile_event_semantics,
    )

    prompt = build_pipeline_prompt(
        goal=goal,
        user_task=user_task,
        controllable=controllable,
        uncontrollable=uncontrollable,
        guidance=guidance_lines or None,
        state_semantics=merged_state_semantics,
        event_semantics=merged_event_semantics,
        example_tasks=load_example_tasks(),
    )

    if print_prompt:
        print("----- BEGIN PIPELINE PROMPT -----")
        print(prompt)
        print("----- END PIPELINE PROMPT -----")

    api_key = read_api_key()
    result = call_chat_completion(
        api_key,
        prompt,
        DEFAULT_MODEL,
        0.0,
        DEFAULT_TIMEOUT_S,
        DEFAULT_RETRIES,
    )
    llm_json.write_text(
        json.dumps(_normalize_pipeline_payload(result), indent=2),
        encoding="utf-8",
    )


def _load_profiles(path: Path) -> Dict[str, Dict[str, object]]:
    if not path.exists():
        raise SystemExit(f"Profile file not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _read_optional_text(value: str | None, file_path: str | None) -> str | None:
    if value and file_path:
        raise SystemExit("Use only one of --user-task or --user-task-file.")
    if file_path:
        text = Path(file_path).read_text(encoding="utf-8").strip()
        return text or None
    if value:
        text = value.strip()
        return text or None
    return None


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
    parser.add_argument(
        "--user-task",
        help="Free-form user task text. Kept separate from the default task profile constraints.",
    )
    parser.add_argument(
        "--user-task-file",
        help="Path to a text file containing the free-form user task.",
    )
    parser.add_argument(
        "--print-prompt",
        action="store_true",
        help="Print the final composed LLM prompt before running the pipeline.",
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
    user_task = _read_optional_text(args.user_task, args.user_task_file)
    if run_llm:
        DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        llm_json_path = _next_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)
        _run_llm(
            args.task,
            llm_json_path,
            user_task=user_task,
            print_prompt=args.print_prompt,
        )
    else:
        llm_json_path = _latest_llm_json_path(DEFAULT_OUTPUT_DIR, args.task)

    if not run_llm and not START_FROM_XML_ONLY and not llm_json_path.exists():
        raise SystemExit(f"Expected input not found: {llm_json_path}")
    if START_FROM_XML_ONLY:
        raise SystemExit(
            "START_FROM_XML_ONLY is not supported with the generated G/E pipeline. "
            "Use --skip-llm with a saved JSON response that contains both G and E."
        )

    index = START_FROM_XML_INDEX if START_FROM_XML_ONLY else _next_output_index(DEFAULT_OUTPUT_DIR)
    llm_payload = None
    if not START_FROM_XML_ONLY:
        llm_payload = _normalize_pipeline_payload(
            json.loads(llm_json_path.read_text(encoding="utf-8"))
        )
    paths = _prepare_output_dir(
        DEFAULT_OUTPUT_DIR,
        index,
        llm_payload,
        START_FROM_XML_ONLY,
    )
    if not START_FROM_XML_ONLY:
        for automaton in llm_payload["E"]:
            temp_json = {
                "transitions": automaton["transitions"],
                "new_states": automaton["states"],
            }
            temp_path = DEFAULT_OUTPUT_DIR / f".tmp_{automaton['name']}_{index}.json"
            temp_path.write_text(json.dumps(temp_json), encoding="utf-8")
            try:
                _validate_llm_json(
                    temp_path,
                    automaton["controllable_events"],
                    automaton["uncontrollable_events"],
                    task=args.task,
                )
            finally:
                temp_path.unlink(missing_ok=True)
    elif not paths["e_xml"].exists():
        raise SystemExit(f"Expected input not found: {paths['e_xml']}")

    _ensure_nadzoru_imports(DEFAULT_NADZORU_ROOT)
    automata: Dict[str, "Automaton"] = {}
    for name, xml_path in zip(paths["g_names"], paths["g_xml_paths"]):
        automata[name] = _load_automaton(xml_path)
    for name, xml_path in zip(paths["e_names"], paths["e_xml_paths"]):
        automata[name] = _load_automaton(xml_path)
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
            f"E XMLs: {paths['e_xml_paths']} "
            f"G XMLs: {paths['g_xml_paths']} "
            f"Error: {exc}"
        ) from exc

    if not paths["sloc_xml"].exists():
        raise SystemExit(
            "Expected output not found. "
            f"Missing: {paths['sloc_xml']} "
            f"Inputs: E={paths['e_xml_paths']} G={paths['g_xml_paths']} "
            f"Script={paths['script']}"
        )
    _check_sloc_valid(paths["sloc_xml"], paths["sloc_name"])    
    _save_disabled_transition_report(
        _load_automaton(paths["gloc_xml"]),
        _load_automaton(paths["sloc_xml"]),
        paths["disabled_report"],
    )

    from machine.automaton import Automaton

    sloc_automatons = []
    automaton = Automaton()
    automaton.load(str(paths["sloc_xml"]))
    sloc_automatons.append(automaton)

    yaml_payload = _build_sct_yaml(sloc_automatons)
    yaml_real_out = DEFAULT_REAL_YAML_OUT_DIR / f"{args.task}_{DEFAULT_YAML_PREFIX}{index}.yaml"
    yaml_real_default = DEFAULT_REAL_YAML_OUT_DIR / "sup_gpt.yaml"
    yaml_real_out.parent.mkdir(parents=True, exist_ok=True)
    for path in (
        yaml_real_out,
        yaml_real_default,
    ):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(yaml_payload, f, sort_keys=False, default_flow_style=True)

    print(f"Wrote YAML to {yaml_real_out} and {yaml_real_default}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
