#!/usr/bin/env python3
"""Generate separate G and E JSON/XML files for basic_sim using the LLM prompt flow."""

from __future__ import annotations

import argparse
import contextlib
import json
import re
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
BASIC_SIM_DIR = Path(__file__).resolve().parent
LLM_DIR = REPO_ROOT / "llm-part"
DEFAULT_USER_PROMPTS_FILE = BASIC_SIM_DIR / "user_prompts.txt"
DEFAULT_OUTPUT_DIR = BASIC_SIM_DIR / "llm_generated"
DEFAULT_REAL_YAML_OUT_DIR = REPO_ROOT / "leo_real" / "config"
DEFAULT_ROBOT_CONFIG_OUTPUT = DEFAULT_REAL_YAML_OUT_DIR / "sup_gpt.yaml"
DEFAULT_SAVE_CONFIG_YAML = True
if str(BASIC_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_SIM_DIR))
if str(LLM_DIR) not in sys.path:
    sys.path.insert(0, str(LLM_DIR))

from run_pipeline import (  # noqa: E402
    DEFAULT_CONSTRAINTS,
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT_S,
    DEFAULT_RETRIES,
    EVENT_SEMANTICS,
    STATE_SEMANTICS,
    _build_sct_yaml,
    _merge_semantics,
    call_chat_completion,
    read_api_key,
)
import json_to_nadzoru_xml  # noqa: E402
from synthesize_sloc import DEFAULT_NADZORU_ROOT, import_nadzoru, synthesize as synthesize_sloc  # noqa: E402


DEFAULT_CONTROLLABLE = [
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
]

DEFAULT_UNCONTROLLABLE = [
    "path_clear",
    "obstacle_front",
    "obstacle_left",
    "obstacle_right",
]

DEFAULT_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement while still allowing "
    "all six controllable actions somewhere in the automaton."
)

RED_CONTROLLABLE = ["go_to_red"]
RED_UNCONTROLLABLE = ["red_detected"]
BLUE_CONTROLLABLE = ["escape_blue"]
BLUE_UNCONTROLLABLE = ["blue_detected"]
OBJECT_CONTROLLABLE = ["push"]
OBJECT_UNCONTROLLABLE = ["detect_object_red"]
OBJECT_GREEN_UNCONTROLLABLE = ["detect_object_green"]
GREEN_CONTROLLABLE = ["go_to_green"]
GREEN_UNCONTROLLABLE = ["green_detected"]
YELLOW_CONTROLLABLE = ["escape_yellow"]
YELLOW_UNCONTROLLABLE = ["yellow_detected"]
RED_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots gather "
    "in a 2x2 red target area while still allowing all seven controllable actions "
    "somewhere in the automaton."
)
BLUE_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots escape "
    "from a 2x2 blue danger area while still allowing all seven controllable "
    "actions somewhere in the automaton."
)
RED_BLUE_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement, make robots gather in "
    "a 2x2 red target area, and make robots avoid or escape from a separate 2x2 "
    "blue danger area while still allowing all eight controllable actions "
    "somewhere in the automaton."
)
OBJECT_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots push "
    "four 1-cell objects into a 2x2 red target area while still allowing all "
    "eight controllable actions somewhere in the automaton."
)
OBJECT_GREEN_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots push "
    "four 1-cell objects into a 2x2 green target area while still allowing all "
    "eight controllable actions somewhere in the automaton."
)
GREEN_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots gather "
    "in a 2x2 green target area while still allowing all seven controllable "
    "actions somewhere in the automaton."
)
YELLOW_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement and make robots escape "
    "from a 2x2 yellow danger area while still allowing all seven controllable "
    "actions somewhere in the automaton."
)
GREEN_YELLOW_GOAL = (
    "Design a small grid-world DES model for multiple agents. The plant G should "
    "model which perception and motion events can occur. The specification E "
    "should constrain motion for obstacle-aware movement, make robots gather in "
    "a 2x2 green target area, and make robots avoid or escape from a separate "
    "2x2 yellow danger area while still allowing all eight controllable actions "
    "somewhere in the automaton."
)

BASIC_SIM_CONSTRAINTS = [
    "Return exactly one plant automaton in G and exactly one specification automaton in E.",
    "Keep the automata small enough to inspect manually.",
    "The plant G may be permissive, but it must include all provided events.",
    "The specification E should make move_backward reachable only from front-obstacle recovery states.",
    "Do not use move_backward from clear/path_clear states.",
    "Use stop only when the robot should intentionally hold position, such as after reaching a goal or when no safe movement is available.",
    "Obstacle states should prefer directed rotations or short recovery before returning to clear.",
]

RED_TASK_CONSTRAINTS = [
    "The specification E should prioritize go_to_red when the robot is not in the red area.",
    "When red_detected occurs, the robot has reached the red area and should remain there or use only minimal safe correction.",
]

BLUE_TASK_CONSTRAINTS = [
    "The specification E should prioritize escape_blue immediately after blue_detected.",
    "When blue_detected occurs, the robot is inside the blue danger area and should leave it before resuming normal movement.",
    "After escape_blue, return to obstacle-aware movement and avoid re-entering the blue area when possible.",
]

OBJECT_TASK_CONSTRAINTS = [
    "The specification E should use push as the main progress action for moving objects toward the red area.",
    "When detect_object_red occurs, all movable objects are in the red area and the robot should stop or hold position.",
    "For object-pushing tasks, do not stop just because red_detected occurs; stop only after detect_object_red.",
    "Movable objects are not obstacles for sensing: if an object is ahead, push should remain available rather than switching to obstacle recovery.",
    "Obstacle recovery should remain brief and should return to object pushing as soon as safely possible.",
]

OBJECT_GREEN_TASK_CONSTRAINTS = [
    "The specification E should use push as the main progress action for moving objects toward the green area.",
    "When detect_object_green occurs, all movable objects are in the green area and the robot should stop or hold position.",
    "For object-pushing tasks, do not stop just because green_detected occurs; stop only after detect_object_green.",
    "Movable objects are not obstacles for sensing: if an object is ahead, push should remain available rather than switching to obstacle recovery.",
    "Obstacle recovery should remain brief and should return to object pushing as soon as safely possible.",
]

GREEN_TASK_CONSTRAINTS = [
    "The specification E should prioritize go_to_green when the robot is not in the green area.",
    "When green_detected occurs, the robot has reached the green area and should remain there or use only minimal safe correction.",
]

YELLOW_TASK_CONSTRAINTS = [
    "The specification E should prioritize escape_yellow immediately after yellow_detected.",
    "When yellow_detected occurs, the robot is inside the yellow danger area and should leave it before resuming normal movement.",
    "After escape_yellow, return to obstacle-aware movement and avoid re-entering the yellow area when possible.",
]

RANDOM_WALK_PROMPT_REPLACEMENTS = {
    "A forward or random_walk action may continue briefly before the next obstacle event is processed.": (
        "A forward motion action may continue briefly before the next obstacle event is processed."
    ),
}

UNSUPPORTED_GUIDANCE_KEYWORDS = (
    "marker_seen",
    "marker_lost",
    "marker_close",
    "move_to_marker",
    "marker_track",
    "marker_approach",
)


def merge_prompt_semantics(
    state_overrides: dict[str, str] | None,
    event_overrides: dict[str, str] | None,
) -> tuple[dict[str, str], dict[str, str]]:
    return (
        _merge_semantics(STATE_SEMANTICS, state_overrides),
        _merge_semantics(EVENT_SEMANTICS, event_overrides),
    )


def load_example_tasks() -> list[str]:
    return []


def build_pipeline_prompt(
    *,
    goal: str,
    user_task: str | None,
    controllable: list[str],
    uncontrollable: list[str],
    guidance: list[str],
    state_semantics: dict[str, str],
    event_semantics: dict[str, str],
    example_tasks: list[str],
) -> str:
    lines: list[str] = []
    lines.append(
        "We operate a DES-based supervisor for a basic grid simulator. Generate one plant "
        "automaton G and one specification automaton E."
    )
    lines.append("")
    lines.append("Controllable events: " + ", ".join(controllable))
    lines.append("Uncontrollable events: " + ", ".join(uncontrollable))
    lines.append("")
    if user_task:
        lines.append("User task: " + user_task)
        lines.append("")
    lines.append("Goal: " + goal)
    lines.append("")

    state_semantics_lines = [f"- {name}: {text}" for name, text in sorted(state_semantics.items())]
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

    if guidance:
        lines.append("Behaviour guidance:")
        lines.extend(f"- {item}" for item in guidance)
        lines.append("")

    if example_tasks:
        lines.append("Examples:")
        lines.extend(f"- {item}" for item in example_tasks)
        lines.append("")

    lines.append(
        "Return ONLY valid JSON with this exact top-level schema. Each automaton must include "
        "the event lists so XML conversion can preserve controllability. The "
        "controllable_events and uncontrollable_events fields must exactly repeat the "
        "provided event lists above, and every transition event must appear in one of those lists:"
    )
    lines.append(
        "{\n"
        '  "G": [\n'
        "    {\n"
        '      "rationale": "<short plant reasoning>",\n'
        '      "new_states": ["state_names"],\n'
        '      "controllable_events": ["..."],\n'
        '      "uncontrollable_events": ["..."],\n'
        '      "transitions": ["(\\"state\\", \\"event\\", \\"next\\")"]\n'
        "    }\n"
        "  ],\n"
        '  "E": [\n'
        "    {\n"
        '      "rationale": "<short spec reasoning>",\n'
        '      "new_states": ["state_names"],\n'
        '      "controllable_events": ["..."],\n'
        '      "uncontrollable_events": ["..."],\n'
        '      "transitions": ["(\\"state\\", \\"event\\", \\"next\\")"]\n'
        "    }\n"
        "  ]\n"
        "}"
    )
    return "\n".join(lines)


def _normalize_pipeline_payload(payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise SystemExit("LLM response must be a JSON object.")
    if not isinstance(payload.get("G"), list) or not isinstance(payload.get("E"), list):
        raise SystemExit("LLM response must contain list fields: G and E.")
    for section_name in ("G", "E"):
        for automaton in payload[section_name]:
            if not isinstance(automaton, dict):
                raise SystemExit(f"{section_name} entries must be JSON objects.")
            if not isinstance(automaton.get("transitions"), list):
                raise SystemExit(f"{section_name} entries must contain a transitions list.")
            _complete_event_lists(automaton)
    return payload


def _transition_event_names(transitions: list[object]) -> list[str]:
    event_names: list[str] = []
    for transition in transitions:
        if isinstance(transition, (list, tuple)) and len(transition) == 3:
            event_names.append(str(transition[1]))
            continue
        if isinstance(transition, str):
            match = re.search(r'\("[^"]+",\s*"([^"]+)",\s*"[^"]+"\)', transition.strip())
            if match:
                event_names.append(match.group(1))
    return event_names


def _complete_event_lists(automaton: dict) -> None:
    controllable = list(automaton.get("controllable_events") or [])
    uncontrollable = list(automaton.get("uncontrollable_events") or [])
    listed = set(controllable) | set(uncontrollable)
    missing = [event for event in _transition_event_names(automaton["transitions"]) if event not in listed]
    for event in missing:
        if event not in uncontrollable:
            uncontrollable.append(event)
    automaton["controllable_events"] = controllable
    automaton["uncontrollable_events"] = uncontrollable


def _write_automaton_xml(automaton: dict, output_path: Path) -> None:
    temp_json = output_path.with_suffix(".json.tmp")
    temp_json.write_text(json.dumps(automaton, indent=2), encoding="utf-8")
    try:
        json_to_nadzoru_xml.convert_json_to_xml(
            argparse.Namespace(
                json=str(temp_json),
                output=str(output_path),
                states=automaton.get("states") or [],
                initial=automaton.get("initial") or None,
                marked=automaton.get("marked") or None,
                controllable=automaton.get("controllable_events") or DEFAULT_CONTROLLABLE,
                uncontrollable=automaton.get("uncontrollable_events") or DEFAULT_UNCONTROLLABLE,
                supervisor_id=automaton.get("id") or output_path.stem,
            )
        )
    finally:
        temp_json.unlink(missing_ok=True)


def _without_random_walk_guidance(items: list[str]) -> list[str]:
    return [
        item
        for item in items
        if "random_walk" not in item
        and not any(keyword in item for keyword in UNSUPPORTED_GUIDANCE_KEYWORDS)
    ]


def _without_random_walk_prompt_text(prompt: str) -> str:
    for old, new in RANDOM_WALK_PROMPT_REPLACEMENTS.items():
        prompt = prompt.replace(old, new)
    return prompt


def is_red_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return "go_to_red" in task or "red_detected" in task or "red area" in task


def is_blue_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return "escape_blue" in task or "blue_detected" in task or "blue area" in task


def is_green_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return "go_to_green" in task or "green_detected" in task or "green area" in task


def is_yellow_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return "escape_yellow" in task or "yellow_detected" in task or "yellow area" in task


def is_object_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return (
        "push" in task
        or "detect_object_red" in task
        or "detect_object_green" in task
        or "object" in task
    )

class Tee:
    def __init__(self, *streams) -> None:
        self.streams = streams

    def write(self, text: str) -> int:
        for stream in self.streams:
            stream.write(text)
        return len(text)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


def resolve_repo_path(path: Path | str | None) -> Path | None:
    if path is None:
        return None
    path = Path(path).expanduser()
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def build_basic_prompt(user_task: str | None, include_examples: bool) -> str:
    red_task = is_red_task(user_task)
    blue_task = is_blue_task(user_task)
    green_task = is_green_task(user_task)
    yellow_task = is_yellow_task(user_task)
    object_task = is_object_task(user_task)
    state_semantics, event_semantics = merge_prompt_semantics(None, None)
    event_semantics.pop("random_walk", None)
    event_semantics["stop"] = (
        "hold position for one step without translating or rotating; use for deliberate waiting, "
        "goal holding, or when no safe movement is available."
    )
    controllable = list(DEFAULT_CONTROLLABLE)
    uncontrollable = list(DEFAULT_UNCONTROLLABLE)
    goal = DEFAULT_GOAL
    constraints = _without_random_walk_guidance(list(DEFAULT_CONSTRAINTS)) + BASIC_SIM_CONSTRAINTS

    if red_task:
        controllable += RED_CONTROLLABLE
        uncontrollable += RED_UNCONTROLLABLE
        goal = RED_GOAL
        if not object_task:
            constraints += RED_TASK_CONSTRAINTS
        event_semantics.update({
            "red_detected": "the agent is inside the 2x2 red gathering area.",
            "go_to_red": (
                "move one grid cell toward the nearest red-area cell; if already in "
                "the red area, remain in place."
            ),
        })

    if blue_task:
        controllable += BLUE_CONTROLLABLE
        uncontrollable += BLUE_UNCONTROLLABLE
        goal = BLUE_GOAL
        constraints += BLUE_TASK_CONSTRAINTS
        event_semantics.update({
            "blue_detected": "the agent is inside the 2x2 blue danger area.",
            "escape_blue": (
                "move one grid cell away from the blue area; use immediately after "
                "blue_detected to leave the danger area."
            ),
        })

    if red_task and blue_task:
        goal = RED_BLUE_GOAL

    if green_task:
        controllable += GREEN_CONTROLLABLE
        uncontrollable += GREEN_UNCONTROLLABLE
        goal = GREEN_GOAL
        if not object_task:
            constraints += GREEN_TASK_CONSTRAINTS
        event_semantics.update({
            "green_detected": "the agent is inside the 2x2 green gathering area.",
            "go_to_green": (
                "move one grid cell toward the nearest green-area cell; if already in "
                "the green area, remain in place."
            ),
        })

    if yellow_task:
        controllable += YELLOW_CONTROLLABLE
        uncontrollable += YELLOW_UNCONTROLLABLE
        goal = YELLOW_GOAL
        constraints += YELLOW_TASK_CONSTRAINTS
        event_semantics.update({
            "yellow_detected": "the agent is inside the 2x2 yellow danger area.",
            "escape_yellow": (
                "move one grid cell away from the yellow area; use immediately after "
                "yellow_detected to leave the danger area."
            ),
        })

    if green_task and yellow_task:
        goal = GREEN_YELLOW_GOAL

    if object_task:
        controllable += OBJECT_CONTROLLABLE
        if green_task:
            uncontrollable += OBJECT_GREEN_UNCONTROLLABLE
            goal = OBJECT_GREEN_GOAL
            constraints += OBJECT_GREEN_TASK_CONSTRAINTS
        else:
            uncontrollable += OBJECT_UNCONTROLLABLE
            goal = OBJECT_GOAL
            constraints += OBJECT_TASK_CONSTRAINTS
        event_semantics.update({
            "detect_object_red": "all four movable 1-cell objects are inside the 2x2 red area.",
            "detect_object_green": "all four movable 1-cell objects are inside the 2x2 green area.",
            "push": (
                "move toward a movable object; if an object is directly in front, push it "
                "one cell forward in the robot's heading and step into its previous cell; "
                "if the object cannot move forward, reposition around it instead of repeatedly pushing into a wall."
            ),
        })

    prompt = build_pipeline_prompt(
        goal=goal,
        user_task=user_task,
        controllable=controllable,
        uncontrollable=uncontrollable,
        guidance=constraints,
        state_semantics=state_semantics,
        event_semantics=event_semantics,
        example_tasks=load_example_tasks() if include_examples else [],
    )
    return _without_random_walk_prompt_text(prompt)


def write_outputs(payload: dict, output_dir: Path, user_task: str) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_dir = output_dir / "json"
    xml_dir = output_dir / "xml"
    json_dir.mkdir(parents=True, exist_ok=True)
    xml_dir.mkdir(parents=True, exist_ok=True)

    user_prompt_path = output_dir / "user_prompt.txt"
    combined_path = json_dir / "basic_sim_ge.json"
    user_prompt_path.write_text(user_task.strip() + "\n", encoding="utf-8")
    combined_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    g = payload["G"][0]
    e = payload["E"][0]
    g_json = json_dir / "G.json"
    e_json = json_dir / "E.json"
    g_xml = xml_dir / "G.xml"
    e_xml = xml_dir / "E.xml"

    g_json.write_text(json.dumps(g, indent=2), encoding="utf-8")
    e_json.write_text(json.dumps(e, indent=2), encoding="utf-8")
    _write_automaton_xml(g, g_xml)
    _write_automaton_xml(e, e_xml)

    print(f"Wrote {user_prompt_path}")
    print(f"Wrote {combined_path}")
    print(f"Wrote {g_json}")
    print(f"Wrote {e_json}")
    print(f"Wrote {g_xml}")
    print(f"Wrote {e_xml}")
    return g_xml, e_xml


def write_full_prompt(output_dir: Path, prompt: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    prompt_path = output_dir / "full_prompt.txt"
    prompt_path.write_text(prompt, encoding="utf-8")
    print(f"Wrote {prompt_path}")
    return prompt_path


def call_llm_for_payload(args: argparse.Namespace, prompt: str) -> dict:
    api_key = read_api_key()
    result = call_chat_completion(
        api_key,
        prompt,
        args.model,
        args.temperature,
        args.timeout,
        args.retries,
    )
    payload = _normalize_pipeline_payload(result)
    if len(payload["G"]) != 1 or len(payload["E"]) != 1:
        raise SystemExit("Expected exactly one G and one E automaton from the LLM.")
    return payload


def synthesize_outputs(
    args: argparse.Namespace,
    g_xml: Path,
    e_xml: Path,
    synthesized_dir: Path,
) -> Path:
    print()
    print("Synthesizing Sloc...")
    synthesize_sloc(
        argparse.Namespace(
            g=g_xml,
            e=e_xml,
            output_dir=synthesized_dir,
            nadzoru_root=args.nadzoru_root,
        )
    )
    return synthesized_dir / "Sloc.xml"


def write_sct_yaml(sloc_xml: Path, yaml_path: Path, nadzoru_root: Path) -> Path:
    automaton_cls = import_nadzoru(nadzoru_root)
    supervisor = automaton_cls()
    supervisor.load(str(sloc_xml))
    yaml_payload = _build_sct_yaml([supervisor])
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(yaml.safe_dump(yaml_payload, sort_keys=False), encoding="utf-8")
    print(f"Wrote {yaml_path}")
    warn_for_real_robot_events(yaml_payload, yaml_path)
    return yaml_path


def warn_for_real_robot_events(yaml_payload: dict, yaml_path: Path) -> None:
    supported_controllables = {
        "EV_move_forward",
        "EV_move_backward",
        "EV_rotate_clockwise",
        "EV_rotate_counterclockwise",
        "EV_full_rotate",
        "EV_random_walk",
        "EV_go_to_red",
        "EV_escape_blue",
        "EV_go_to_green",
        "EV_escape_yellow",
        "EV_push",
    }
    supported_uncontrollables = {
        "EV_path_clear",
        "EV_obstacle_front",
        "EV_obstacle_left",
        "EV_obstacle_right",
        "EV_red_detected",
        "EV_blue_detected",
        "EV_green_detected",
        "EV_yellow_detected",
        "EV_detect_object_green",
    }
    events = list(yaml_payload["events"])
    controllability = list(yaml_payload["ev_controllable"])
    unsupported = []
    for event, flag in zip(events, controllability):
        supported = supported_controllables if int(flag) else supported_uncontrollables
        if event not in supported:
            unsupported.append(event)

    if unsupported:
        print(
            "Warning: "
            f"{yaml_path} contains events the current real-robot supervisor does not execute/sense: "
            f"{', '.join(sorted(unsupported))}."
        )
        print(
            "Use prompts limited to obstacle/path-clear exploration events, or extend "
            "leo_real.robot_supervisor_rgb before running this YAML on robots."
        )


def load_user_prompts(path: Path) -> list[str]:
    if not path.exists():
        raise SystemExit(f"User prompts file not found: {path}")
    chunks = [chunk.strip() for chunk in path.read_text(encoding="utf-8").split("\n\n")]
    prompts = [chunk for chunk in chunks if chunk]
    if not prompts:
        raise SystemExit(f"No prompts found in {path}")
    return prompts


def resolve_user_tasks(args: argparse.Namespace) -> list[tuple[str, str]]:
    if args.user_task:
        return [("custom", args.user_task.strip())]

    prompts = load_user_prompts(args.user_prompts_file)
    if args.all_user_prompts:
        return [(f"prompt_{idx}", prompt) for idx, prompt in enumerate(prompts, start=1)]

    if args.user_prompt_index < 1 or args.user_prompt_index > len(prompts):
        raise SystemExit(
            f"--user-prompt-index must be between 1 and {len(prompts)} "
            f"for {args.user_prompts_file}"
        )
    return [(f"prompt_{args.user_prompt_index}", prompts[args.user_prompt_index - 1])]


def safe_label(value: str) -> str:
    label = re.sub(r"[^a-zA-Z0-9_.-]+", "_", value.strip())
    return label.strip("_") or "run"


def config_yaml_path_for_run(output_dir: Path) -> Path:
    try:
        relative = output_dir.resolve().relative_to(DEFAULT_OUTPUT_DIR.resolve())
        stem = safe_label("_".join(relative.parts))
    except ValueError:
        stem = safe_label(f"{output_dir.parent.name}_{output_dir.name}")
    return DEFAULT_REAL_YAML_OUT_DIR / f"{stem}.yaml"


def next_run_dir(parent: Path) -> Path:
    parent.mkdir(parents=True, exist_ok=True)
    run_numbers = []
    for child in parent.iterdir():
        match = re.fullmatch(r"run_(\d+)", child.name)
        if child.is_dir() and match:
            run_numbers.append(int(match.group(1)))

    next_number = max(run_numbers, default=0) + 1
    while True:
        candidate = parent / f"run_{next_number:03d}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            next_number += 1


def run_one(args: argparse.Namespace, label: str, user_task: str, output_dir: Path) -> None:
    prompt = build_basic_prompt(user_task, args.include_examples)
    if args.print_prompt:
        print(f"----- BEGIN BASIC_SIM LLM PROMPT: {label} -----")
        print(prompt)
        print(f"----- END BASIC_SIM LLM PROMPT: {label} -----")
        if args.dry_run:
            return

    write_full_prompt(output_dir, prompt)
    payload = call_llm_for_payload(args, prompt)

    g_xml, e_xml = write_outputs(payload, output_dir, user_task)

    synthesized_dir = output_dir / "synthesized"
    sloc_xml = synthesized_dir / "Sloc.xml"
    if not args.skip_synthesis:
        sloc_xml = synthesize_outputs(args, g_xml, e_xml, synthesized_dir)

    current_yaml = None
    if not args.skip_synthesis:
        current_yaml = write_sct_yaml(sloc_xml, sloc_xml.with_suffix(".yaml"), args.nadzoru_root)
        if args.save_config_yaml:
            config_run_yaml = write_sct_yaml(
                sloc_xml,
                config_yaml_path_for_run(output_dir),
                args.nadzoru_root,
            )
            current_yaml = config_run_yaml
        if args.robot_config_output:
            robot_config_yaml = write_sct_yaml(sloc_xml, args.robot_config_output, args.nadzoru_root)
            current_yaml = robot_config_yaml

    print()
    print("Next step:")
    print(
        "  python3 basic_sim/grid_sim.py "
        f"--automata {sloc_xml} --agents 5"
    )
    if current_yaml is not None:
        print()
        print("Real robot YAML:")
        print(f"  {current_yaml}")
        print("Use it with the ROS launch argument:")
        print(
            "  ros2 launch leo_real leo_real.launch.py "
            f"supervisor_yaml_path:={current_yaml}"
        )


def run_one_with_logging(
    args: argparse.Namespace,
    label: str,
    user_task: str,
    output_dir: Path,
) -> None:
    if args.dry_run:
        run_one(args, label, user_task, output_dir)
        return

    log_path = output_dir / "generation.log"
    with log_path.open("w", encoding="utf-8") as log_file:
        stdout_tee = Tee(sys.stdout, log_file)
        stderr_tee = Tee(sys.stderr, log_file)
        with contextlib.redirect_stdout(stdout_tee), contextlib.redirect_stderr(stderr_tee):
            print(f"Writing generation outputs to {output_dir}")
            print(f"Writing generation log to {log_path}")
            run_one(args, label, user_task, output_dir)


def run(args: argparse.Namespace) -> int:
    args.user_prompts_file = resolve_repo_path(args.user_prompts_file)
    args.output_dir = resolve_repo_path(args.output_dir)
    args.robot_config_output = resolve_repo_path(args.robot_config_output)
    args.nadzoru_root = Path(args.nadzoru_root).expanduser()
    tasks = resolve_user_tasks(args)
    for label, user_task in tasks:
        output_parent = args.output_dir / safe_label(label)
        output_dir = output_parent if args.dry_run else next_run_dir(output_parent)
        run_one_with_logging(args, label, user_task, output_dir)
    return 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate basic_sim G/E JSON and XML files with an LLM."
    )
    parser.add_argument("--user-task", help="Use one explicit prompt instead of user_prompts.txt.")
    parser.add_argument(
        "--user-prompts-file",
        type=Path,
        default=DEFAULT_USER_PROMPTS_FILE,
        help="Prompt file split by blank lines.",
    )
    parser.add_argument(
        "--user-prompt-index",
        type=int,
        default=1,
        help="1-based prompt index from --user-prompts-file.",
    )
    parser.add_argument(
        "--all-user-prompts",
        action="store_true",
        help="Generate outputs for every prompt in --user-prompts-file.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES)
    parser.add_argument("--print-prompt", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--skip-synthesis",
        action="store_true",
        help="Only write G/E JSON/XML files; do not generate synthesized/Sloc.xml or Sloc.yaml.",
    )
    parser.add_argument(
        "--robot-config-output",
        type=Path,
        default=DEFAULT_ROBOT_CONFIG_OUTPUT,
        help=(
            "Also write the synthesized SCT YAML to this path. Relative paths are "
            f"resolved from {REPO_ROOT}. Defaults to {DEFAULT_ROBOT_CONFIG_OUTPUT}."
        ),
    )
    parser.add_argument(
        "--no-save-config-yaml",
        dest="save_config_yaml",
        action="store_false",
        default=DEFAULT_SAVE_CONFIG_YAML,
        help=(
            "Do not save a per-run YAML copy in leo_real/config. By default the "
            "script writes both synthesized/Sloc.yaml and leo_real/config/<prompt>_<run>.yaml."
        ),
    )
    parser.add_argument("--nadzoru-root", type=Path, default=DEFAULT_NADZORU_ROOT)
    parser.add_argument(
        "--include-examples",
        action="store_true",
        help="Include the longer example task library in the LLM prompt.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    return run(make_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
