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
DEFAULT_USER_PROMPTS_FILE = "/home/ecem/ros2_ws/src/Leo_sct/basic_sim/user_prompts.txt"
if str(BASIC_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_SIM_DIR))
if str(LLM_DIR) not in sys.path:
    sys.path.insert(0, str(LLM_DIR))

from pipeline_prompts import (  # noqa: E402
    DEFAULT_CONSTRAINTS,
    SYSTEM_PROMPT,
    build_pipeline_prompt,
    load_example_tasks,
    merge_prompt_semantics,
)
from run_pipeline import (  # noqa: E402
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT_S,
    _normalize_pipeline_payload,
    _write_automaton_xml,
    call_chat_completion,
    read_api_key,
)
from synthesize_sloc import DEFAULT_NADZORU_ROOT, synthesize as synthesize_sloc  # noqa: E402
from automata import build_sct_yaml, load_nadzoru_xml  # noqa: E402


DEFAULT_CONTROLLABLE = [
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    "stop",
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

RANDOM_WALK_PROMPT_REPLACEMENTS = {
    "A forward or random_walk action may continue briefly before the next obstacle event is processed.": (
        "A forward motion action may continue briefly before the next obstacle event is processed."
    ),
}


def _without_random_walk_guidance(items: list[str]) -> list[str]:
    return [item for item in items if "random_walk" not in item]


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


def is_object_task(user_task: str | None) -> bool:
    if not user_task:
        return False
    task = user_task.lower()
    return "push" in task or "detect_object_red" in task or "object" in task

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


def build_basic_prompt(user_task: str | None, include_examples: bool) -> str:
    red_task = is_red_task(user_task)
    blue_task = is_blue_task(user_task)
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

    if object_task:
        controllable += OBJECT_CONTROLLABLE
        uncontrollable += OBJECT_UNCONTROLLABLE
        goal = OBJECT_GOAL
        constraints += OBJECT_TASK_CONSTRAINTS
        event_semantics.update({
            "detect_object_red": "all four movable 1-cell objects are inside the 2x2 red area.",
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
        api_key=api_key,
        prompt=prompt,
        model=args.model,
        temperature=args.temperature,
        timeout_s=args.timeout,
        retries=args.retries,
        cooldown_min_s=args.cooldown_min,
        cooldown_max_s=args.cooldown_max,
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


def write_sct_yaml(sloc_xml: Path, yaml_path: Path) -> Path:
    supervisor = load_nadzoru_xml(sloc_xml)
    yaml_payload = build_sct_yaml(supervisor)
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
        "EV_stop",
        "EV_random_walk",
        "EV_move_to_marker",
    }
    supported_uncontrollables = {
        "EV_path_clear",
        "EV_obstacle_front",
        "EV_obstacle_left",
        "EV_obstacle_right",
        "EV_marker_seen",
        "EV_marker_lost",
        "EV_marker_close",
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
            "swarm_basics.robot_supervisor before running this YAML on robots."
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
        current_yaml = write_sct_yaml(sloc_xml, sloc_xml.with_suffix(".yaml"))
        if args.robot_config_output:
            robot_config_yaml = write_sct_yaml(sloc_xml, args.robot_config_output)
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
            "  ros2 launch swarm_basics spawn_multi_robots.launch.py "
            f"metadata_yaml_path:={current_yaml}"
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
        default=Path(__file__).with_name("llm_generated"),
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--cooldown-min", type=float, default=0.0)
    parser.add_argument("--cooldown-max", type=float, default=0.0)
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
        help=(
            "Also write the synthesized SCT YAML to this path, for example "
            "swarm_basics/config/my_generated_controller.yaml."
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
