#!/usr/bin/env python3
"""Generate separate G and E JSON/XML files for basic_sim using the LLM prompt flow."""

from __future__ import annotations

import argparse
import contextlib
import json
import random
import re
import sys
from collections import Counter
from pathlib import Path


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


def build_feedback_prompt(base_prompt: str, previous_payload: dict, simulation_output: str) -> str:
    previous_json = json.dumps(previous_payload, indent=2)
    return (
        f"{base_prompt}\n\n"
        "Simulation feedback loop:\n"
        "You already returned the JSON below. It was synthesized and run in the grid simulator.\n"
        "Use the simulator output to improve the automata. Return a complete replacement JSON object "
        "with exactly the same schema: one plant in G and one specification in E.\n\n"
        "Focus on behavioral fixes, not cosmetic renaming:\n"
        "- First satisfy the user's task objective; collision count is only one diagnostic.\n"
        "- Use the task-specific feedback section to fix missing path patterns or goal progress.\n"
        "- If collisions are high, make the specification recover earlier from obstacle events.\n"
        "- If stop dominates before the task is complete, make progress actions reachable sooner.\n"
        "- If a square task does not complete square-like cycles, use explicit side/corner states "
        "that alternate forward segments with one consistent 90-degree turn direction.\n"
        "- If a red-area task does not reach or hold the red area, prioritize go_to_red until red_detected.\n"
        "- If a blue-area task enters or stays in the blue area, prioritize escape_blue after blue_detected.\n"
        "- If an object task does not move objects into red, prioritize push until detect_object_red.\n\n"
        "Previous JSON:\n"
        f"{previous_json}\n\n"
        "Grid simulator output:\n"
        f"{simulation_output.strip()}\n"
    )


def run_feedback_simulation(
    sloc_path: Path,
    args: argparse.Namespace,
    user_task: str,
    output_dir: Path,
) -> str:
    import grid_sim  # noqa: PLC0415
    from grid_world import GridEnvironment  # noqa: PLC0415

    sim_args = argparse.Namespace(
        automata=[sloc_path],
        agents=args.feedback_agents,
        steps=args.feedback_steps,
        width=args.feedback_width,
        height=args.feedback_height,
        seed=args.feedback_seed,
        heading=args.feedback_heading,
        start=None,
        obstacle=None,
        object=None,
        red_area_origin=None,
        blue_area_origin=None,
        side_sensors=args.feedback_side_sensors,
        text=True,
        save_plot=None,
        quiet=True,
        log_file=None,
        log_dir=output_dir / "sim_logs",
        no_log=True,
    )

    rng = random.Random(sim_args.seed)
    random.seed(sim_args.seed)
    red_enabled = grid_sim.should_enable_red_area(sim_args)
    blue_enabled = grid_sim.should_enable_blue_area(sim_args)
    if (
        red_enabled
        and blue_enabled
        and sim_args.red_area_origin is None
        and sim_args.blue_area_origin is None
    ):
        red_area, blue_area = grid_sim.default_separate_red_blue_areas(
            sim_args.width,
            sim_args.height,
        )
    else:
        red_area = set()
        blue_area = set()

    if sim_args.red_area_origin is not None:
        red_area = grid_sim.build_red_area(sim_args.red_area_origin, sim_args.width, sim_args.height)
    elif red_enabled and not red_area:
        red_area = grid_sim.default_red_area(sim_args.width, sim_args.height)

    if sim_args.blue_area_origin is not None:
        blue_area = grid_sim.build_blue_area(sim_args.blue_area_origin, sim_args.width, sim_args.height)
    elif blue_enabled and not blue_area:
        blue_area = grid_sim.default_blue_area(sim_args.width, sim_args.height)

    obstacles = set(sim_args.obstacle or [])
    if sim_args.object:
        objects = set(sim_args.object)
    elif grid_sim.should_enable_objects(sim_args):
        objects = grid_sim.default_objects(
            sim_args.width,
            sim_args.height,
            obstacles | red_area | blue_area,
            rng,
        )
    else:
        objects = set()

    env = GridEnvironment(sim_args.width, sim_args.height, obstacles, red_area, blue_area, objects)
    agents = grid_sim.build_agents(sim_args, env)
    reports_by_agent: dict[str, list[dict]] = {agent.name: [] for agent in agents}

    for _ in range(1, sim_args.steps + 1):
        reports = [agent.step(env, agents, rng) for agent in agents]
        for report in reports:
            reports_by_agent[report["agent"]].append(report)

    simulation_output = build_feedback_report(
        user_task=user_task,
        args=sim_args,
        env=env,
        agents=agents,
        reports_by_agent=reports_by_agent,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    feedback_path = output_dir / "simulation_feedback.txt"
    feedback_path.write_text(simulation_output, encoding="utf-8")
    print(f"Wrote {feedback_path}")
    return simulation_output


def build_feedback_report(
    user_task: str,
    args: argparse.Namespace,
    env,
    agents: list,
    reports_by_agent: dict[str, list[dict]],
) -> str:
    total_cells = env.width * env.height - len(env.obstacles)
    visited = set().union(*(agent.visited for agent in agents))
    total_collisions = sum(agent.collisions for agent in agents)
    action_counts = Counter()
    for agent in agents:
        action_counts.update(agent.action_counts)

    lines = [
        f"Feedback simulation: {args.agents} agents, {args.steps} steps, grid={args.width}x{args.height}, seed={args.seed}.",
        f"Visited {len(visited)}/{total_cells} free cells ({len(visited) / total_cells:.1%}).",
        f"Collisions: {total_collisions} total.",
        f"Actions overall: {format_counts(action_counts)}.",
    ]

    for agent in agents:
        lines.append(
            f"{agent.name}: final position={agent.position}, heading={agent.heading}, "
            f"collisions={agent.collisions}, actions={format_counts(Counter(agent.action_counts))}."
        )

    task_feedback = build_task_specific_feedback(user_task, env, agents, reports_by_agent)
    if task_feedback:
        lines.append("")
        lines.append("Task-specific feedback:")
        lines.extend(task_feedback)

    return "\n".join(lines) + "\n"


def build_task_specific_feedback(
    user_task: str,
    env,
    agents: list,
    reports_by_agent: dict[str, list[dict]],
) -> list[str]:
    task = user_task.lower()
    if "square path" in task or "four square sides" in task:
        return square_task_feedback(reports_by_agent)
    if "zigzag path" in task:
        return zigzag_task_feedback(reports_by_agent)
    if is_red_task(user_task) and not is_object_task(user_task):
        reached = sum(1 for agent in agents if agent.position in env.red_area)
        return [f"Red goal: {reached}/{len(agents)} agents ended inside the red area."]
    if is_blue_task(user_task):
        inside = sum(1 for agent in agents if agent.position in env.blue_area)
        return [f"Blue avoidance: {inside}/{len(agents)} agents ended inside the blue area."]
    if is_object_task(user_task):
        in_red = sum(1 for obj in env.objects if obj in env.red_area)
        return [f"Object goal: {in_red}/{len(env.objects)} objects ended inside the red area."]
    return []


def square_task_feedback(reports_by_agent: dict[str, list[dict]]) -> list[str]:
    lines = []
    for agent_name, reports in reports_by_agent.items():
        actions = [report["action"] for report in reports if report["action"] not in (None, "none")]
        forward_count = actions.count("move_forward")
        clockwise = actions.count("rotate_clockwise")
        counterclockwise = actions.count("rotate_counterclockwise")
        dominant_turn = max(clockwise, counterclockwise)
        opposite_turn = min(clockwise, counterclockwise)
        corner_turns = clockwise + counterclockwise
        forward_then_turn = sum(
            1
            for prev_action, action in zip(actions, actions[1:])
            if prev_action == "move_forward"
            and action in {"rotate_clockwise", "rotate_counterclockwise"}
        )
        completed_square_cycles = min(forward_count // 4, dominant_turn // 4)
        consistent_turns = corner_turns > 0 and opposite_turn == 0
        status = "OK" if completed_square_cycles >= 1 and consistent_turns else "needs revision"
        lines.append(
            f"Square path {agent_name}: {status}; move_forward={forward_count}, "
            f"corner_turns={corner_turns}, consistent_one_direction_turns={consistent_turns}, "
            f"forward_then_turn_pairs={forward_then_turn}, estimated_completed_square_cycles={completed_square_cycles}."
        )
    lines.append(
        "Expected square behavior: repeated forward side segments followed by one consistent 90-degree turn direction, "
        "with explicit side/corner states so the sequence returns to side 1 after side 4."
    )
    return lines


def zigzag_task_feedback(reports_by_agent: dict[str, list[dict]]) -> list[str]:
    lines = []
    for agent_name, reports in reports_by_agent.items():
        actions = [report["action"] for report in reports if report["action"] not in (None, "none")]
        alternating_turns = 0
        last_turn = None
        for action in actions:
            if action not in {"rotate_clockwise", "rotate_counterclockwise"}:
                continue
            if last_turn is not None and action != last_turn:
                alternating_turns += 1
            last_turn = action
        lines.append(
            f"Zigzag path {agent_name}: move_forward={actions.count('move_forward')}, "
            f"clockwise={actions.count('rotate_clockwise')}, "
            f"counterclockwise={actions.count('rotate_counterclockwise')}, "
            f"alternating_turn_pairs={alternating_turns}."
        )
    return lines


def format_counts(counts: Counter) -> str:
    total = sum(counts.values())
    if total == 0:
        return "no actions"
    return ", ".join(
        f"{name}={count / total:.1%} ({count})"
        for name, count in sorted(counts.items())
    )


def feedback_needs_revision(simulation_output: str) -> bool:
    if "needs revision" in simulation_output:
        return True

    red_match = re.search(r"Red goal: (\d+)/(\d+) agents ended inside the red area", simulation_output)
    if red_match and red_match.group(1) != red_match.group(2):
        return True

    blue_match = re.search(r"Blue avoidance: (\d+)/(\d+) agents ended inside the blue area", simulation_output)
    if blue_match and int(blue_match.group(1)) > 0:
        return True

    object_match = re.search(r"Object goal: (\d+)/(\d+) objects ended inside the red area", simulation_output)
    if object_match and object_match.group(1) != object_match.group(2):
        return True

    coverage_match = re.search(r"Visited \d+/\d+ free cells \((\d+\.\d+)%\)", simulation_output)
    if coverage_match and float(coverage_match.group(1)) < 20.0:
        return True

    none_match = re.search(r"\bnone=(\d+\.\d+)%", simulation_output)
    if none_match and float(none_match.group(1)) > 50.0:
        return True

    return False


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

    if args.feedback_iterations and args.skip_synthesis:
        raise SystemExit("--feedback-iterations requires synthesis; remove --skip-synthesis.")

    current_payload = payload
    current_sloc = sloc_xml
    before_feedback_output = None
    if args.feedback_iterations:
        before_feedback_dir = output_dir / "before_feedback_simulation"
        print()
        print("Running before-feedback simulation...")
        before_feedback_output = run_feedback_simulation(
            current_sloc,
            args,
            user_task,
            before_feedback_dir,
        )

    for feedback_idx in range(1, args.feedback_iterations + 1):
        if before_feedback_output is not None:
            simulation_output = before_feedback_output
            before_feedback_output = None
        else:
            feedback_dir_for_sim = output_dir / f"feedback_round_{feedback_idx:03d}"
            print()
            print(f"Running feedback simulation round {feedback_idx}...")
            simulation_output = run_feedback_simulation(
                current_sloc,
                args,
                user_task,
                feedback_dir_for_sim,
            )

        if not feedback_needs_revision(simulation_output):
            print("Feedback simulation passed task checks; stopping feedback early.")
            break

        feedback_dir = output_dir / f"feedback_round_{feedback_idx:03d}"
        feedback_dir.mkdir(parents=True, exist_ok=True)
        feedback_prompt = build_feedback_prompt(prompt, current_payload, simulation_output)
        write_full_prompt(feedback_dir, feedback_prompt)
        current_payload = call_llm_for_payload(args, feedback_prompt)
        round_g_xml, round_e_xml = write_outputs(current_payload, feedback_dir, user_task)
        current_sloc = synthesize_outputs(
            args,
            round_g_xml,
            round_e_xml,
            feedback_dir / "synthesized",
        )

    if args.feedback_iterations:
        final_feedback_dir = output_dir / "final_feedback_simulation"
        final_feedback_dir.mkdir(parents=True, exist_ok=True)
        print()
        print("Running final feedback simulation...")
        run_feedback_simulation(current_sloc, args, user_task, final_feedback_dir)

    print()
    print("Next step:")
    print(
        "  python3 basic_sim/grid_sim.py "
        f"--automata {current_sloc} --agents 5"
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
    if args.feedback_iterations < 0:
        raise SystemExit("--feedback-iterations must be non-negative.")
    if args.feedback_steps < 1:
        raise SystemExit("--feedback-steps must be at least 1.")
    if args.feedback_agents < 1:
        raise SystemExit("--feedback-agents must be at least 1.")
    if args.feedback_width < 1 or args.feedback_height < 1:
        raise SystemExit("--feedback-width and --feedback-height must be at least 1.")

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
        help="Only write G/E JSON/XML files; do not generate synthesized/Sloc.xml.",
    )
    parser.add_argument("--nadzoru-root", type=Path, default=DEFAULT_NADZORU_ROOT)
    parser.add_argument(
        "--include-examples",
        action="store_true",
        help="Include the longer example task library in the LLM prompt.",
    )
    parser.add_argument(
        "--feedback-iterations",
        type=int,
        default=0,
        help=(
            "After the first LLM answer, synthesize and simulate Sloc.xml this many times, "
            "feeding each simulator summary back to the LLM for revision."
        ),
    )
    parser.add_argument(
        "--feedback-steps",
        type=int,
        default=50,
        help="Grid simulation steps per feedback round.",
    )
    parser.add_argument(
        "--feedback-agents",
        type=int,
        default=5,
        help="Number of agents to use in feedback simulations.",
    )
    parser.add_argument("--feedback-width", type=int, default=8)
    parser.add_argument("--feedback-height", type=int, default=6)
    parser.add_argument("--feedback-seed", type=int, default=7)
    parser.add_argument(
        "--feedback-heading",
        choices=["N", "E", "S", "W"],
        default="E",
        help="Initial heading for feedback simulation agents.",
    )
    parser.add_argument(
        "--feedback-side-sensors",
        action="store_true",
        help="Use obstacle_left/right perception events during feedback simulations.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    return run(make_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
