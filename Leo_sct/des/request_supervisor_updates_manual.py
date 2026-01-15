#!/usr/bin/env python3
"""
Helper script to ask the GPT API for improved DES supervisors without reading
robot_navigation.cpp.  Supply the states, events, and any existing transitions
via CLI flags or external text files, and the script will assemble the prompt
and return JSON with suggested transitions (same schema as the original tool).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import requests

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_KEY_FILE = ROOT / "api_key.txt"
DEFAULT_MODEL = "gpt-4.1"

DEFAULT_STATES = ["clear", "obs_left", "obs_right", "obs_front"]
DEFAULT_CONTROLLABLE_EVENTS = [
    "move_forward",
    "move_backward",
    "rotate_clockwise",
    "rotate_counterclockwise",
    "full_rotate",
    # "random_walk",
]
DEFAULT_UNCONTROLLABLE_EVENTS = [
    "path_clear",
    "obstacle_front",
    "obstacle_left",
    "obstacle_right",
]
DEFAULT_CONSTRAINTS = [
    "Every state must have an outgoing arrow for all uncontrollable events.",
    "Never allow moving forward or random walk when there is an obstacle.",
]


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


def parse_transition_line(line: str) -> Tuple[str, str, str]:
    line = line.strip()
    if not line:
        raise ValueError("Empty transition line.")
    match = re.search(r'("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)', line)
    if match:
        return match.group(1), match.group(2), match.group(3)
    csv = [token.strip() for token in re.split(r"[,\s]+", line) if token.strip()]
    if len(csv) == 3:
        return csv[0], csv[1], csv[2]
    raise ValueError(
        "Transition line must be either (...) or "
        '"state event next" with three tokens.'
    )


def load_transitions(lines: Iterable[str]) -> Dict[str, Dict[str, str]]:
    transitions: Dict[str, Dict[str, str]] = {}
    for line in lines:
        state, event, nxt = parse_transition_line(line)
        transitions.setdefault(state, {})[event] = nxt
    return transitions


def extend_unique(base: List[str], additions: Iterable[str]) -> List[str]:
    seen = set(base)
    for item in additions:
        if item not in seen:
            base.append(item)
            seen.add(item)
    return base


def collect_states_from_transitions(transitions: Dict[str, Dict[str, str]]) -> List[str]:
    states: List[str] = []
    seen = set()
    for state, edges in transitions.items():
        if state not in seen:
            states.append(state)
            seen.add(state)
        for nxt in edges.values():
            if nxt not in seen:
                states.append(nxt)
                seen.add(nxt)
    return states


def build_prompt(
    goal: str,
    states: List[str],
    controllable: List[str],
    uncontrollable: List[str],
    transitions: Dict[str, Dict[str, str]],
    include_existing: bool,
    scenario: str | None,
    guidance: List[str] | None,
) -> str:
    states = sorted(states)
    lines: List[str] = []
    lines.append(
        "We operate a DES-based supervisor for a mobile robot with multiple "
        "sensing modes. Each entry describes a sensor switch "
        "(uncontrollable event) or an allowed control action."
    )
    lines.append("")
    lines.append("Controllable events: " + ", ".join(controllable))
    lines.append("Uncontrollable events: " + ", ".join(uncontrollable))
    lines.append("Current states: " + ", ".join(states))
    lines.append("")
    if scenario:
        lines.append("Scenario context: " + scenario)
        lines.append("")
    if guidance:
        lines.append("Behaviour guidance:")
        for item in guidance:
            lines.append(f"- {item}")
        lines.append("")
    if include_existing and transitions:
        lines.append("Existing transitions:")
        for state in states:
            if state not in transitions:
                continue
            lines.append(f"- {state}:")
            for event, nxt in sorted(transitions[state].items()):
                lines.append(f"    {event} -> {nxt}")
        lines.append("")
    else:
        lines.append(
            "Design a new transition structure from scratch using the states "
            "above. You may propose additional helper states if justified."
        )
        lines.append("")
    lines.append(
        f"Goal: {goal}. The supervisor should encourage systematic area "
        "coverage, avoid oscillations, and ensure safe reactions to obstacles."
    )
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
        '  "coverage_strategy": ["bullet list of coverage ideas"]\n'
        "}"
    )
    lines.append(
        "Transitions must only use the listed events unless you introduce a "
        "clearly documented new event."
    )
    return "\n".join(lines)


def call_chat_completion(
    api_key: str,
    prompt: str,
    model: str,
    temperature: float,
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
    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        data=json.dumps(payload),
        timeout=60,
    )
    response.raise_for_status()
    content = response.json()["choices"][0]["message"]["content"]
    return parse_json_response(content)


def parse_json_response(content: str) -> Dict[str, str]:
    def strip_line_comments(text: str) -> str:
        # Remove // comments while respecting string literals.
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
                # Skip to end of line.
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


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Generate DES supervisor transitions via GPT (no C++ parsing)."
    )
    parser.add_argument(
        "--states",
        nargs="+",
        help="List of known DES states (default: clear obs_left obs_right obs_front).",
    )
    parser.add_argument(
        "--controllable-events",
        nargs="+",
        help="List of controllable event names (defaults to robot base actions).",
    )
    parser.add_argument(
        "--uncontrollable-events",
        nargs="+",
        help="List of uncontrollable event names (defaults to obstacle cues).",
    )
    parser.add_argument(
        "--existing-transition",
        action="append",
        default=[],
        help=(
            "Existing transition line, e.g. "
            '"(\\"clear\\", \\"path_clear\\", \\"clear\\")" '
            'or "clear path_clear clear".  Repeat for multiple lines.'
        ),
    )
    parser.add_argument(
        "--transitions-file",
        action="append",
        type=Path,
        help="Optional file(s) containing transition lines (one per line).",
    )
    parser.add_argument(
        "--goal",
        default="Cover the entire arena efficiently while staying safe",
        help="High level objective passed to GPT.",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help="Chat Completions model identifier.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.55,
        help="Sampling temperature for the API call.",
    )
    parser.add_argument(
        "--scenario",
        help="Optional environment description (crowded hallway, tunnel, etc.).",
    )
    parser.add_argument(
        "--constraint",
        action="append",
        help="Behaviour constraints / guidance (repeat flag for multiple entries).",
    )
    parser.add_argument(
        "--no-default-constraints",
        action="store_true",
        help="Do not include the baked-in obstacle-handling guidance strings.",
    )
    parser.add_argument(
        "--no-current",
        action="store_true",
        help="Do not include existing transitions in the prompt.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the prompt instead of calling the API.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to write the raw JSON reply.",
    )
    args = parser.parse_args(argv)

    transitions_input: List[str] = []
    if args.transitions_file:
        for path in args.transitions_file:
            transitions_input.extend(path.read_text(encoding="utf-8").splitlines())
    transitions_input.extend(args.existing_transition)

    transitions: Dict[str, Dict[str, str]] = {}
    if transitions_input and not args.no_current:
        transitions = load_transitions(transitions_input)

    states = list(args.states) if args.states else list(DEFAULT_STATES)
    inferred_states = collect_states_from_transitions(transitions)
    if inferred_states:
        extend_unique(states, inferred_states)
    if not states:
        raise SystemExit("At least one state must be provided (via --states or transitions).")

    controllable = args.controllable_events or DEFAULT_CONTROLLABLE_EVENTS
    uncontrollable = args.uncontrollable_events or DEFAULT_UNCONTROLLABLE_EVENTS

    guidance_lines: List[str] = []
    if not args.no_default_constraints:
        guidance_lines.extend(DEFAULT_CONSTRAINTS)
    if args.constraint:
        guidance_lines.extend(args.constraint)

    prompt = build_prompt(
        goal=args.goal,
        states=states,
        controllable=controllable,
        uncontrollable=uncontrollable,
        transitions=transitions,
        include_existing=not args.no_current,
        scenario=args.scenario,
        guidance=guidance_lines or None,
    )

    if args.dry_run:
        print(prompt)
        return 0

    api_key = read_api_key()
    result = call_chat_completion(api_key, prompt, args.model, args.temperature)
    print(json.dumps(result, indent=2))
    transitions_out = result.get("transitions")
    if transitions_out:
        print("\nSuggested lines:\n")
        for line in transitions_out:
            print(line)
    if args.save:
        args.save.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
