#!/usr/bin/env python3
"""
Helper script to ask the GPT API for improved DES supervisors.

The script scrapes the controllable/uncontrollable events and the existing
SetTransition(...) rules from robot_navigation.cpp, builds a prompt that
describes the current behaviour and desired coverage goal, and sends it to
the OpenAI Chat Completions endpoint.  The response is expected to be JSON
with new transitions that can be pasted back into robot_navigation.cpp.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Dict, List

import requests

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CPP = Path(__file__).resolve().with_name("robot_navigation.cpp")
DEFAULT_KEY_FILE = ROOT / "api_key.txt"
DEFAULT_MODEL = "gpt-4.1"


def read_api_key() -> str:
    """Return API key from env or api_key.txt."""
    env_key = os.getenv("OPENAI_API_KEY")
    if env_key:
        return env_key.strip()
    if DEFAULT_KEY_FILE.exists():
        return DEFAULT_KEY_FILE.read_text(encoding="utf-8").strip()
    raise SystemExit(
        "OpenAI API key not found. Set OPENAI_API_KEY or create "
        f"{DEFAULT_KEY_FILE}"
    )


def parse_event_list(source: str, var_name: str) -> List[str]:
    """Extract the list of event names defined in the C++ vector."""
    pattern = rf"{var_name}\s*=\s*\{{(.*?)\}};"
    match = re.search(pattern, source, flags=re.DOTALL)
    if not match:
        raise ValueError(f"Unable to find definition for {var_name}")
    return re.findall(r'"([^"]+)"', match.group(1))


def parse_transitions(source: str) -> Dict[str, Dict[str, str]]:
    """
    Parse K.SetTransition entries into a state -> event -> next state map.
    """
    transitions: Dict[str, Dict[str, str]] = {}
    pattern = r'K\.SetTransition\("([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\)'
    for state, event, nxt in re.findall(pattern, source):
        transitions.setdefault(state, {})[event] = nxt
    if not transitions:
        raise ValueError("No K.SetTransition(...) entries found.")
    return transitions


def parse_states(source: str) -> List[str]:
    """Extract the declared DES states (order preserved)."""
    states: List[str] = []
    for state in re.findall(r'K\.InsState\("([^"]+)"\)', source):
        if state not in states:
            states.append(state)
    return states


def build_prompt(
    goal: str,
    states: List[str],
    controllable: List[str],
    uncontrollable: List[str],
    transitions: Dict[str, Dict[str, str]],
    include_existing: bool,
) -> str:
    """Assemble the coverage/supervisor design prompt."""
    states = sorted(states)
    lines: List[str] = []
    lines.append(
        "We operate a DES-based supervisor for a mobile robot with four "
        "sensing modes.  Each SetTransition entry describes either a sensor "
        "mode switch (uncontrollable event) or an allowed control action."
    )
    lines.append("")
    lines.append("Controllable events: " + ", ".join(controllable))
    lines.append("Uncontrollable events: " + ", ".join(uncontrollable))
    lines.append("Current states: " + ", ".join(states))
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
            "above.  You may propose additional helper states if justified."
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
        '     "K.SetTransition(\\"state\\", \\"event\\", \\"next\\")",\n'
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
    """Send the prompt to the Chat Completions API."""
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
    return json.loads(content)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Generate improved DES supervisor transitions via GPT."
    )
    parser.add_argument(
        "--controllable-events",
        nargs="+",
        help="Override controllable events list (otherwise read from C++).",
    )
    parser.add_argument(
        "--uncontrollable-events",
        nargs="+",
        help="Override uncontrollable events list (otherwise read from C++).",
    )
    parser.add_argument(
        "--goal",
        default="Cover the entire arena efficiently while staying safe",
        help="High level objective passed to GPT.",
    )
    parser.add_argument(
        "--cpp",
        type=Path,
        default=DEFAULT_CPP,
        help="Path to robot_navigation.cpp (for parsing events/transitions).",
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
        "--no-current",
        action="store_true",
        help="Do not include existing transitions from robot_navigation.cpp.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the prompt instead of calling the API.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to save the raw JSON response.",
    )
    args = parser.parse_args(argv)

    source: str | None = None
    controllable = args.controllable_events
    uncontrollable = args.uncontrollable_events
    transitions: Dict[str, Dict[str, str]] = {}

    def ensure_source() -> str:
        nonlocal source
        if source is None:
            source = args.cpp.read_text(encoding="utf-8")
        return source

    if controllable is None:
        controllable = parse_event_list(ensure_source(), "kControllableEvents")
    if uncontrollable is None:
        uncontrollable = parse_event_list(ensure_source(), "kUncontrollableEvents")
    if not args.no_current:
        transitions = parse_transitions(ensure_source())

    states = parse_states(ensure_source())
    if not states and transitions:
        states = sorted(transitions)
    if not states:
        raise SystemExit("No DES states found in the source file.")

    prompt = build_prompt(
        goal=args.goal,
        states=states,
        controllable=controllable,
        uncontrollable=uncontrollable,
        transitions=transitions,
        include_existing=not args.no_current,
    )

    if args.dry_run:
        print(prompt)
        return 0

    api_key = read_api_key()
    result = call_chat_completion(api_key, prompt, args.model, args.temperature)
    print(json.dumps(result, indent=2))
    transitions_out = result.get("transitions")
    if transitions_out:
        print("\nSuggested K.SetTransition lines:\n")
        for line in transitions_out:
            print(line)
    if args.save:
        args.save.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
