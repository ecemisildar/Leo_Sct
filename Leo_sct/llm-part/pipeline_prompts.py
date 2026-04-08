from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Sequence

THIS_DIR = Path(__file__).resolve().parent
DEFAULT_EXAMPLE_TASKS_PATH = THIS_DIR / "example_tasks" / "examples.json"

SYSTEM_PROMPT = (
    "You are a DES/control expert. Respond ONLY with valid JSON. "
    "Design both plant automata G and specification automata E. "
    "Do not add commentary outside JSON."
)

DEFAULT_CONSTRAINTS = [
    "Do NOT introduce any new controllable or uncontrollable events. Use ONLY the provided event lists.",
    "Do NOT invent sensor events beyond the given uncontrollable list (e.g., no 'battery_low', etc.).",
    "All transitions must be formatted exactly as: (\"state\", \"event\", \"next\").",
    "Determinism required: for each (state, event) pair, specify EXACTLY ONE next state. No duplicates.",
    "No epsilon transitions. Every transition must be labeled by one event from the lists.",
    "For EVERY state, include an outgoing transition for ALL uncontrollable events (total w.r.t. uncontrollables).",
    "For EVERY non-terminal state, include at least ONE controllable transition (otherwise the robot may stall).",
    "Enable EXACTLY ONE controllable action in each non-terminal state unless the task profile explicitly allows otherwise.",
    "Runtime semantics: uncontrollable events are processed BEFORE a controllable action each step.",
    "Therefore, in action/commit/scan/recovery states, non-critical uncontrollables (e.g., path_clear, marker_seen/marker_lost) should usually SELF-LOOP to avoid preempting the intended controllable in the same step.",
    "Only safety-critical uncontrollables (obstacle_*, marker_close) should force leaving an action/commit state.",
    "Never allow move_forward from obs_front (obstacle_front).",
    "After obstacle_front, do NOT return directly to move_forward; pass through a recovery/escape step (e.g., move_backward and/or rotate/full_rotate).",
    "Obstacle recovery states must be deterministic: they must not enable random_walk, and they must not enable multiple alternative recoveries.",
    "If marker_close occurs in ANY state, transition to a terminal goal/stop state where stop is the only controllable action.",
    "Avoid oscillations: do NOT enable both rotate_clockwise and rotate_counterclockwise as controllables in the same state.",
    "Keep the behavior consistent: obs_left should prefer rotate_clockwise OR a clear escape policy; obs_right should prefer rotate_counterclockwise (or vice versa), but do not alternate rapidly.",
    "You MAY introduce helper states such as forward_commit, rotate_commit_cw, rotate_commit_ccw, scan_full, recover_back, marker_track, marker_approach.",
    "Commit/scan/recovery states must be ONE-SHOT on their primary controllable action: the primary controllable must transition OUT to a decision/perception state (e.g., clear), NOT self-loop.",
    "Examples (illustrative only): forward_commit: move_forward->clear; rotate_commit_cw: rotate_clockwise->clear; rotate_commit_ccw: rotate_counterclockwise->clear; scan_full: full_rotate->clear; recover_back: move_backward->scan_full or ->clear.",
    "Do NOT create infinite controllable loops like full_rotate->scan_full, move_backward->recover_back, or move_forward->forward_commit.",
    "When marker_seen occurs (and marker_close has not occurred), prefer switching to a marker-tracking mode/state where the primary controllable is move_to_marker.",
    "When marker_lost occurs during marker tracking, transition to a search behavior (e.g., scan_full) rather than continuing move_to_marker blindly."
]

DEFAULT_STATE_SEMANTICS = {
    "clear": "no obstacle in front/left/right; safe to advance.",
    "obs_front": "obstacle detected in front region; forward is unsafe.",
    "obs_left": "obstacle close on left; left turn is unsafe; right turn may be preferred.",
    "obs_right": "obstacle close on right; right turn is unsafe; left turn may be preferred.",
}

DEFAULT_EVENT_SEMANTICS = {
    "path_clear": "sensors say forward corridor is clear (no front obstacle).",
    "obstacle_front": "front sensor region is blocked.",
    "obstacle_left": "left sensor region is blocked.",
    "obstacle_right": "right sensor region is blocked.",
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


def merge_prompt_semantics(
    state_semantics_overrides: Dict[str, str] | None,
    event_semantics_overrides: Dict[str, str] | None,
) -> tuple[Dict[str, str], Dict[str, str]]:
    state_semantics = dict(DEFAULT_STATE_SEMANTICS)
    event_semantics = dict(DEFAULT_EVENT_SEMANTICS)
    if state_semantics_overrides:
        state_semantics.update(state_semantics_overrides)
    if event_semantics_overrides:
        event_semantics.update(event_semantics_overrides)
    return state_semantics, event_semantics


def load_example_tasks(path: Path = DEFAULT_EXAMPLE_TASKS_PATH) -> List[Dict[str, object]]:
    if not path.exists():
        return []
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise ValueError("Example tasks JSON must contain a top-level list.")
    return payload


def _render_examples(examples: Sequence[Dict[str, object]]) -> str:
    if not examples:
        return "No external examples provided."

    def render_automata(items: object, label: str) -> List[str]:
        rendered: List[str] = []
        if not isinstance(items, list):
            return rendered
        for automaton in items:
            if not isinstance(automaton, dict):
                continue
            name = str(automaton.get("name", "?"))
            purpose = str(automaton.get("purpose", "")).strip()
            rendered.append(f"  {label} {name}: {purpose}")

            states = automaton.get("states")
            if isinstance(states, list) and states:
                rendered.append("    States: " + ", ".join(str(state) for state in states))

            controllable = automaton.get("controllable_events")
            if isinstance(controllable, list) and controllable:
                rendered.append(
                    "    Controllable: " + ", ".join(str(event) for event in controllable)
                )

            uncontrollable = automaton.get("uncontrollable_events")
            if isinstance(uncontrollable, list) and uncontrollable:
                rendered.append(
                    "    Uncontrollable: " + ", ".join(str(event) for event in uncontrollable)
                )

            transitions = automaton.get("transitions")
            if isinstance(transitions, list) and transitions:
                rendered.append("    Transitions:")
                for transition in transitions:
                    rendered.append(f"      {transition}")
        return rendered

    lines: List[str] = []
    for example in examples:
        name = str(example.get("name", "unnamed_example"))
        summary = str(example.get("summary", "")).strip()
        patterns = example.get("patterns") or []
        plants = example.get("plants") or []
        specifications = example.get("specifications") or []

        lines.append(f"- {name}: {summary}")
        if isinstance(patterns, list) and patterns:
            lines.append("  Patterns: " + "; ".join(str(item) for item in patterns))
        lines.extend(render_automata(plants, "Plant"))
        lines.extend(render_automata(specifications, "Spec"))
    return "\n".join(lines)


def build_pipeline_prompt(
    goal: str,
    controllable: Sequence[str],
    uncontrollable: Sequence[str],
    guidance: Sequence[str] | None,
    state_semantics: Dict[str, str],
    event_semantics: Dict[str, str],
    example_tasks: Sequence[Dict[str, object]],
) -> str:
    lines: List[str] = []
    lines.append("Design a DES model for a mobile robot task.")
    lines.append("Return both plant automata G and specification automata E.")
    lines.append("")
    lines.append("Goal: " + goal)
    lines.append("Controllable events: " + ", ".join(controllable))
    lines.append("Uncontrollable events: " + ", ".join(uncontrollable))
    lines.append("")
    if state_semantics:
        lines.append("Reference state semantics:")
        for name, meaning in state_semantics.items():
            lines.append(f"- {name}: {meaning}")
        lines.append("")
    if event_semantics:
        lines.append("Reference event semantics:")
        for name in list(controllable) + list(uncontrollable):
            if name in event_semantics:
                lines.append(f"- {name}: {event_semantics[name]}")
        lines.append("")
    if guidance:
        lines.append("Design constraints:")
        for item in guidance:
            lines.append(f"- {item}")
        lines.append("")
    lines.append("Structured example tasks:")
    lines.append(_render_examples(example_tasks))
    lines.append("")
    lines.append("Output JSON schema:")
    lines.append("{")
    lines.append('  "rationale": "short reasoning",')
    lines.append('  "strategy": ["design choice 1", "design choice 2"],')
    lines.append('  "G": [')
    lines.append("    {")
    lines.append('      "name": "G1",')
    lines.append('      "purpose": "what this plant automaton models",')
    lines.append('      "states": ["q1", "q2"],')
    lines.append('      "initial_states": ["q1"],')
    lines.append('      "marked_states": ["q1", "q2"],')
    lines.append('      "controllable_events": ["event_a"],')
    lines.append('      "uncontrollable_events": ["event_b"],')
    lines.append('      "transitions": ["(\\"q1\\", \\"event_a\\", \\"q2\\")"]')
    lines.append("    }")
    lines.append("  ],")
    lines.append('  "E": [')
    lines.append("    {")
    lines.append('      "name": "E1",')
    lines.append('      "purpose": "what this specification constrains",')
    lines.append('      "states": ["q1", "q2"],')
    lines.append('      "initial_states": ["q1"],')
    lines.append('      "marked_states": ["q1", "q2"],')
    lines.append('      "controllable_events": ["event_a"],')
    lines.append('      "uncontrollable_events": ["event_b"],')
    lines.append('      "transitions": ["(\\"q1\\", \\"event_b\\", \\"q2\\")"]')
    lines.append("    }")
    lines.append("  ]")
    lines.append("}")
    lines.append("")
    lines.append("Rules:")
    lines.append("- Use only the provided event names unless a genuinely necessary plant event is justified in the rationale.")
    lines.append("- Every automaton must be deterministic.")
    lines.append("- Use tuple-string transitions exactly in the form (\"state\", \"event\", \"next\").")
    lines.append("- Return at least one plant automaton in G and at least one specification automaton in E.")
    lines.append("- Keep the JSON compact and valid.")
    return "\n".join(lines)
