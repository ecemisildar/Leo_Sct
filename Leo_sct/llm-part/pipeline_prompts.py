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
    "Determinism is required: for each (state, event) pair, specify EXACTLY ONE next state. No duplicates.",
    "Every transition must be labeled by one event from the lists.",
    "For EVERY state, include an outgoing transition for ALL uncontrollable events (total w.r.t. uncontrollable events).",
    "Every non-terminal state must enable exactly one controllable action, unless the state is purely a perception/decision state used only for uncontrollable-event branching.",
    "Prefer a single clear controllable action per state, but choose the action that maximizes forward progress when safe.",
    "Runtime semantics: uncontrollable events are processed BEFORE a controllable action each step.",
    "Therefore, in action/commit/scan/recovery states, non-critical uncontrollable events (e.g., path_clear) should usually SELF-LOOP to avoid preempting the intended controllable in the same step.",
    "Only safety-critical uncontrollable events (obstacle_*) should force leaving an action/commit state.",
    "When path_clear is observed after a recovery, return to a forward-progress state quickly rather than chaining extra scan or recovery states.",

    "move_backward is a brief one-step escape action, not a normal exploration or recovery behavior.",
    "Use move_backward only after obstacle_front, and only when a short backward escape is needed before turning or resuming exploration.",
    "Do not use move_backward in clear, initial, search, path_clear, obstacle_left, or obstacle_right states.",
    "move_backward must only be reachable from dedicated front-obstacle recovery states.",
    "Do not transition into move_backward directly from clear, move_forward, random_walk, obstacle_left, or obstacle_right handling states.",
    "After move_backward, transition only to a short directed-turn recovery state or a decision/perception state, never to another backward state.",
    "Use move_backward carefully: the robot has only a forward-looking camera and receives no rear obstacle input, so backward motion must be rare, brief, and recovery-oriented.",

    "full_rotate is a last-resort reorientation action, not a standard obstacle-recovery action.",
    "Do not use full_rotate as the immediate response to obstacle_left or obstacle_right.",
    "Do not use full_rotate as the default response to obstacle_front if a short directed turn can resolve the blockage.",
    "Ordinary obstacle recovery must prefer short directed turns (rotate_clockwise or rotate_counterclockwise).",
    "Short directed turns are the default obstacle-recovery behavior; full_rotate is reserved for last-resort reorientation only.",
    "Use full_rotate only when short directed turns repeatedly fail to restore progress.",
    "Use full_rotate only after repeated blocked conditions via a dedicated stuck/last_resort helper state.",
    "full_rotate must only be reachable from a dedicated stuck/last_resort state, not from clear, move_forward, random_walk, or immediate obstacle-recovery states.",
    "Do not transition directly into full_rotate from clear, move_forward, random_walk, obstacle_left, obstacle_right, or obstacle_front handling states.",
    "Any state whose primary controllable is full_rotate must be one-shot and must transition out immediately after execution.",
    "A full_rotate state must not self-loop on any controllable event.",
    "After full_rotate, the supervisor must return to a forward-progress or short-turn decision state, not to another scan, rotate, or full_rotate state.",
    "Do not chain full_rotate actions or transition from full_rotate into another full_rotate-capable state without first attempting forward progress.",

    "Do not use repeated rotation or full_rotate as a substitute for exploration progress.",
    "After any recovery action, the supervisor must attempt forward progress again as soon as safely possible.",
    "Pure rotation sequences must remain brief and recovery-oriented; they must not become steady-state exploration behavior.",

    "Avoid oscillations: do NOT enable both rotate_clockwise and rotate_counterclockwise as controllables in the same state.",
    "Keep the behavior consistent: obstacle_left should prefer rotate_clockwise OR a clear escape policy; obstacle_right should prefer rotate_counterclockwise (or vice versa), but do not alternate rapidly.",
    "You MAY introduce helper states such as forward_commit, rotate_commit_cw, rotate_commit_ccw, recover_back, stuck, last_resort, but keep the total number of helper states small.",
    "Commit/scan/recovery states must be ONE-SHOT on their primary controllable action: the primary controllable must transition OUT to a decision/perception state (e.g., clear), NOT self-loop.",
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
    "move_forward": "apply forward motion for 0.2 s (one pulse).",
    "random_walk": "apply a short exploratory wandering motion for 0.2 s; suitable in clear/search states but not obstacle recovery states.",
    "move_backward": "apply backward motion for 0.2 s (one pulse).",
    "rotate_clockwise": "apply clockwise rotation for 0.2 s (one pulse).",
    "rotate_counterclockwise": "apply counterclockwise rotation for 0.2 s (one pulse).",
    "full_rotate": "rotate 360 degrees (atomic action; completes a full scan).",
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
    user_task: str | None,
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
    lines.append(
        "Important runtime semantics: controllable motion events are executed as open-loop pulses."
    )
    lines.append(
        "A forward or random_walk action may continue briefly before the next obstacle event is processed."
    )
    lines.append(
        "Therefore the design must remain safe, but it should also bias strongly toward coverage: use conservative recovery only when needed,"
        " and return to forward exploration as soon as the path becomes clear."
    )
    lines.append("")
    if user_task:
        lines.append("User task: " + user_task)
        lines.append("Task profile goal: " + goal)
    else:
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
