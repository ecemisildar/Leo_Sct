#!/usr/bin/env python3
"""
Evaluates multiple prompt text files for SCT supervisor generation.

Each .txt file in the 'prompts/' directory is treated as a complete system/user prompt.
For each prompt:
  - Send to the LLM (GPT-4.1)
  - Save the generated supervisors
  - Evaluate syntax + structure validity
  - Record token counts and quality scores
  - Save summary CSV + PNG plot

Author: Ecem Isildar
"""

import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from openai import OpenAI
import tiktoken

# ==========================================================
#  1. Configuration
# ==========================================================
MODEL = "gpt-4.1"
TEMPERATURE = 0.3
MAX_OUTPUT_TOKENS = 1000

PROMPT_DIR = Path("prompts_2")                 # where .txt prompt files live
FBD_DIR = Path("fbd")                        # where controllable/uncontrollable event files live
RESULT_DIR = Path("prompt_eval_results_for_prompts_2")     # output folder
RESULT_DIR.mkdir(parents=True, exist_ok=True)

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

TASK_OVERVIEW = (
    "TASK OVERVIEW:\n"
    "You are synthesizing Supervisory Control Theory (SCT) automata for a swarm robot.\n"
    "Explain the goal, follow SCT safety/determinism rules, and only use the events in the provided vocab."
)

EVENT_DESCRIPTIONS = {
    "controllable": {
        "move_forward": "Advance one cell when the path is clear.",
        "move_backward": "Retreat slightly to recover from a blockage.",
        "random_walk": "Pick a safe random direction to explore.",
        "clockwise_turn": "Turn right (90°) to avoid a left obstacle.",
        "counterclockwise_turn": "Turn left (90°) to avoid a right obstacle.",
        "full_rotate": "Perform a 360° scan when surrounded."
    },
    "uncontrollable": {
        "obstacle_front": "Sensor detects an obstacle directly ahead.",
        "obstacle_left": "Sensor detects an obstacle on the left side.",
        "obstacle_right": "Sensor detects an obstacle on the right side.",
        "path_clear": "No obstacles in the front/left/right arcs."
    }
}

EXAMPLE_SUPERVISORS = [
    """
Example supervisor (clockwise_turn):
### Supervisor for clockwise_turn
States:
    name=0, initial=True, marked=True
    name=1, initial=False, marked=True

Transitions:
    source=0, target=0, event=path_clear
    source=0, target=0, event=obstacle_front
    source=0, target=0, event=obstacle_right

    source=0, target=1, event=obstacle_left

    source=1, target=1, event=obstacle_left

    source=1, target=0, event=clockwise_turn
    source=1, target=0, event=path_clear
    source=1, target=0, event=obstacle_front
    source=1, target=0, event=obstacle_right
""".strip(),
    """
Example supervisor (counterclockwise_turn):
### Supervisor for counterclockwise_turn
States:
    name=0, initial=True, marked=True
    name=1, initial=False, marked=True

Transitions:
    source=0, target=0, event=path_clear
    source=0, target=0, event=obstacle_front
    source=0, target=0, event=obstacle_left

    source=0, target=1, event=obstacle_right

    source=1, target=1, event=obstacle_right

    source=1, target=0, event=counterclockwise_turn
    source=1, target=0, event=path_clear
    source=1, target=0, event=obstacle_front
    source=1, target=0, event=obstacle_left
""".strip(),
]

RESPONSE_REQUIREMENTS = (
    "RESPONSE REQUIREMENTS:\n"
    "- Follow the supervisor format exactly (States/Transitions blocks).\n"
    "- Every supervisor must include at least one transition that fires its controllable event from a valid state.\n"
    "- Show determinism and include all uncontrollables at each state."
)

# ==========================================================
#  2. Load event vocab
# ==========================================================
def load_events():
    """Read controllable and uncontrollable events from text files."""
    cont_file = FBD_DIR / "controllable_events.txt"
    unctrl_file = FBD_DIR / "uncontrollable_events.txt"

    with open(cont_file) as f:
        controllable_text = f.read()
    with open(unctrl_file) as f:
        uncontrollable_text = f.read()

    allowed_unctrl, allowed_ctrl = [], []
    for line in uncontrollable_text.splitlines():
        if "name=" in line:
            allowed_unctrl.append(line.split("name=")[1].split(",")[0].strip())
    for line in controllable_text.splitlines():
        if "name=" in line and "controllable=True" in line:
            allowed_ctrl.append(line.split("name=")[1].split(",")[0].strip())
    return allowed_unctrl, allowed_ctrl


allowed_unctrl, allowed_ctrl = load_events()


def build_event_meanings(unctrl_names, ctrl_names):
    """Return a description block listing every allowed event and meaning."""
    lines = ["EVENT MEANINGS:"]
    if ctrl_names:
        lines.append("Controllable events:")
        for name in ctrl_names:
            meaning = EVENT_DESCRIPTIONS["controllable"].get(
                name, "Use an SCT-compatible action for this controllable event."
            )
            lines.append(f"- {name}: {meaning}")
    if unctrl_names:
        lines.append("Uncontrollable events:")
        for name in unctrl_names:
            meaning = EVENT_DESCRIPTIONS["uncontrollable"].get(
                name, "Environment signal describing perceived context."
            )
            lines.append(f"- {name}: {meaning}")
    return "\n".join(lines)


def example_section():
    """Format the two reference supervisors for inclusion in every prompt."""
    return "\n\n".join(EXAMPLE_SUPERVISORS)


def response_requirements(ctrl_names):
    """Customize the response checklist with the controllable vocabulary."""
    ctrl_list = ", ".join(ctrl_names)
    return (
        f"{RESPONSE_REQUIREMENTS}\n"
        f"- Use at least one of the controllable events ({ctrl_list}) in the answer."
    )


def assemble_prompt(base_prompt, ctrl_names, unctrl_names):
    """Inject explanations, vocab meanings, and examples into each prompt."""
    parts = [
        TASK_OVERVIEW.strip(),
        build_event_meanings(unctrl_names, ctrl_names),
        example_section(),
        "Prompt instructions:\n" + base_prompt.strip(),
        response_requirements(ctrl_names),
    ]
    return "\n\n".join(parts)

# ==========================================================
#  3. Validators
# ==========================================================
def syntax_score(text: str) -> float:
    """Checks for key sections in the generated automaton text."""
    has_supervisor = bool(re.search(r'### Supervisor', text))
    has_states = bool(re.search(r'States:', text))
    has_transitions = bool(re.search(r'Transitions:', text))
    return (has_supervisor + has_states + has_transitions) / 3.0


def parse_transitions(text: str):
    """
    Extract (source_state, target_state, event_name) entries from supervisor text.
    Supports numeric or named states. Robust to spaces, commas, weird formatting.
    """
    transitions = []

    # Matches lines containing source=..., target=..., and event=...
    for line in text.splitlines():
        if "source=" in line and "event=" in line:
            src_match = re.search(r'source\s*=\s*([A-Za-z0-9_]+)', line)
            tgt_match = re.search(r'target\s*=\s*([A-Za-z0-9_]+)', line)
            evt_match = re.search(r'event\s*=\s*([A-Za-z0-9_]+)', line)

            if not src_match or not tgt_match or not evt_match:
                continue

            transitions.append(
                (
                    src_match.group(1).strip(),
                    tgt_match.group(1).strip(),
                    evt_match.group(1).strip(),
                )
            )

    return transitions



def structural_score(text: str, uncontrollables, controllables) -> float:
    """Evaluate determinism, totality, vocab usage, and safety of transitions."""
    parts = re.split(r'### Supervisor for', text)
    if len(parts) <= 1:
        return 0.0

    total = 0.0
    # parts -> ['', name1\n..., name2\n..., ...]
    for block in parts[1:]:
        block_score = 1.0
        transitions = parse_transitions(block)
        if not transitions:
            total += 0.0
            continue

        events_in_block = {evt for _, _, evt in transitions}
        allowed_events = set(uncontrollables) | set(controllables)
        unknown_events = events_in_block - allowed_events
        if unknown_events:
            # Unknown vocabulary is a hard failure for this block.
            total += 0.0
            continue

        # Determinism: no duplicate (state,event) pairs
        pairs = [(src, evt) for src, _, evt in transitions]
        if len(pairs) != len(set(pairs)):
            block_score -= 0.25

        # Totality: every state must react to all uncontrollables
        states = {src for src, _, _ in transitions}
        for s in states:
            events = [evt for src, _, evt in transitions if src == s]
            missing = [e for e in uncontrollables if e not in events]
            if missing:
                block_score -= 0.25 * len(missing) / max(1, len(uncontrollables))

        # Safety: states reached under obstacle_front must disable forward
        blocked_states = {tgt for _, tgt, evt in transitions if evt == "obstacle_front"}
        unsafe = [
            (src, tgt)
            for src, tgt, evt in transitions
            if evt == "move_forward" and src in blocked_states
        ]
        if unsafe:
            block_score -= 0.25

        total += max(0.0, block_score)

    return round(total / (len(parts) - 1), 2)


def evaluate_output(text: str, uncontrollables, controllables):
    """Combine syntax and structure scores."""
    syn = syntax_score(text)
    struc = structural_score(text, uncontrollables, controllables)
    return round((syn + struc) / 2, 2), syn, struc


def detect_invalid_events(text: str, allowed_events):
    """Return a sorted list of events not in the allowed vocabulary."""
    transitions = parse_transitions(text)
    used_events = {evt for _, _, evt in transitions}
    return sorted(used_events - set(allowed_events))


def find_forward_conflicts(text: str):
    """Identify states that both receive obstacle_front and issue move_forward."""
    transitions = parse_transitions(text)
    blocked_states = {tgt for _, tgt, evt in transitions if evt == "obstacle_front"}
    return [
        (src, tgt)
        for src, tgt, evt in transitions
        if evt == "move_forward" and src in blocked_states
    ]


def split_supervisors(text: str):
    """Return (name, body) tuples for every supervisor block."""
    pattern = r"### Supervisor for ([A-Za-z0-9_]+)"
    chunks = re.split(pattern, text)
    supervisors = []
    for i in range(1, len(chunks), 2):
        name = chunks[i].strip()
        body = chunks[i + 1].strip()
        supervisors.append((name, body))
    return supervisors


def find_missing_controllables(text: str, controllables):
    """Supervisors that never emit their own controllable event."""
    missing = []
    for name, body in split_supervisors(text):
        if name not in controllables:
            # skip mixed supervisors (examples containing turns/full_rotate)
            continue
        transitions = parse_transitions(body)
        if not any(evt == name for _, _, evt in transitions):
            missing.append(name)
    return missing


def find_used_controllables(text: str, controllables):
    """Return sorted controllable events mentioned in the output."""
    transitions = parse_transitions(text)
    return sorted({evt for _, _, evt in transitions if evt in controllables})


# ==========================================================
#  4. Evaluate prompts from files
# ==========================================================
enc = tiktoken.encoding_for_model(MODEL)
results = []

prompt_files = sorted(PROMPT_DIR.glob("*.txt"))
if not prompt_files:
    raise FileNotFoundError(f"No .txt prompts found in {PROMPT_DIR}/")

for pf in prompt_files:
    name = pf.stem
    prompt = pf.read_text()
    vocab_block = (
        "\n\nALLOWED EVENT NAMES (no other names are permitted):\n"
        f"Controllable: {', '.join(allowed_ctrl)}\n"
        f"Uncontrollable: {', '.join(allowed_unctrl)}\n"
    )
    enhanced_prompt = assemble_prompt(prompt, allowed_ctrl, allowed_unctrl)
    prompt_payload = f"{enhanced_prompt}\n{vocab_block}"

    print(f"\n▶ Evaluating prompt: {name}")

    # ---- LLM call ----
    resp = client.responses.create(
        model=MODEL,
        temperature=TEMPERATURE,
        input=prompt_payload,
        max_output_tokens=MAX_OUTPUT_TOKENS,
    )
    output = resp.output_text

    # ---- Token counts ----
    in_tok = len(enc.encode(prompt_payload))
    out_tok = len(enc.encode(output))

    # ---- Evaluation ----
    total, syn, struc = evaluate_output(output, allowed_unctrl, allowed_ctrl)

    # ---- Hard constraints ----
    invalid_events = detect_invalid_events(output, allowed_unctrl + allowed_ctrl)
    if invalid_events:
        print(f"  [!] Invalid events detected: {', '.join(invalid_events)}")

    forward_conflicts = find_forward_conflicts(output)
    if forward_conflicts:
        print("  [!] Unsafe move_forward usage after obstacle_front:")
        for src, tgt in forward_conflicts:
            print(f"      state {src} -> {tgt}")

    missing_ctrl = find_missing_controllables(output, allowed_ctrl)
    if missing_ctrl:
        print("  [!] Missing controllable transitions for supervisors:")
        for name in missing_ctrl:
            print(f"      {name}")

    used_ctrl = find_used_controllables(output, allowed_ctrl)
    if not used_ctrl:
        print("  [!] No controllable events were emitted in the generated output.")

    # ---- Save generated output ----
    output_file = RESULT_DIR / f"{name}_output.txt"
    output_file.write_text(output)

    results.append({
        "prompt_file": pf.name,
        "input_tokens": in_tok,
        "output_tokens": out_tok,
        "syntax_score": syn,
        "structure_score": struc,
        "total_score": total,
        "output_path": str(output_file),
    })


# ==========================================================
#  5. Save summary and plot
# ==========================================================
df = pd.DataFrame(results)
csv_path = RESULT_DIR / "prompt_results.csv"
df.to_csv(csv_path, index=False)
print(f"\nSummary saved to {csv_path}")

plt.figure(figsize=(7, 5))
plt.scatter(df["input_tokens"], df["total_score"], color="royalblue")
for _, r in df.iterrows():
    plt.text(r["input_tokens"], r["total_score"], r["prompt_file"], fontsize=7)
plt.xlabel("Prompt token count")
plt.ylabel("Total validity score")
plt.title("Prompt efficiency vs SCT validity (read from text files)")
plt.grid(True)
plt.tight_layout()
plt.savefig(RESULT_DIR / "prompt_efficiency.png", dpi=200)
plt.show()

print("\n=== Top 5 Prompts by Score ===")
print(df.sort_values("total_score", ascending=False).head(5))
