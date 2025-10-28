#!/usr/bin/env python3
import os
from openai import OpenAI
from pathlib import Path

# ===========================
#  Configuration
# ===========================
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

FBD_DIR = Path("fbd")
controllable_path = FBD_DIR / "controllable_events.txt"
uncontrollable_path = FBD_DIR / "uncontrollable_events.txt"
output_path = Path("supervisors_xml") / "test.txt"

# ===========================
#  Read converted FBD text files
# ===========================
with open(controllable_path, "r") as f:
    controllable_text = f.read().strip()
with open(uncontrollable_path, "r") as f:
    uncontrollable_text = f.read().strip()

# --- Extract controllable event names automatically ---
controllable_events = []
for line in controllable_text.splitlines():
    if "name=" in line and "controllable=True" in line:
        part = line.strip().split("name=")[1]
        event_name = part.split(",")[0].strip()
        controllable_events.append(event_name)

# ===========================
#  Example specification for full_rotate
# ===========================
example_full_rotate = """
HARD POLICY for Supervisor of move_forward (must be followed exactly):
- States (exactly two):
  - name=0, initial=True, marked=True
  - name=1, initial=False, marked=True
- Events: use exactly those given (obstacle_front, obstacle_left, obstacle_right, path_clear, move_forward).
- Transitions (and ONLY these):
  From state 0:
    - source=0, target=0, event=obstacle_front
    - source=0, target=0, event=obstacle_left
    - source=0, target=0, event=obstacle_right
    - source=0, target=1, event=path_clear
  From state 1:
    - source=1, target=0, event=move_forward
    - source=1, target=0, event=obstacle_front
    - source=1, target=0, event=obstacle_left
    - source=1, target=0, event=obstacle_right
    - source=1, target=1, event=path_clear
- Do NOT include any other transitions. Do NOT duplicate transitions. Deterministic only.

"""

# ===========================
#  Task / Behavior description
# ===========================
behavior_description = """
The robot moves in an environment with obstacles detected by its camera.
Uncontrollable events are sensor outputs; controllable events are robot actions.

Objectives:
1. Avoid collisions — disable movement actions if an obstacle is detected.
2. Turn away from obstacles (left/right avoidance).
3. Fully rotate when the obstacle is in front.
4. Perform random walk or move forward when path is clear.

Each controllable event must have its own control specification (supervisor)
that defines when it is enabled or disabled depending on sensor (uncontrollable) events.
"""

# ===========================
#  GPT Prompt
# ===========================
prompt = f"""
You are an expert in Supervisory Control Theory (SCT) for autonomous robots.

You will generate one control specification per controllable event.

Below are the FBD-derived diagrams:

Controllable diagram:
{controllable_text}

Uncontrollable diagram:
{uncontrollable_text}

Task description:
{behavior_description}

Controllable events detected:
{', '.join(controllable_events)}

Use the following as an example format and logic for 'full_rotate':
{example_full_rotate}

Now, generate similar control specifications for the remaining controllable events.
Follow the same structured text format:

### Supervisor for <event_name>
States:
  - ...
Events:
  - ...
Transitions:
  - source=..., target=..., event=...
"""

# ===========================
#  GPT Request
# ===========================
response = client.responses.create(
    model="gpt-4.1",
    input=prompt,
    temperature=0.3,
    max_output_tokens=5000,
)

output_text = response.output_text

# ===========================
#  Save result
# ===========================
output_path.parent.mkdir(parents=True, exist_ok=True)
with open(output_path, "w") as f:
    f.write(output_text)

print(f"Multi-supervisor control specs generated and saved to: {output_path}")
