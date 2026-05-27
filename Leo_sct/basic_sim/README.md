# Basic SCT Grid Simulation

This folder contains a small, ROS-free grid simulator for trying Nadzoru SCT
automata before sending them to the robot stack.

## Files

- `grid_sim.py`: command-line runner.
- `grid_world.py`: grid environment and agent motion.
- `automata.py`: loaders for Nadzoru XML, generated supervisor JSON, and compact SCT YAML.
- `llm_generate_ge.py`: asks the LLM for one `G` and one `E`, then saves separate JSON and XML files.
- `synthesize_sloc.py`: builds `Gloc.xml`, `Kloc.xml`, and `Sloc.xml` from sample `G/E` XML files using Nadzoru.
- `sample_G.xml`: permissive sample plant automaton.
- `sample_explore_nadzoru.json`: default exploration supervisor.

## Run

From the repository root:

```bash
python3 basic_sim/grid_sim.py
```

This opens a Matplotlib plot that updates as the agents move. If you are
running without a display, save the final plot instead:

```bash
python3 basic_sim/grid_sim.py --save-plot basic_sim/final_grid.png
```

Run with your generated Nadzoru files:

```bash
python3 basic_sim/grid_sim.py \
  --automata llm-part/full_pipeline/explore_1_nadzoru.json \
  --agents 3 \
  --steps 40 \
  --width 10 \
  --height 8 \
  --obstacle 4,2 \
  --obstacle 4,3
```

The plot includes a 2x2 red gathering area only for red-area tasks. It is
enabled automatically when the automata path is under `prompt_8`, `prompt_11`,
or `prompt_13`, or explicitly with `--red-area-origin x,y`, where `x,y` is the
top-left cell.
A 2x2 blue danger area is enabled automatically when the automata path is under
`prompt_9` or `prompt_12`, or explicitly with `--blue-area-origin x,y`.
When running the combined red-and-blue task under `prompt_10`, both areas are
enabled automatically and placed in different locations.
When running the object-pushing task under `prompt_13`, four movable 1-cell
objects are also enabled automatically in random non-adjacent free cells using
the run seed. Override them with repeated `--object x,y` arguments.

Use terminal output instead of plotting:

```bash
python3 basic_sim/grid_sim.py --text
```

You can also pass Nadzoru XML files such as `llm-part/full_pipeline/Sloc3.xml`
or compact YAML files such as `swarm_basics/config/sup_gpt.yaml`.

## Sample G/E to Sloc

Generate `user_prompt.txt`, `json/G.json`, `json/E.json`, `xml/G.xml`,
`xml/E.xml`, `synthesized/Sloc.xml`, and `synthesized/Sloc.yaml` with the
LLM:

```bash
python3 basic_sim/llm_generate_ge.py \
  --user-prompt-index 1 \
  --output-dir basic_sim/llm_generated
```

For a single selected prompt, outputs are placed under a prompt-specific folder
and each invocation gets its own run folder:

```text
basic_sim/llm_generated/prompt_1/run_001/
  generation.log
  user_prompt.txt
  json/
  xml/
  synthesized/
```

By default this reads prompts from:

```text
LAST_PROMPTS/user_prompts.txt
```

Generate outputs for every prompt in that file:

```bash
python3 basic_sim/llm_generate_ge.py \
  --all-user-prompts \
  --output-dir basic_sim/llm_generated
```

Preview the prompt without calling the API:

```bash
python3 basic_sim/llm_generate_ge.py --user-prompt-index 1 --print-prompt --dry-run
```

`--dry-run` does not write `G.json`, `E.json`, `G.xml`, or `E.xml`; it only prints the prompt.

Use `--skip-synthesis` if you want only the generated JSON/XML files and do
not want to create `Sloc.xml` or `Sloc.yaml`.

Generate `Sloc.xml` from the sample plant `G` and spec `E`:

```bash
python3 basic_sim/synthesize_sloc.py \
  --g basic_sim/sample_G.xml \
  --e basic_sim/sample_all_actions.xml \
  --output-dir basic_sim/synthesized
```

Run the LLM-generated synthesized supervisor:

```bash
python3 basic_sim/grid_sim.py \
  --automata basic_sim/llm_generated/prompt_1/run_001/synthesized/Sloc.xml \
  --agents 5
```

The same run also writes a compact SCT YAML file for the ROS robot supervisor:

```text
basic_sim/llm_generated/prompt_1/run_001/synthesized/Sloc.yaml
```

Use that YAML directly in the robot launch file:

```bash
ros2 launch swarm_basics spawn_multi_robots.launch.py \
  metadata_yaml_path:=/home/ecem/ros2_ws/src/Leo_sct/basic_sim/llm_generated/prompt_1/run_001/synthesized/Sloc.yaml
```

To also write the generated controller into `swarm_basics/config/`, pass
`--robot-config-output`:

```bash
python3 basic_sim/llm_generate_ge.py \
  --user-prompt-index 1 \
  --output-dir basic_sim/llm_generated \
  --robot-config-output swarm_basics/config/my_generated_controller.yaml
```

Then launch with:

```bash
ros2 launch swarm_basics spawn_multi_robots.launch.py \
  metadata_yaml_path:=/home/ecem/ros2_ws/src/Leo_sct/swarm_basics/config/my_generated_controller.yaml
```

Or pass it to one supervisor node:

```bash
ros2 run swarm_basics robot_supervisor \
  --ros-args -p supervisor_yaml_path:=/home/ecem/ros2_ws/src/Leo_sct/basic_sim/llm_generated/prompt_1/run_001/synthesized/Sloc.yaml
```

For real robots, keep the generated task limited to events that the current
`swarm_basics.robot_supervisor` can sense and execute. Supported perception
events are `path_clear`, `obstacle_front`, `obstacle_left`, and
`obstacle_right`. Supported actions are `move_forward`, `move_backward`,
`rotate_clockwise`, `rotate_counterclockwise`, `full_rotate`, and `stop`.
Generated red-area, blue-area, or object-pushing events are useful in
`basic_sim`, but the real robot supervisor will need matching sensor callbacks
and action handlers before those YAML files can drive robots correctly.

The synthesis helper uses Nadzoru from `~/Documents/Nadzoru2` by default. Pass
`--nadzoru-root /path/to/Nadzoru2` if needed.

## Event Mapping

The simulator emits one perception event per agent step:

- `obstacle_front`
- `path_clear`
- `red_detected`
- `blue_detected`
- `detect_object_red`

By default, `path_clear` means the cell directly in front of the agent is free.
Pass `--side-sensors` to also emit `obstacle_left` and `obstacle_right` when
side cells are blocked.

Then it chooses one enabled controllable event from the current automaton state:

- `move_forward`
- `move_backward`
- `rotate_clockwise`
- `rotate_counterclockwise`
- `full_rotate`
- `stop`
- `go_to_red`
- `escape_blue`
- `push`

`red_detected` is emitted when an agent is inside the 2x2 red area. `go_to_red`
moves the agent one grid cell toward the nearest red-area cell, or keeps it in
place if it has already arrived. `blue_detected` is emitted when an agent is
inside the 2x2 blue area. `escape_blue` moves the agent one grid cell away from
the blue area. `detect_object_red` is emitted when all four movable objects are
inside the red area. `push` moves toward a movable object; if an object is
directly in front, it pushes that object one cell forward in the robot's heading
and steps into the object's previous cell. If the object cannot move forward,
`push` repositions the robot around the object instead of repeatedly pushing it
into a wall. `stop` holds the agent in place for one step without translating or
rotating.

Multiple automata can be supplied. They are assigned to agents round-robin.

## Collision Counting

A collision is counted when an agent executes a movement event but the target
cell is blocked by:

- the grid boundary (`wall`)
- a configured obstacle (`obstacle`)
- another agent (`agent`)

Rotations and blocked perception events do not count as collisions by
themselves; the count increases only when a movement action tries to enter a
blocked cell.

The simulator writes a copy of terminal output to a per-run log under
`basic_sim/logs/` by default, for example
`basic_sim/logs/grid_sim_20260515_142233_123456.log`. Pass
`--log-dir path/to/logs` to choose another per-run log directory,
`--log-file path/to/log.txt` to choose one exact file, or `--no-log` to disable
file logging.

Collision logs include the action that caused the blocked movement and what was
hit, for example `move_backward->wall`, `move_forward->obstacle@(4, 2)`, or
`move_forward->agent_1`.

## Action Percentages

The simulator also tracks the percentage of controllable actions selected by
the automata. The final terminal summary reports overall percentages and one
breakdown per agent. In Matplotlib mode, the terminal step log and plot footer
also show the running overall action mix.
