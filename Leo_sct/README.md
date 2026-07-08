# Leo SCT

ROS 2 workspace packages and scripts for running Leo Rover swarm simulations with
supervisory-control policies generated from LLM/Nadzoru pipeline outputs.

The main runtime path is:

1. Build the ROS 2 workspace with `colcon`.
2. Launch Gazebo through `swarm_basics`.
3. Run the supervisors against a YAML policy in `Leo_sct/swarm_basics/config`.
4. Optionally generate new policy YAML files with `Leo_sct/llm-part/run_pipeline.py`.

## Repository Layout

- `Leo_sct/swarm_basics/`: ROS 2 Python package for multi-robot launch files,
  robot supervisor nodes, coverage counting, and bump counting.
- `Leo_sct/leo_image/`: ROS 2 C++ depth/image processing node used by each robot.
- `Leo_sct/leo_common-ros2/`: Leo Rover description, messages, and common packages.
- `Leo_sct/leo_simulator-ros2/`: Gazebo simulation packages for Leo Rover.
- `Leo_sct/llm-part/`: LLM -> JSON -> Nadzoru XML -> SCT YAML generation pipeline.
- `Leo_sct/website/`: browser control UI backed by ROS bridge.
- `Leo_sct/video_analyze/`: offline video analysis scripts.
- `Leo_sct/sim_recordings/`: saved simulation runs and logs.

## Prerequisites

This project is intended for ROS 2 with Gazebo and the Leo Rover ROS packages.
The simulator README notes support for ROS 2 Humble/Iron and Gazebo
Fortress/Garden/Harmonic.

Expected tools:

- ROS 2 sourced in the shell, for example `/opt/ros/humble/setup.bash`.
- Gazebo / `ros_gz` packages matching the ROS 2 distribution.
- `colcon`, `rosdep`, and standard ROS build tools.
- Python packages used by the LLM pipeline: `requests` and `PyYAML`.
- Optional for LLM policy generation: `OPENAI_API_KEY` or `Leo_sct/api_key.txt`.
- Optional for Nadzoru conversion: Nadzoru2 at `~/Documents/Nadzoru2`.

## Build

From the workspace root:

```bash
cd /home/ecem/sct_llm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you use a different ROS 2 distribution, replace `humble` with that distro.

## Run the Swarm Simulation

Launch the default headless Gazebo simulation with 10 robots:

```bash
cd /home/ecem/sct_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch swarm_basics leo_gz.launch.py
```

Common launch arguments:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=5 \
  run_duration:=200.0 \
  spawn_layout:=spread \
  headless:=true \
  auto_start_supervisor:=true
```

Useful values:

- `total_robots`: number of robots to spawn. Default: `10`.
- `run_duration`: seconds before launch shuts down. Default: `200.0`.
- `spawn_layout`: `spread`, `middle_circle`, or `random_safe`.
- `headless`: `true` runs Gazebo without the GUI; use `false` to see the world.
- `metadata_yaml_path`: supervisor YAML to load. Default is the installed
  `swarm_basics/config/sup_gpt.yaml`.
- `results_dir`: directory for run artifacts. Default is
  `~/ros2_ws/src/Leo_sct/results_exp`.
- `prompt_text` or `prompt_file_path`: text copied into the run folder as
  `prompt.txt`.

Example with a specific supervisor YAML:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=5 \
  metadata_yaml_path:=/home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/swarm_basics/config/sup_gpt.yaml \
  results_dir:=/home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/results_exp
```

During launch, `swarm_basics` starts Gazebo, spawns the robots, bridges Gazebo
topics, starts one `robot_supervisor` per robot, and starts coverage/bump
counters.

## Generate or Reuse SCT YAML Policies

The LLM/Nadzoru pipeline lives in `Leo_sct/llm-part`.

Run a fresh LLM generation for the `explore` task:

```bash
cd /home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/llm-part
python3 run_pipeline.py --task explore --run-llm
```

Reuse the latest JSON for a task and regenerate downstream artifacts:

```bash
python3 run_pipeline.py --task find_marker --skip-llm
```

Grouped experiment example:

```bash
python3 run_pipeline.py \
  --task explore \
  --run-llm \
  --prompt-set-file prompt_groups_explore.txt \
  --prompt-repeats 3 \
  --model gpt-4.1
```

Supported task names are defined in `Leo_sct/llm-part/task_profiles.json`.

Pipeline outputs include generated JSON, Nadzoru XML, and YAML files. The latest
runtime YAML is copied to `Leo_sct/swarm_basics/config/sup_gpt.yaml`.

More details are in `Leo_sct/llm-part/README_supervisor_api.md`.

## Run the Web Controller

Start ROS bridge:

```bash
ros2 run rosbridge_server rosbridge_websocket
```

Start the web server from the project source directory:

```bash
cd /home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct
python3 website/server.py --host 0.0.0.0 --port 8080
```

Open:

```text
http://localhost:8080
```

Robot namespaces are configured in `Leo_sct/website/robots.json`.

## Useful Direct Node Commands

Run an individual supervisor after the workspace is built and sourced:

```bash
ros2 run swarm_basics robot_supervisor
```

Run coverage or bump counters directly:

```bash
ros2 run swarm_basics coverage_counter
ros2 run swarm_basics bump_counter
```

Most normal runs should use the launch file instead, because it wires all robot
namespaces, bridges, timers, and result paths together.

## Troubleshooting

- If `ros2 launch` cannot find `swarm_basics`, source
  `/opt/ros/<distro>/setup.bash` and the workspace `install/setup.bash`.
- If Gazebo assets are missing, rebuild and source the workspace; the launch file
  sets `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH` from installed
  package paths.
- If a supervisor YAML is not found, pass `metadata_yaml_path:=/absolute/path/to/file.yaml`.
- If `run_pipeline.py` cannot import Nadzoru modules, check that Nadzoru2 exists
  at `~/Documents/Nadzoru2` or update `DEFAULT_NADZORU_ROOT` in the script.
- If LLM generation fails immediately, set `OPENAI_API_KEY` or create
  `Leo_sct/api_key.txt`.




SPAWN_SEED=$(date +%s)
RANDOM_SEED=$((SPAWN_SEED + 1000))

ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=5 \
  run_duration:=300.0 \
  spawn_layout:=random_safe \
  spawn_seed:=$SPAWN_SEED \
  random_seed:=$RANDOM_SEED \
  sct_choice_mode:=random