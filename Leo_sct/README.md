# Leo SCT

ROS 2 workspace packages and scripts for running Leo Rover swarm simulations in
Gazebo, visualizing them in RViz, and evaluating SCT or random-walk/CBF
controllers.

The main runtime path is:

1. Build the workspace with `colcon`.
2. Launch Gazebo with `swarm_basics`.
3. Visualize the robot TF/model/camera topics in RViz.
4. Run either the SCT supervisor or the random-walk/CBF controller.
5. Inspect run artifacts in `Leo_sct/results_exp`.

## Repository Layout

- `swarm_basics/`: Main ROS 2 Python package. Contains Gazebo launch files,
  multi-robot spawning, SCT supervisor, random-walk/CBF controller entry point,
  proximity warning helpers, coverage counting, bump counting, worlds, config,
  and RViz config.
- `leo_image/`: ROS 2 C++ depth image processor. Converts the depth camera stream
  into obstacle-zone topics used by the controllers.
- `leo_common-ros2/`: Leo Rover description, messages, teleop, and common robot
  packages.
- `leo_simulator-ros2/`: Gazebo simulation support packages for Leo Rover.
- `llm-part/`: LLM to JSON to Nadzoru XML to SCT YAML generation pipeline.
- `results_exp/`: New simulation run artifacts.
- `sim_recordings/`: Saved simulation recordings and logs.

## Prerequisites

- ROS 2 sourced in the shell, for example `/opt/ros/humble/setup.bash`.
- Gazebo and `ros_gz` / `ros_ign` packages matching your ROS 2 distribution.
- `colcon`, `rosdep`, and standard ROS build tools.
- Python packages used by the LLM pipeline, including `requests` and `PyYAML`.
- Optional for LLM policy generation: `OPENAI_API_KEY` or `Leo_sct/api_key.txt`.
- Optional for Nadzoru conversion: Nadzoru2 at `~/Documents/Nadzoru2`.

Replace `humble` below if you use another ROS 2 distribution.

## Build

From the workspace root:

```bash
cd /home/ecem/sct_llm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

After code or launch changes, rebuild at least the affected packages:

```bash
cd /home/ecem/sct_llm_ws
colcon build --packages-select swarm_basics leo_image leo_description
source install/setup.bash
```

## Run Gazebo

Start the default simulation:

```bash
cd /home/ecem/sct_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch swarm_basics leo_gz.launch.py
```

Run three robots with the Gazebo GUI:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=3 \
  headless:=false
```

Run three robots with the SCT controller:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=3 \
  controller_mode:=sct \
  auto_start_supervisor:=true \
  headless:=false
```

Run three robots with random-walk/CBF instead of SCT:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=3 \
  controller_mode:=random_walk_cbf \
  auto_start_supervisor:=true \
  headless:=false
```

Launch arguments commonly used:

- `total_robots`: number of robots. Default: `1`.
- `run_duration`: seconds before shutdown. Default: `300.0`.
- `headless`: `true` runs Gazebo without GUI; `false` opens Gazebo.
- `auto_start`: start Gazebo physics immediately. Default: `true`.
- `spawn_layout`: `spread`, `origin`, `middle_circle`, or `random_safe`.
- `spawn_seed`: seed for randomized spawn layouts. Default: `auto`.
- `random_seed`: base seed for per-robot controller choices. Default: `auto`.
- `controller_mode`: `sct` or `random_walk_cbf`. Default: `sct`.
- `sct_choice_mode`: SCT controllable event choice mode, `random` or `first`.
- `auto_start_supervisor`: enables the selected controller on launch.
- `metadata_yaml_path`: SCT YAML file. Default: `swarm_basics/config/sup_gpt.yaml`.
- `results_dir`: run artifact directory. Default: `Leo_sct/results_exp`.
- `prompt_text` or `prompt_file_path`: copied into the run folder as `prompt.txt`.

Example with a specific SCT YAML:

```bash
ros2 launch swarm_basics leo_gz.launch.py \
  total_robots:=5 \
  controller_mode:=sct \
  metadata_yaml_path:=/home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/swarm_basics/config/sup_gpt.yaml \
  results_dir:=/home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/results_exp
```

## Run RViz

The RViz config is installed from `swarm_basics/rviz/swarm_3_robots.rviz`.

After Gazebo is running, start RViz with:

```bash
cd /home/ecem/sct_llm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d /home/ecem/sct_llm_ws/src/Leo_Sct/Leo_sct/swarm_basics/rviz/swarm_3_robots.rviz \
  --ros-args -p use_sim_time:=true
```

The config uses:

- Fixed frame: `odom`
- Robot description topics:
  - `/robot_0/robot_description`
  - `/robot_1/robot_description`
  - `/robot_2/robot_description`
- RGB image topic: `/robot_0/depth_camera/image`

If you add RobotModel displays manually, use **Description Source = Topic** and
set the topic to the matching robot description. The default `/robot_description`
topic is not used by the multi-robot launch.

Useful checks while Gazebo is running:

```bash
ros2 topic echo /tf --once
ros2 topic echo /robot_0/joint_states --once
ros2 topic echo /robot_0/odom --once
ros2 topic list | grep robot_0
```

Expected TF shape:

```text
odom -> robot_0/odom -> robot_0/base_footprint -> robot_0/base_link -> wheels/camera
odom -> robot_1/odom -> robot_1/base_footprint -> ...
```

## Controller and Sensor Topics

Each robot namespace uses topics like:

- `/robot_i/cmd_vel`
- `/robot_i/odom`
- `/robot_i/tf`
- `/robot_i/joint_states`
- `/robot_i/depth_camera/image`
- `/robot_i/depth_camera/depth_image`
- `/robot_i/depth_camera/camera_info`
- `/robot_i/detected_zones`
- `/robot_i/front_obstacle_distance`

`leo_image` provides the depth-zone detector. `swarm_basics` launches it per
robot and feeds its outputs to either the SCT supervisor or random-walk/CBF
controller.

## Generate or Reuse SCT YAML Policies

The LLM/Nadzoru pipeline lives in `llm-part`.

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

Supported task names are defined in `llm-part/task_profiles.json`. Pipeline
outputs include generated JSON, Nadzoru XML, and YAML files. The latest runtime
YAML is copied to `swarm_basics/config/sup_gpt.yaml`.

More details are in `llm-part/README_supervisor_api.md`.

## Direct Node Commands

Most normal runs should use `leo_gz.launch.py`, because it wires namespaces,
bridges, TF, timers, and result paths together.

Direct commands after the workspace is built and sourced:

```bash
ros2 run swarm_basics robot_supervisor
ros2 run swarm_basics random_walk_cbf_controller
ros2 run swarm_basics robot_proximity_warner
ros2 run swarm_basics robot_id_warning_relay
ros2 run swarm_basics coverage_counter
ros2 run swarm_basics bump_counter
ros2 run leo_image image_processor
```

## Troubleshooting

- If `ros2 launch` cannot find `swarm_basics`, source both
  `/opt/ros/<distro>/setup.bash` and `/home/ecem/sct_llm_ws/install/setup.bash`.
- If Gazebo assets are missing, rebuild and source the workspace. The launch sets
  `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH` from package paths.
- If RViz does not show robots, set fixed frame to `odom` and use RobotModel
  description topics `/robot_0/robot_description`, `/robot_1/robot_description`,
  and `/robot_2/robot_description`.
- If all RViz robots appear at the origin, check `/tf` for
  `odom -> robot_i/odom` and `robot_i/odom -> robot_i/base_footprint`.
- If wheels or rockers are missing in RViz, check `/robot_i/joint_states`.
- If the camera image is black, display `/robot_i/depth_camera/image` for RGB.
  `/robot_i/depth_camera/depth_image` is depth data and may look black in a
  normal Image display.
- If a supervisor YAML is not found, pass
  `metadata_yaml_path:=/absolute/path/to/file.yaml`.
- If `run_pipeline.py` cannot import Nadzoru modules, check that Nadzoru2 exists
  at `~/Documents/Nadzoru2` or update `DEFAULT_NADZORU_ROOT`.
- If LLM generation fails immediately, set `OPENAI_API_KEY` or create
  `Leo_sct/api_key.txt`.
