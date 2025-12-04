#!/usr/bin/env python3
import json
import os
import time
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.monitor import Monitor


##############################################################
# 1. ENVIRONMENT
##############################################################

class SocialNavEnv(gym.Env):
    """
    Parameter-conditioned social navigation environment.

    train_mode=True:
        - Randomizes cfg["max_speed"], cfg["personal_space"],
          cfg["courtesy_weight"], and number of humans per episode.

    train_mode=False:
        - Uses cfg as provided (e.g. from LLM) without resampling.
    """
    metadata = {"render_modes": ["human"]}

    # max speed used for action space upper bound (constant)
    MAX_SPEED_TRAIN = 2.0
    GOAL_THRESHOLD = 0.5
    NEAR_GOAL_RADIUS = 0.7

    def __init__(self, config, train_mode=True, render_mode=None):
        super().__init__()
        # make a local copy so we don't mutate the caller's dict
        self.cfg = dict(config)
        self.train_mode = train_mode
        self.render_mode = render_mode

        # Max humans: fixes observation size
        # actual num_humans per episode will be <= max_humans
        self.max_humans = int(self.cfg.get("max_humans", self.cfg.get("num_humans", 4)))

        # We'll store the actual number of humans for this episode in self.num_humans
        self.num_humans = int(self.cfg.get("num_humans", self.max_humans))

        # Observation Space:
        # [robot_x, robot_y, goal_dx, goal_dy]
        # + max_humans * (dist, angle)
        # + 4 cfg parameters (normalized):
        #   [max_speed, personal_space, courtesy_weight, num_humans]
        self.obs_dim = 4 + self.max_humans * 2 + 4

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.obs_dim,),
            dtype=np.float32
        )

        # Action: [v, w], where v in [0, MAX_SPEED_TRAIN], w in [-1, 1]
        # We still clip v inside step() using self.cfg["max_speed"].
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([self.MAX_SPEED_TRAIN, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # Sim timestep
        self.dt = 0.1
        self.max_episode_steps = 400
        self.step_count = 0

        self.robot_pos = None
        self.robot_theta = None
        self.goal = None
        self.humans = None
        self.prev_dist_to_goal = None

        self.reset()

    def _sample_training_cfg(self):
        """
        Sample cfg parameters for training.
        Adjust ranges as you like.
        """
        # max_speed between real robot's 0.4 and training upper bound 2.0
        self.cfg["max_speed"] = float(np.random.uniform(0.4, 2.0))
        # personal space between 0.5m and 1.5m
        self.cfg["personal_space"] = float(np.random.uniform(0.5, 1.5))
        # courtesy weight between 0.5 and 4.0
        self.cfg["courtesy_weight"] = float(np.random.uniform(0.5, 4.0))
        # number of humans between 0 and max_humans (inclusive)
        self.num_humans = int(np.random.randint(0, self.max_humans + 1))

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        MIN_SPAWN_DIST = 1.0

        # Randomize cfg & num_humans if in training mode
        if self.train_mode:
            self._sample_training_cfg()
        else:
            # In test mode, use cfg["num_humans"] if provided, else default to max_humans
            self.num_humans = int(self.cfg.get("num_humans", self.max_humans))
            self.num_humans = max(0, min(self.num_humans, self.max_humans))

        # Robot initial pose
        self.robot_pos = np.array([0.0, 0.0], dtype=np.float32)
        self.robot_theta = 0.0

        # Random goal in a band [3, 5] meters away, with some lateral variation
        goal_x = np.random.uniform(3.0, 5.0)
        goal_y = np.random.uniform(-2.0, 2.0)
        self.goal = np.array([goal_x, goal_y], dtype=np.float32)

        # Humans
        self.humans = []
        for _ in range(self.num_humans):
            while True:
                pos = np.random.uniform(-2, 2, size=2)
                # ensure human not too close to robot
                if np.linalg.norm(pos - self.robot_pos) > MIN_SPAWN_DIST:
                    break
            vel = np.random.uniform(-0.2, 0.2, size=2)
            self.humans.append({"pos": pos, "vel": vel})

        self.prev_dist_to_goal = np.linalg.norm(self.goal - self.robot_pos)
        self.step_count = 0

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1

        v, w = action
        # clip by training space, then by current cfg
        v = float(np.clip(v, 0.0, self.MAX_SPEED_TRAIN))
        v = float(np.clip(v, 0.0, self.cfg["max_speed"]))
        w = float(np.clip(w, -1.0, 1.0))

        # Update robot
        self.robot_theta += w * self.dt
        dx = v * np.cos(self.robot_theta)
        dy = v * np.sin(self.robot_theta)
        self.robot_pos += np.array([dx, dy]) * self.dt

        # Update humans
        for h in self.humans:
            h["pos"] += h["vel"] * self.dt

        ###########################################################
        # REWARD SHAPING
        ###########################################################

        reward = 0.0

        # heading difference between robot direction and goal direction
        goal_vec = self.goal - self.robot_pos
        goal_dist = np.linalg.norm(goal_vec)
        goal_angle = np.arctan2(goal_vec[1], goal_vec[0])
        heading_raw = goal_angle - self.robot_theta
        heading_error = np.arctan2(np.sin(heading_raw), np.cos(heading_raw))  # [-pi,pi]

        # Distances
        dist = goal_dist
        progress = self.prev_dist_to_goal - dist

        # strong progress reward (telescoping sum over episode)
        reward += 25.0 * progress

        # heading alignment only when reasonably near
        if dist < 2.0:
            reward += 0.5 * np.cos(heading_error)

        # penalize moving away from goal
        if progress < 0:
            reward += 3.0 * progress  # progress < 0 → negative

        # penalize standing still / tiny movement
        if np.linalg.norm([dx, dy]) < 0.01:
            reward -= 0.05

        # mild global attractor + time penalty
        reward += 0.02 * (8.0 - dist)
        reward -= 0.005

        # Personal space
        if self.num_humans > 0:
            min_dist = min(
                np.linalg.norm(h["pos"] - self.robot_pos)
                for h in self.humans
            )
        else:
            min_dist = np.inf

        if min_dist < self.cfg["personal_space"]:
            violation = self.cfg["personal_space"] - min_dist
            proximity_scale = 1.5 if dist < 1.0 else 1.0
            reward -= self.cfg["courtesy_weight"] * violation * 0.5 * proximity_scale

        # Smoothness (prefer small angular velocity)
        reward += 0.05 * max(0.0, 0.1 - abs(w))

        # Collision
        if min_dist < 0.15:
            reward -= 300.0
            info = {"terminated_reason": "collision"}
            return self._get_obs(), reward, True, False, info

        # encourage entering inner circle / discourage lingering
        # if dist < self.NEAR_GOAL_RADIUS:
        #     reward += 4.0 * (self.NEAR_GOAL_RADIUS - dist)
        #     if progress < 1e-3:
        #         reward -= 0.1  # penalize hovering near the goal without committing
        #     linger_fraction = self.step_count / max(1, self.max_episode_steps)
        #     if linger_fraction > 0.5:
        #         reward -= 0.05 * (linger_fraction - 0.5)

        # Goal
        if dist < self.GOAL_THRESHOLD:
            reward += 100.0
            info = {"terminated_reason": "goal"}
            return self._get_obs(), reward, True, False, info

        # Timeout
        if self.step_count >= self.max_episode_steps:
            timeout_penalty = 30.0 * dist
            reward -= timeout_penalty
            if dist < self.NEAR_GOAL_RADIUS:
                reward -= 20.0 * (self.NEAR_GOAL_RADIUS - dist)
            info = {"terminated_reason": "timeout"}
            return self._get_obs(), reward, True, False, info

        ###########################################################
        self.prev_dist_to_goal = dist
        return self._get_obs(), reward, False, False, {}

    def _get_obs(self):
        dx, dy = self.goal - self.robot_pos
        obs = [self.robot_pos[0], self.robot_pos[1], dx, dy]

        # humans: (dist, angle) in robot frame (padded up to max_humans)
        # existing humans first
        for h in self.humans:
            rel = h["pos"] - self.robot_pos
            dist = np.linalg.norm(rel)
            angle = np.arctan2(rel[1], rel[0]) - self.robot_theta
            obs.extend([dist, angle])

        # pad remaining slots with "no human" (far away)
        for _ in range(self.max_humans - self.num_humans):
            obs.extend([10.0, 0.0])  # 10m away, angle 0

        # normalized cfg parameters
        max_speed_norm = self.cfg["max_speed"] / self.MAX_SPEED_TRAIN    # in [0.4/2, 1]
        personal_space_norm = self.cfg["personal_space"] / 1.5           # assume <= 1.5
        courtesy_norm = self.cfg["courtesy_weight"] / 4.0                # assume <= 4.0
        num_humans_norm = self.num_humans / max(1, self.max_humans)      # in [0,1]

        obs.extend([max_speed_norm, personal_space_norm, courtesy_norm, num_humans_norm])

        return np.array(obs, dtype=np.float32)

    def get_state_snapshot(self):
        """Used for visualization."""
        human_positions = np.array([h["pos"] for h in self.humans]) if self.num_humans > 0 else np.zeros((0, 2))
        return self.robot_pos.copy(), human_positions.copy(), self.goal.copy()


##############################################################
# 2. LLM CONFIG PLACEHOLDER
##############################################################

def llm_generate_config(user_text):
    print("User description:\n", user_text)
    # Default config for test-time.
    # During training, max_speed/personal_space/courtesy_weight/num_humans
    # are randomized inside the env if train_mode=True.
    return {
        "max_speed": 2.0,          # upper bound used for training range
        "personal_space": 1.0,
        "courtesy_weight": 2.0,
        "num_humans": 4,           # default test density
        "max_humans": 6            # maximum humans used to define obs size
    }


##############################################################
# 3. ROLLOUT + PLOTTING
##############################################################

def _get_base_env(env):
    """Return underlying env whether wrapped in Monitor or not."""
    return getattr(env, "env", env)

def rollout(model, env, max_steps=400):
    robot_traj = []
    human_traj = []
    obs, _ = env.reset()
    term_reason = "unknown"
    base_env = _get_base_env(env)

    for step in range(max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)

        rpos, hpos, goal = base_env.get_state_snapshot()
        robot_traj.append(rpos)
        human_traj.append(hpos)

        if done:
            term_reason = info.get("terminated_reason", "unknown")
            break

    print(f"[rollout] terminated because: {term_reason}")
    return np.array(robot_traj), np.array(human_traj), goal, term_reason


def plot_trajectory_to_file(robot_traj, human_traj, goal, save_path):
    plt.figure(figsize=(6, 6))

    # robot path
    plt.plot(robot_traj[:, 0], robot_traj[:, 1], '-o', markersize=2, label="Robot")

    # human paths
    if human_traj.size > 0:
        num_humans = human_traj.shape[1]
        for j in range(num_humans):
            plt.plot(
                human_traj[:, j, 0], human_traj[:, j, 1],
                '--', alpha=0.6, label=f"Human {j}"
            )

        # human starts
        for j in range(num_humans):
            plt.scatter(
                human_traj[0, j, 0], human_traj[0, j, 1],
                s=60, marker='x', label=f"Human {j} Start"
            )

    # start + goal
    plt.scatter(robot_traj[0, 0], robot_traj[0, 1],
                c='green', s=80, label="Robot Start")
    plt.scatter(goal[0], goal[1],
                c='red', s=80, label="Goal")

    plt.title("Robot + Human Trajectories")
    plt.legend(loc="upper right", bbox_to_anchor=(1.4, 1))
    plt.grid(True)
    plt.axis("equal")

    plt.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close()


##############################################################
# 4. MULTIPLE EPISODE TESTING
##############################################################

def rollout_collect(model, env, max_steps=400):
    robot_traj = []
    human_traj = []
    obs, _ = env.reset()
    term_reason = "unknown"
    base_env = _get_base_env(env)

    for step in range(max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)

        rpos, hpos, goal = base_env.get_state_snapshot()
        robot_traj.append(rpos)
        human_traj.append(hpos)

        if done:
            term_reason = info.get("terminated_reason", "unknown")
            break

    return np.array(robot_traj), np.array(human_traj), goal, term_reason


def run_multiple_tests(model, env, num_episodes=20, save_dir="eval_results"):
    os.makedirs(save_dir, exist_ok=True)

    for ep in range(num_episodes):
        print(f"\n=== Test Episode {ep} ===")
        robot_traj, human_traj, goal, reason = rollout_collect(model, env)
        print(f"Episode {ep} ended because: {reason}")

        save_path = os.path.join(save_dir, f"episode_{ep}.png")
        plot_trajectory_to_file(robot_traj, human_traj, goal, save_path)

    print(f"\nSaved all test episodes in: {save_dir}")

def generate_eval_parameter_sets(base_config, num_runs=20, seed=0):
    """
    Deterministically build a list of parameter overrides for evaluation.
    Keeps max_humans fixed to the training value so the observation size
    matches the saved policy.
    """
    max_humans = int(base_config.get("max_humans", 6))

    max_speed_values = np.linspace(0.5, base_config.get("max_speed", SocialNavEnv.MAX_SPEED_TRAIN), 5)
    personal_space_values = np.linspace(0.5, 1.5, 4)
    num_humans_values = list(range(0, max_humans + 1))

    scenarios = []
    for max_speed, personal_space, num_humans in itertools.product(
        max_speed_values, personal_space_values, num_humans_values
    ):
        cfg = dict(base_config)
        cfg["max_speed"] = float(np.round(max_speed, 3))
        cfg["personal_space"] = float(np.round(personal_space, 3))
        cfg["num_humans"] = int(min(num_humans, max_humans))
        cfg["max_humans"] = max_humans
        scenarios.append(cfg)

    if not scenarios:
        return []

    rng = np.random.default_rng(seed)
    order = rng.permutation(len(scenarios))
    selected = [scenarios[i] for i in order[:num_runs]]
    return selected    


def evaluate_saved_model(model_path, base_config, num_runs=20, save_root="test_results_param_sweep", seed=0):
    """
    Load a saved model and evaluate it over multiple parameter configurations.
    """
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Saved model not found at: {model_path}")

    model = SAC.load(model_path)
    scenarios = generate_eval_parameter_sets(base_config, num_runs=num_runs, seed=seed)

    os.makedirs(save_root, exist_ok=True)
    summary = []

    for idx, cfg in enumerate(scenarios):
        env = SocialNavEnv(cfg, train_mode=False)
        robot_traj, human_traj, goal, term_reason = rollout_collect(model, env)

        scenario_tag = (
            f"run_{idx:02d}_speed{cfg['max_speed']:.2f}_"
            f"ps{cfg['personal_space']:.2f}_hum{cfg['num_humans']}"
        )
        scenario_dir = os.path.join(save_root, scenario_tag)
        os.makedirs(scenario_dir, exist_ok=True)

        plot_path = os.path.join(scenario_dir, "trajectory.png")
        plot_trajectory_to_file(robot_traj, human_traj, goal, plot_path)

        summary.append(
            {
                "run_id": idx,
                "max_speed": cfg["max_speed"],
                "personal_space": cfg["personal_space"],
                "num_humans": cfg["num_humans"],
                "termination_reason": term_reason,
                "plot_path": os.path.relpath(plot_path, save_root),
            }
        )
        print(
            f"[Eval {idx+1}/{len(scenarios)}] "
            f"speed={cfg['max_speed']:.2f}, "
            f"personal_space={cfg['personal_space']:.2f}, "
            f"num_humans={cfg['num_humans']} -> {term_reason}"
        )

    summary_path = os.path.join(save_root, "summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"\nEvaluation summary written to: {summary_path}")


##############################################################
# 5. MAIN SCRIPT
##############################################################

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Train or evaluate the social navigation policy.")
    parser.add_argument("--mode", choices=["train", "test"], default="test", help="train a model or evaluate a saved one")
    parser.add_argument("--model-path", default="social_nav_policy.zip", help="path where the SAC model is saved/loaded")
    parser.add_argument("--init-policy", default=None, help="optional checkpoint whose policy weights initialize training")
    parser.add_argument("--timesteps", type=int, default=1_000_000, help="number of training timesteps")
    parser.add_argument("--eval-dir", default="test_results", help="directory to store evaluation plots")
    parser.add_argument("--eval-runs", type=int, default=20, help="number of parameterized evaluation runs")
    parser.add_argument("--eval-seed", type=int, default=0, help="random seed for shuffling eval configurations")
    args = parser.parse_args()

    user_description = """
    Crowded area with slow walkers. Robot should behave politely,
    keep distance, and avoid sharp movements.
    """

    base_config = llm_generate_config(user_description)

    if args.mode == "train":

        train_env = Monitor(SocialNavEnv(base_config, train_mode=True))
        run_name = time.strftime("finetune_%Y%m%d_%H%M%S")
        log_dir = os.path.join("./tensorboard_logs_sac", run_name)

        print("\nTraining SAC...\n")
        # model = PPO(
        #     "MlpPolicy",
        #     train_env,
        #     verbose=1,
        #     tensorboard_log="./tensorboard_logs/",
        #     learning_rate=3e-4,
        #     gamma=0.99,
        #     n_steps=2048,
        #     batch_size=64,
        #     n_epochs=20,
        #     clip_range=0.2,
        #     gae_lambda=0.95,
        #     ent_coef=0.0,
        #     vf_coef=0.5,
        # )

        model = SAC(
            "MlpPolicy",
            train_env,
            verbose=1,
            tensorboard_log=log_dir,
            learning_rate=1e-4,
            buffer_size=300_000,
            batch_size=256,
            gamma=0.99,
            tau=0.005,
            train_freq=64,
            gradient_steps=64,
            ent_coef="auto",
        )

        if args.init_policy:
            if not os.path.exists(args.init_policy):
                raise FileNotFoundError(f"Init policy '{args.init_policy}' not found")
            pretrained = SAC.load(args.init_policy)
            model.policy.load_state_dict(pretrained.policy.state_dict())
            print(f"Loaded initial policy weights from {args.init_policy}")

        model.learn(total_timesteps=args.timesteps)
        model.save(args.model_path)

    
        test_env = SocialNavEnv(base_config, train_mode=False)
        run_multiple_tests(model, test_env, num_episodes=20, save_dir=args.eval_dir)
    else:
        evaluate_saved_model(
            model_path=args.model_path,
            base_config=base_config,
            num_runs=args.eval_runs,
            save_root=args.eval_dir,
        )
