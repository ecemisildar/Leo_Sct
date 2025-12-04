#!/usr/bin/env python3
"""
Multi-robot social navigation environment + MADDPG trainer.

Adds 4 more robots (total 5) that head to the same goal, penalizing
collisions with humans and between robots. The script implements a
minimal MADDPG loop so you can fine-tune or experiment with cooperative
multi-agent behavior.
"""
import argparse
import math
import os
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


def _mlp(input_dim: int, output_dim: int, hidden: Tuple[int, ...] = (128, 128), final_act=None):
    layers: List[nn.Module] = []
    last = input_dim
    for h in hidden:
        layers.extend([nn.Linear(last, h), nn.ReLU()])
        last = h
    layers.append(nn.Linear(last, output_dim))
    if final_act is not None:
        layers.append(final_act)
    return nn.Sequential(*layers)


class MultiRobotSocialNavEnv:
    """Simplified multi-robot navigation environment for MADDPG."""

    MAX_SPEED = 2.0
    GOAL_THRESHOLD = 0.3
    ROBOT_COLLISION_RADIUS = 0.35

    def __init__(self, config: Dict, num_robots: int = 5, max_humans: int = 4):
        self.cfg = dict(config)
        self.num_agents = num_robots
        self.agent_ids = [f"robot_{i}" for i in range(num_robots)]
        self.max_humans = max_humans
        self.dt = 0.1
        self.max_episode_steps = 400

        # State
        self.goal = np.zeros(2, dtype=np.float32)
        self.robot_pos = np.zeros((num_robots, 2), dtype=np.float32)
        self.robot_theta = np.zeros(num_robots, dtype=np.float32)
        self.humans: List[Dict[str, np.ndarray]] = []
        self.step_count = 0

        # Observation components
        self.single_obs_dim = 2 + 2 + 2  # robot position, goal delta, heading (cos,sin)
        self.single_obs_dim += 2 * (self.num_agents - 1)  # relative other robots
        self.single_obs_dim += 2 * self.max_humans  # humans dist+angle
        self.single_obs_dim += 2  # speed + courtesy weight
        self.action_dim = 2

    def reset(self):
        self.step_count = 0
        self.goal = np.array(
            [np.random.uniform(3.0, 5.0), np.random.uniform(-2.0, 2.0)], dtype=np.float32
        )
        # Spawn robots around origin, spaced evenly on a circle
        radius = 0.5
        for i in range(self.num_agents):
            angle = 2 * math.pi * i / self.num_agents
            self.robot_pos[i] = np.array([radius * math.cos(angle), radius * math.sin(angle)], dtype=np.float32)
            self.robot_theta[i] = 0.0

        # Humans
        self.humans = []
        num_humans = int(self.cfg.get("num_humans", self.max_humans))
        for _ in range(num_humans):
            pos = np.random.uniform(-2.0, 2.0, size=2)
            vel = np.random.uniform(-0.2, 0.2, size=2)
            self.humans.append({"pos": pos.astype(np.float32), "vel": vel.astype(np.float32)})

        obs = {agent: self._get_obs(idx) for idx, agent in enumerate(self.agent_ids)}
        return obs

    def step(self, actions: Dict[str, np.ndarray]):
        self.step_count += 1
        info: Dict[str, Dict] = {agent: {} for agent in self.agent_ids}

        # Update robots
        for idx, agent in enumerate(self.agent_ids):
            v, w = np.clip(actions[agent], [0.0, -1.0], [self.MAX_SPEED, 1.0])
            self.robot_theta[idx] += w * self.dt
            dx = v * math.cos(self.robot_theta[idx])
            dy = v * math.sin(self.robot_theta[idx])
            self.robot_pos[idx] += np.array([dx, dy], dtype=np.float32) * self.dt

        # Update humans
        for human in self.humans:
            human["pos"] += human["vel"] * self.dt

        rewards = {}
        dones = {}
        truncs = {}

        # pairwise robot collision check
        pairwise_penalties = np.zeros(self.num_agents, dtype=np.float32)
        for i in range(self.num_agents):
            for j in range(i + 1, self.num_agents):
                dist = np.linalg.norm(self.robot_pos[i] - self.robot_pos[j])
                if dist < self.ROBOT_COLLISION_RADIUS:
                    penalty = -300.0
                    pairwise_penalties[i] += penalty
                    pairwise_penalties[j] += penalty
                    info[self.agent_ids[i]]["terminated_reason"] = "robot_collision"
                    info[self.agent_ids[j]]["terminated_reason"] = "robot_collision"

        # Compute rewards
        for idx, agent in enumerate(self.agent_ids):
            reward, done = self._compute_agent_reward(idx, pairwise_penalties[idx], info[agent])
            rewards[agent] = reward
            dones[agent] = done
            truncs[agent] = False

        all_done = all(dones.values()) or self.step_count >= self.max_episode_steps
        if self.step_count >= self.max_episode_steps:
            for agent in self.agent_ids:
                info[agent]["terminated_reason"] = info[agent].get("terminated_reason", "timeout")
                truncs[agent] = True

        dones["__all__"] = all_done
        truncs["__all__"] = self.step_count >= self.max_episode_steps
        obs = {agent: self._get_obs(i) for i, agent in enumerate(self.agent_ids)}
        return obs, rewards, dones, truncs, info

    def _compute_agent_reward(self, idx: int, pairwise_penalty: float, info: Dict) -> Tuple[float, bool]:
        pos = self.robot_pos[idx]
        theta = self.robot_theta[idx]

        goal_vec = self.goal - pos
        goal_dist = np.linalg.norm(goal_vec)
        progress = -goal_dist  # distance based
        reward = 2.0 * (-goal_dist)

        # Head toward goal
        goal_angle = math.atan2(goal_vec[1], goal_vec[0])
        heading_error = math.atan2(math.sin(goal_angle - theta), math.cos(goal_angle - theta))
        reward += 0.5 * math.cos(heading_error)

        # Human proximity
        min_human = np.inf
        if self.humans:
            for human in self.humans:
                dist = np.linalg.norm(human["pos"] - pos)
                min_human = min(min_human, dist)
        if min_human < self.cfg.get("personal_space", 1.0):
            reward -= 150.0 * (self.cfg["personal_space"] - min_human)
            info["terminated_reason"] = info.get("terminated_reason", "human_violation")

        reward += pairwise_penalty

        # Success check
        if goal_dist < self.GOAL_THRESHOLD:
            reward += 200.0
            info["terminated_reason"] = "goal"
            return reward, True

        # Small penalty per step
        reward -= 0.01
        return reward, False

    def _get_obs(self, idx: int):
        pos = self.robot_pos[idx]
        theta = self.robot_theta[idx]
        obs: List[float] = []
        obs.extend(pos.tolist())
        goal_vec = self.goal - pos
        obs.extend(goal_vec.tolist())
        obs.extend([math.cos(theta), math.sin(theta)])

        for j in range(self.num_agents):
            if j == idx:
                continue
            rel = self.robot_pos[j] - pos
            obs.extend(rel.tolist())

        for h_idx in range(self.max_humans):
            if h_idx < len(self.humans):
                rel = self.humans[h_idx]["pos"] - pos
                dist = np.linalg.norm(rel)
                angle = math.atan2(rel[1], rel[0]) - theta
                obs.extend([dist, angle])
            else:
                obs.extend([10.0, 0.0])

        obs.extend([self.cfg.get("max_speed", self.MAX_SPEED) / self.MAX_SPEED])
        obs.extend([self.cfg.get("courtesy_weight", 2.0) / 4.0])

        return np.array(obs, dtype=np.float32)


@dataclass
class Agent:
    actor: nn.Module
    actor_target: nn.Module
    actor_opt: optim.Optimizer
    critic: nn.Module
    critic_target: nn.Module
    critic_opt: optim.Optimizer


class ReplayBuffer:
    def __init__(self, capacity: int, num_agents: int, obs_dim: int, act_dim: int):
        self.capacity = capacity
        self.num_agents = num_agents
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.ptr = 0
        self.size = 0

        self.obs = np.zeros((capacity, num_agents, obs_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, num_agents, act_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, num_agents), dtype=np.float32)
        self.next_obs = np.zeros((capacity, num_agents, obs_dim), dtype=np.float32)
        self.dones = np.zeros((capacity, num_agents), dtype=np.float32)

    def add(self, obs, actions, rewards, next_obs, dones):
        self.obs[self.ptr] = obs
        self.actions[self.ptr] = actions
        self.rewards[self.ptr] = rewards
        self.next_obs[self.ptr] = next_obs
        self.dones[self.ptr] = dones

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        idxs = np.random.randint(0, self.size, size=batch_size)
        return (
            torch.tensor(self.obs[idxs]),
            torch.tensor(self.actions[idxs]),
            torch.tensor(self.rewards[idxs]),
            torch.tensor(self.next_obs[idxs]),
            torch.tensor(self.dones[idxs]),
        )


class MADDPG:
    def __init__(self, env: MultiRobotSocialNavEnv, lr=1e-3, gamma=0.95, tau=0.01, buffer_size=200000, batch_size=256):
        self.env = env
        self.num_agents = env.num_agents
        self.obs_dim = env.single_obs_dim
        self.act_dim = env.action_dim
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size

        self.buffer = ReplayBuffer(buffer_size, self.num_agents, self.obs_dim, self.act_dim)
        self.agents: List[Agent] = []

        for _ in range(self.num_agents):
            actor = _mlp(self.obs_dim, self.act_dim, final_act=nn.Tanh())
            actor_target = _mlp(self.obs_dim, self.act_dim, final_act=nn.Tanh())
            actor_target.load_state_dict(actor.state_dict())
            actor_opt = optim.Adam(actor.parameters(), lr=lr)

            critic_input_dim = self.obs_dim * self.num_agents + self.act_dim * self.num_agents
            critic = _mlp(critic_input_dim, 1)
            critic_target = _mlp(critic_input_dim, 1)
            critic_target.load_state_dict(critic.state_dict())
            critic_opt = optim.Adam(critic.parameters(), lr=lr)

            self.agents.append(
                Agent(actor=actor, actor_target=actor_target, actor_opt=actor_opt,
                      critic=critic, critic_target=critic_target, critic_opt=critic_opt)
            )

    def select_actions(self, obs_dict: Dict[str, np.ndarray], noise_scale=0.1) -> Dict[str, np.ndarray]:
        actions = {}
        for idx, agent_id in enumerate(self.env.agent_ids):
            obs = torch.tensor(obs_dict[agent_id], dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                action = self.agents[idx].actor(obs).squeeze(0).numpy()
            # map tanh output (-1,1) to [0,1] for v and [-1,1] for w
            v = (action[0] + 1.0) * 0.5 * self.env.MAX_SPEED
            w = action[1]
            noisy = np.array([v, w], dtype=np.float32)
            noisy += noise_scale * np.random.randn(2)
            noisy[0] = np.clip(noisy[0], 0.0, self.env.MAX_SPEED)
            noisy[1] = np.clip(noisy[1], -1.0, 1.0)
            actions[agent_id] = noisy
        return actions

    def store(self, obs, actions, rewards, next_obs, dones):
        obs_arr = np.array([obs[agent] for agent in self.env.agent_ids], dtype=np.float32)
        next_arr = np.array([next_obs[agent] for agent in self.env.agent_ids], dtype=np.float32)
        act_arr = np.array([actions[agent] for agent in self.env.agent_ids], dtype=np.float32)
        rew_arr = np.array([rewards[agent] for agent in self.env.agent_ids], dtype=np.float32)
        done_arr = np.array([float(dones[agent]) for agent in self.env.agent_ids], dtype=np.float32)
        self.buffer.add(obs_arr, act_arr, rew_arr, next_arr, done_arr)

    def update(self):
        if self.buffer.size < self.batch_size:
            return
        obs_b, act_b, rew_b, next_obs_b, done_b = self.buffer.sample(self.batch_size)

        obs_b = obs_b.float()
        act_b = act_b.float()
        rew_b = rew_b.float()
        next_obs_b = next_obs_b.float()
        done_b = done_b.float()

        # Concatenate for critics
        obs_cat = obs_b.reshape(self.batch_size, -1)
        act_cat = act_b.reshape(self.batch_size, -1)
        next_obs_cat = next_obs_b.reshape(self.batch_size, -1)

        with torch.no_grad():
            next_actions = []
            for agent, next_obs in zip(self.agents, next_obs_b.transpose(0, 1)):
                next_actions.append(agent.actor_target(next_obs))
            next_act_cat = torch.cat(next_actions, dim=-1)

        for agent_idx, agent in enumerate(self.agents):
            # Critic update
            target_q = agent.critic_target(torch.cat([next_obs_cat, next_act_cat], dim=-1))
            target = rew_b[:, agent_idx:agent_idx+1] + self.gamma * (1 - done_b[:, agent_idx:agent_idx+1]) * target_q
            current_q = agent.critic(torch.cat([obs_cat, act_cat], dim=-1))
            critic_loss = nn.MSELoss()(current_q, target)
            agent.critic_opt.zero_grad()
            critic_loss.backward()
            agent.critic_opt.step()

            # Actor update (policy gradient)
            curr_actions = []
            for other_idx, other_agent in enumerate(self.agents):
                obs_slice = obs_b[:, other_idx, :]
                if other_idx == agent_idx:
                    curr_actions.append(other_agent.actor(obs_slice))
                else:
                    with torch.no_grad():
                        curr_actions.append(other_agent.actor(obs_slice))
            act_stack = torch.cat(curr_actions, dim=-1)
            actor_loss = -agent.critic(torch.cat([obs_cat, act_stack], dim=-1)).mean()
            agent.actor_opt.zero_grad()
            actor_loss.backward()
            agent.actor_opt.step()

            self._soft_update(agent.actor_target, agent.actor)
            self._soft_update(agent.critic_target, agent.critic)

    def _soft_update(self, target, source):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - self.tau) + param.data * self.tau)

    def save(self, path: str):
        payload = {
            "agents": [
                {
                    "actor": agent.actor.state_dict(),
                    "critic": agent.critic.state_dict(),
                }
                for agent in self.agents
            ]
        }
        torch.save(payload, path)

    def load(self, path: str):
        payload = torch.load(path, map_location="cpu")
        for agent, saved in zip(self.agents, payload["agents"]):
            agent.actor.load_state_dict(saved["actor"])
            agent.actor_target.load_state_dict(saved["actor"])
            agent.critic.load_state_dict(saved["critic"])
            agent.critic_target.load_state_dict(saved["critic"])


def train(args):
    base_cfg = {
        "max_speed": 2.0,
        "personal_space": 1.0,
        "courtesy_weight": 2.0,
        "num_humans": 4,
    }
    env = MultiRobotSocialNavEnv(base_cfg, num_robots=5, max_humans=4)
    maddpg = MADDPG(env, lr=args.lr, gamma=args.gamma, tau=args.tau, buffer_size=args.buffer, batch_size=args.batch)
    if args.checkpoint and os.path.exists(args.checkpoint):
        maddpg.load(args.checkpoint)
        print(f"Loaded MADDPG checkpoint from {args.checkpoint}")

    for episode in range(args.episodes):
        obs = env.reset()
        episode_reward = 0.0
        done = {"__all__": False}
        while not done["__all__"]:
            actions = maddpg.select_actions(obs, noise_scale=args.noise)
            next_obs, rewards, dones, truncs, info = env.step(actions)
            maddpg.store(obs, actions, rewards, next_obs, dones)
            maddpg.update()
            obs = next_obs
            episode_reward += sum(rewards.values())
            done = dones
        if (episode + 1) % args.log_interval == 0:
            print(f"[Episode {episode+1}] reward={episode_reward:.2f} steps={env.step_count}")
        if args.checkpoint and (episode + 1) % args.save_interval == 0:
            maddpg.save(args.checkpoint)
    if args.checkpoint:
        maddpg.save(args.checkpoint)


def main():
    parser = argparse.ArgumentParser(description="MADDPG training for 5 cooperative robots.")
    parser.add_argument("--episodes", type=int, default=1000, help="training episodes")
    parser.add_argument("--lr", type=float, default=1e-3, help="learning rate")
    parser.add_argument("--gamma", type=float, default=0.95, help="discount factor")
    parser.add_argument("--tau", type=float, default=0.01, help="target update rate")
    parser.add_argument("--buffer", type=int, default=200000, help="replay buffer size")
    parser.add_argument("--batch", type=int, default=256, help="batch size")
    parser.add_argument("--noise", type=float, default=0.1, help="exploration noise std")
    parser.add_argument("--checkpoint", type=str, default="maddpg_multi_robot.pt", help="path to save/load model")
    parser.add_argument("--log-interval", type=int, default=10)
    parser.add_argument("--save-interval", type=int, default=50)
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
