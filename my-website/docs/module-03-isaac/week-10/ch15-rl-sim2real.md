---
id: ch15-rl-sim2real
title: Reinforcement Learning and Sim-to-Real
sidebar_label: Reinforcement Learning...
sidebar_position: 16
---

# Reinforcement Learning and Sim-to-Real

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand reinforcement learning fundamentals (PPO, SAC algorithms)
- Train robot policies in Isaac Sim using IsaacGymEnvs
- Apply domain randomization for sim-to-real transfer
- Deploy trained policies to real robots
- Analyze and bridge the reality gap

## Introduction

The **sim-to-real problem** is one of the grand challenges in robotics: policies trained in simulation often fail catastrophically when deployed to real robots. This **reality gap** arises from differences in physics (friction, contact dynamics), sensors (noise, latency), and environment (lighting, textures). For example, a quadruped robot that walks perfectly in Isaac Sim might fall immediately on real hardware due to unmodeled actuator delays or ground compliance.

Reinforcement learning (RL) is particularly dependent on simulation because learning requires millions of environment interactions—far too many to collect on real hardware. A single RL training run for quadruped locomotion might require 100 million steps, equivalent to months of continuous real-world operation. Simulation enables **massively parallel training** (thousands of robots in parallel), **safe exploration** (no hardware damage), and **rapid iteration** (reset to any state instantly).

Successful sim-to-real examples demonstrate what's possible: Boston Dynamics' quadrupeds learned agile locomotion in simulation before real-world deployment, OpenAI's Dactyl robot learned in-hand manipulation with domain randomization, and Agility Robotics' Digit humanoid uses RL-trained policies for dynamic walking. In this chapter, we'll explore the foundations of RL, train policies in Isaac Sim, and apply proven techniques for sim-to-real transfer.

## Reinforcement Learning Fundamentals

Reinforcement learning frames robot control as a **Markov Decision Process (MDP)**, defined by:

- **State (s)**: Current robot configuration and environment observations (e.g., joint positions, velocities, LiDAR scans)
- **Action (a)**: Control commands (e.g., joint torques, velocities)
- **Reward (r)**: Scalar feedback signal (+1 for reaching goal, -0.1 for falling)
- **Transition (s' ~ P(s'|s,a))**: Next state after taking action in current state

The goal is to learn a **policy π(a|s)** that maximizes cumulative reward: **E[Σ γ^t r_t]**, where γ is the discount factor.

### Policy Gradient Methods

Modern RL for robotics uses **policy gradient** methods, which directly optimize the policy parameters θ:

**∇_θ J(θ) = E[ Σ ∇_θ log π_θ(a|s) * A(s,a) ]**

where **A(s,a)** is the **advantage function** (how much better action a is than average).

### Proximal Policy Optimization (PPO)

**PPO** (Schulman et al., 2017) is the most popular RL algorithm for robotics, offering:

- **Sample efficiency**: Reuses experience via importance sampling
- **Stability**: Clips policy updates to prevent large destructive changes
- **Simplicity**: Easy to implement and tune

PPO update rule:

```
L(θ) = E[ min( r_t(θ) * A_t,  clip(r_t(θ), 1-ε, 1+ε) * A_t ) ]
```

where **r_t(θ) = π_θ(a|s) / π_old(a|s)** is the importance ratio, clipped to [1-ε, 1+ε] (typically ε=0.2).

### Soft Actor-Critic (SAC)

**SAC** (Haarnoja et al., 2018) is an off-policy algorithm that maximizes both reward and entropy:

**J(θ) = E[ Σ (r_t + α * H(π(·|s_t))) ]**

SAC advantages:
- **Sample efficient**: Learns from replay buffer (off-policy)
- **Robust**: Entropy term encourages exploration
- **Continuous control**: Designed for continuous action spaces

SAC is preferred for manipulation tasks where sample efficiency is critical.

### Reward Shaping

Designing reward functions is the art of RL. For quadruped walking:

```python
def compute_reward(self, obs, actions):
    """
    Reward function for quadruped locomotion

    Args:
        obs: dict with 'base_lin_vel', 'base_ang_vel', 'joint_pos', etc.
        actions: commanded joint positions
    """
    # Target: walk forward at 1 m/s
    target_velocity = 1.0
    velocity_error = torch.abs(obs['base_lin_vel'][:, 0] - target_velocity)
    velocity_reward = torch.exp(-2.0 * velocity_error)  # Smooth reward

    # Penalty for falling (base height < threshold)
    fallen = obs['base_height'] < 0.3
    fall_penalty = -10.0 * fallen.float()

    # Penalty for large joint accelerations (smooth motion)
    joint_accel = (actions - self.prev_actions) / self.dt
    smoothness_penalty = -0.01 * torch.sum(joint_accel ** 2, dim=-1)

    # Penalty for energy consumption
    power = torch.sum(obs['joint_vel'] * obs['joint_torque'], dim=-1)
    energy_penalty = -0.001 * torch.abs(power)

    total_reward = (
        velocity_reward +
        fall_penalty +
        smoothness_penalty +
        energy_penalty
    )

    return total_reward
```

**Key principles**:
- **Dense rewards**: Provide feedback at every step (not just terminal)
- **Shaped rewards**: Use smooth functions (exp, gaussian) instead of thresholds
- **Penalty balancing**: Tune weights to balance competing objectives
- **Normalization**: Keep rewards in [-1, 1] range for stable learning

## Training in Isaac Sim

NVIDIA's **IsaacGymEnvs** provides a framework for massively parallel RL training in Isaac Sim, capable of simulating **thousands of robots simultaneously** on a single GPU.

### IsaacGymEnvs Architecture

Key components:

1. **VecEnv**: Vectorized environment running N robots in parallel
2. **Observation/Action spaces**: Defined in `Gym` interface
3. **Reward computation**: Custom reward function per task
4. **Reset logic**: Randomize initial states and environment parameters

Example training configuration for quadruped:

```python
# File: isaacgymenvs/tasks/anymal.py

class Anymal(VecTask):
    """Anymal quadruped locomotion task"""

    def __init__(self, cfg, sim_device, graphics_device_id, headless):
        self.cfg = cfg

        # Define observation space (48-dim: joint states, base velocity, etc.)
        self.num_obs = 48
        # Define action space (12-dim: 12 joint position targets)
        self.num_actions = 12

        super().__init__(
            config=self.cfg,
            sim_device=sim_device,
            graphics_device_id=graphics_device_id,
            headless=headless
        )

        # Create environments (4096 parallel robots)
        self.num_envs = self.cfg["env"]["numEnvs"]  # e.g., 4096

    def compute_observations(self):
        """Return current state observations"""
        self.obs_buf[:, :12] = self.dof_pos  # Joint positions
        self.obs_buf[:, 12:24] = self.dof_vel  # Joint velocities
        self.obs_buf[:, 24:27] = self.base_lin_vel  # Base linear velocity
        self.obs_buf[:, 27:30] = self.base_ang_vel  # Base angular velocity
        # ... more observations

        return self.obs_buf

    def compute_reward(self):
        """Compute reward for each environment"""
        # Use reward function from previous section
        self.rew_buf[:] = self._compute_locomotion_reward()

    def reset_idx(self, env_ids):
        """Reset specified environments"""
        # Randomize initial joint positions
        self.dof_pos[env_ids] = torch_rand_float(
            self.dof_lower_limits, self.dof_upper_limits
        )
        # Set base position and orientation
        self.root_states[env_ids, :3] = self.initial_root_pos
        # ...
```

### Training Script

```python
# File: train.py
import isaacgymenvs
from rl_games.torch_runner import Runner

# Load configuration
cfg = {
    "task": {
        "name": "Anymal",
        "physics_engine": "physx",
        "num_envs": 4096,
        "num_observations": 48,
        "num_actions": 12
    },
    "train": {
        "params": {
            "algo": {
                "name": "a2c_continuous"  # PPO variant
            },
            "network": {
                "name": "actor_critic",
                "separate": False,
                "mlp": {
                    "units": [256, 128, 64],  # Hidden layers
                    "activation": "elu"
                }
            },
            "config": {
                "learning_rate": 3e-4,
                "gamma": 0.99,
                "tau": 0.95,  # GAE parameter
                "entropy_coef": 0.01,
                "num_actors": 4096,
                "horizon_length": 24,  # Steps per rollout
                "minibatch_size": 16384,
                "num_minibatches": 4,
                "epochs_num": 5,
                "max_epochs": 10000
            }
        }
    }
}

# Create environment
envs = isaacgymenvs.make(
    seed=cfg["seed"],
    task=cfg["task"]["name"],
    num_envs=cfg["task"]["num_envs"],
    sim_device="cuda:0",
    rl_device="cuda:0"
)

# Train
runner = Runner()
runner.load(cfg)
runner.reset()
runner.run({
    "train": True,
    "play": False,
    "checkpoint": None
})
```

### Monitoring Training

Track training progress with **TensorBoard**:

```bash
tensorboard --logdir runs/Anymal_PPO
```

Key metrics:
- **Reward**: Should increase over time (e.g., -5 → +10)
- **Episode length**: Should increase as robot learns to walk longer
- **Policy loss**: Should decrease and stabilize
- **Value loss**: Should decrease and converge

**Training time**: 2-6 hours on RTX 4090 for 10M steps.

## Domain Randomization

**Domain randomization** (DR) is the key technique for sim-to-real transfer: by training on a **distribution of simulations** (varying physics, visuals, sensors), the policy learns to be robust to the specific reality it encounters.

### Physical Randomization

Randomize physics parameters at each episode reset:

```python
def randomize_physics(self, env_ids):
    """Randomize physical properties for specified environments"""

    # Mass randomization (±20%)
    for body_id in self.rigid_body_ids:
        mass = self.default_mass[body_id]
        rand_mass = mass * torch_rand_float(0.8, 1.2, (len(env_ids),))
        self.gym.set_rigid_body_mass(env_ids, body_id, rand_mass)

    # Friction randomization (0.5 to 1.5)
    rand_friction = torch_rand_float(0.5, 1.5, (len(env_ids),))
    self.gym.set_actor_friction(env_ids, rand_friction)

    # Joint damping randomization (±30%)
    for joint_id in range(self.num_dof):
        damping = self.default_damping[joint_id]
        rand_damping = damping * torch_rand_float(0.7, 1.3, (len(env_ids),))
        self.gym.set_dof_damping(env_ids, joint_id, rand_damping)

    # Actuator strength randomization (±15%)
    for joint_id in range(self.num_dof):
        max_force = self.default_max_force[joint_id]
        rand_force = max_force * torch_rand_float(0.85, 1.15, (len(env_ids),))
        self.gym.set_dof_max_force(env_ids, joint_id, rand_force)
```

### Visual Randomization

Randomize visual appearance to prevent overfitting to specific textures/colors:

```python
def randomize_visuals(self, env_ids):
    """Randomize visual properties"""

    # Ground texture randomization
    texture_files = [
        "textures/concrete.jpg",
        "textures/wood.jpg",
        "textures/carpet.jpg"
    ]
    texture_id = np.random.choice(len(texture_files))
    self.gym.set_actor_texture(env_ids, texture_id)

    # Lighting randomization
    rand_intensity = torch_rand_float(0.5, 1.5, (1,))
    self.gym.set_light_intensity(env_ids, rand_intensity)

    # Robot color randomization (HSV jitter)
    for link_id in self.robot_link_ids:
        rand_color = torch_rand_float(0.0, 1.0, (3,))  # RGB
        self.gym.set_rigid_body_color(env_ids, link_id, rand_color)
```

### Sensor Noise Injection

Add realistic sensor noise:

```python
def add_sensor_noise(self, obs):
    """Add sensor noise to observations"""

    # IMU noise (gyroscope drift, accelerometer noise)
    imu_noise = torch.randn_like(obs[:, :6]) * 0.05  # 5% noise
    obs[:, :6] += imu_noise

    # Joint encoder noise (0.01 rad position, 0.1 rad/s velocity)
    joint_pos_noise = torch.randn_like(obs[:, 6:18]) * 0.01
    joint_vel_noise = torch.randn_like(obs[:, 18:30]) * 0.1
    obs[:, 6:18] += joint_pos_noise
    obs[:, 18:30] += joint_vel_noise

    # Latency simulation (delay observations by 1-2 timesteps)
    if np.random.rand() < 0.3:
        obs = self.obs_buffer_prev  # Use previous observation

    return obs
```

### Why Randomization Works

DR works because the policy learns an **ensemble of skills** that work across the randomized distribution. When deployed to reality, the real world is likely within this distribution, so the policy generalizes. Empirical studies show DR improves sim-to-real success rates from 20-30% to 70-90%.

## Sim-to-Real Transfer

### Exporting the Policy

After training, export the policy for deployment:

```python
# Export policy as TorchScript (for fast inference)
import torch

policy = runner.get_policy()
policy.eval()

# Trace the model
example_input = torch.randn(1, 48)  # num_obs
traced_policy = torch.jit.trace(policy, example_input)

# Save to file
traced_policy.save("anymal_policy.pt")
```

### Running Policy on Real Robot

Integrate the policy into a ROS 2 node:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np

class RLPolicyNode(Node):
    """Deploy RL policy on real robot"""

    def __init__(self):
        super().__init__('rl_policy_node')

        # Load TorchScript policy
        self.policy = torch.jit.load("anymal_policy.pt")
        self.policy.eval()

        # ROS interfaces
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_position_commands', 10
        )

        self.obs = np.zeros(48)
        self.create_timer(0.02, self.control_loop)  # 50 Hz

    def state_callback(self, msg):
        """Update observations from robot state"""
        self.obs[:12] = msg.position  # Joint positions
        self.obs[12:24] = msg.velocity  # Joint velocities
        # Update other obs from IMU, base velocity estimate, etc.

    def control_loop(self):
        """Run policy inference and publish commands"""
        with torch.no_grad():
            obs_tensor = torch.tensor(self.obs, dtype=torch.float32).unsqueeze(0)
            actions = self.policy(obs_tensor).squeeze(0).numpy()

        # Publish joint commands
        cmd = Float64MultiArray()
        cmd.data = actions.tolist()
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = RLPolicyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Fine-Tuning with Real Data

If sim-to-real gap is large, **fine-tune** the policy with real-world data:

1. **Collect real trajectories**: Run policy on real robot, record (s, a, s', r)
2. **Fine-tune with RL**: Continue PPO training with real data (100-1000 episodes)
3. **Residual learning**: Train a residual policy Δπ that corrects sim policy

### Common Failure Modes

1. **Policy ignores critical features**: Train longer or increase network capacity
2. **Actuator saturation**: Reduce action magnitude or add action penalties
3. **Unmodeled delays**: Increase latency randomization in sim
4. **Contact dynamics mismatch**: Tune ground friction/compliance parameters

## Best Practices

1. **Start simple**: Train walking before running, grasping before manipulation
2. **Validate in sim first**: Achieve high success rate (>90%) in sim before real deployment
3. **Monitor safety**: Implement emergency stop if policy diverges
4. **Log everything**: Record all sensor data and actions for post-hoc analysis
5. **Iterate**: Analyze real failures, add corresponding randomization, retrain

## Summary

Key takeaways from this chapter:

- **Reinforcement learning** enables robots to learn complex behaviors through trial-and-error in simulation
- **PPO and SAC** are the dominant RL algorithms for robotics, balancing sample efficiency and stability
- **IsaacGymEnvs** enables massively parallel training (thousands of robots) for rapid policy learning
- **Domain randomization** is essential for sim-to-real transfer, training policies robust to physics and sensor variations
- **Successful deployment** requires careful reward shaping, extensive randomization, and real-world fine-tuning

These techniques have enabled breakthrough achievements in quadruped locomotion, dexterous manipulation, and bipedal walking for Physical AI systems.

## Review Questions

1. What is the reality gap, and why does it make sim-to-real transfer challenging?
2. Explain the difference between on-policy (PPO) and off-policy (SAC) RL algorithms. When would you choose each?
3. How does domain randomization help with sim-to-real transfer? Provide three specific examples of parameters to randomize.
4. Why is reward shaping critical for RL, and what are the principles for designing good reward functions?
5. Describe the IsaacGymEnvs workflow for training a quadruped locomotion policy from scratch.
6. What are the trade-offs between training in simulation vs. fine-tuning on real hardware?
7. How would you debug a policy that works in simulation but fails on real hardware?

## Hands-On Exercises

### Exercise 1: Train Navigation Policy

**Objective**: Train a PPO policy for point-to-point navigation in Isaac Sim.

**Steps**:
1. Set up IsaacGymEnvs environment for mobile robot
2. Define observation space (LiDAR, goal position, current velocity)
3. Define action space (linear velocity, angular velocity)
4. Design reward function (distance to goal, collision penalty)
5. Train for 5M steps and evaluate success rate

**Expected Outcome**: Policy achieves >80% success rate on navigation tasks in sim.

### Exercise 2: Apply Domain Randomization

**Objective**: Improve sim-to-real transfer with domain randomization.

**Steps**:
1. Implement physics randomization (mass, friction, damping)
2. Implement visual randomization (textures, lighting)
3. Add sensor noise (IMU, encoders)
4. Retrain policy with randomization
5. Compare sim-to-real transfer with and without DR

**Expected Outcome**: DR improves real-world success rate by 30-50%.

### Exercise 3: Deploy Policy on Real Robot

**Objective**: Export and deploy trained policy on real hardware.

**Steps**:
1. Export policy as TorchScript
2. Create ROS 2 node for policy inference
3. Test in controlled environment (soft ground, low speed)
4. Collect failure data and analyze
5. Retrain with additional randomization based on failures

**Expected Outcome**: Successful deployment with >70% success rate on real robot.

## Further Reading

- **PPO Paper**: "Proximal Policy Optimization Algorithms" (Schulman et al., 2017)
- **SAC Paper**: "Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL" (Haarnoja et al., 2018)
- **IsaacGymEnvs Documentation**: [https://github.com/NVIDIA-Omniverse/IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)
- **Domain Randomization**: "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization" (Peng et al., 2018)
- **Learning Dexterity**: OpenAI's Dactyl project ([https://openai.com/research/learning-dexterity](https://openai.com/research/learning-dexterity))

---

**Previous**: [Chapter 14 - Perception Pipelines](../week-09/ch14-perception.md)
**Next**: [Chapter 16 - Sensor Fusion for Robotics](ch16-sensor-fusion.md)
