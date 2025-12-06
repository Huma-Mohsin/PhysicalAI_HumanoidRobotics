---
sidebar_position: 1
title: Gazebo - Physics Simulation
---

# Gazebo: Simulating Physical Robots

Gazebo provides physics simulation for training AI before deploying to real hardware.

## AI + Physical Integration

### Reinforcement Learning in Simulation

```python
# Train RL agent in Gazebo, deploy to physical robot
env = GazeboEnv()  # Simulated environment
agent = PPO('MlpPolicy', env)  # AI agent
agent.learn(total_timesteps=100000)  # Train in simulation

# Deploy trained policy to physical robot
real_robot.execute_policy(agent.policy)
```

**Physical Execution**: AI learns in Gazebo → Transfers policy to real robot motors

See [Gazebo documentation](http://gazebosim.org/).
