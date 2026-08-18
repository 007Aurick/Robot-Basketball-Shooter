# 🏀 Basketball Free-Throw Robot Arm

Simulated robot arm shooting basketball free throws in MuJoCo, controlled via ROS 2. Uses projectile motion to compute release angle/velocity, drives a prebuilt arm model through a throwing motion, and releases the ball to sink the shot. Built to learn MuJoCo + ROS 2 control.

![MuJoCo](https://img.shields.io/badge/MuJoCo-Physics%20Sim-orange)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-yellow)
![Status](https://img.shields.io/badge/status-in%20progress-brightgreen)

## Overview

This project simulates a robotic arm performing a basketball free throw entirely in physics simulation. Given a fixed distance and hoop height, the system computes the required release angle and velocity using projectile motion, then drives a robot arm through a throwing motion in [MuJoCo](https://mujoco.org/), releasing the ball at the correct point to sink the shot — all coordinated through [ROS 2](https://docs.ros.org/en/humble/index.html).

## Features

- 🎯 Analytical projectile-motion solver for release angle/velocity
- 🦾 Prebuilt robot arm model (via [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)) driven through ROS 2 joint commands
- 🏀 Basketball, hoop, backboard, and court modeled in MuJoCo (MJCF)
- 🔌 ROS 2 node architecture connecting planning and control to the sim
- 📈 Planned: variable court positions, 3-pointers/long jumpers, and RL-based shot policies

## Tech Stack

- **Simulation:** MuJoCo
- **Middleware:** ROS 2 (Humble)
- **Language:** Python
- **Robot Model:** MuJoCo Menagerie

## Getting Started

\`\`\`bash
# clone the repo
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>

# build the ROS 2 workspace
colcon build
source install/setup.bash

# run the free-throw shooter node
ros2 run bball_arm_control free_throw_shooter
\`\`\`

## Roadmap

- [x] Arm + ball + hoop scene in MuJoCo
- [x] Analytical free-throw solver
- [ ] Full ROS 2 integration for throw execution
- [ ] Variable court position support
- [ ] Three-pointers / long jumpers
- [ ] RL-based shot policy (stretch goal)

## Tags

\`#robotics\` \`#mujoco\` \`#ros2\` \`#simulation\` \`#basketball\` \`#roboticsengineering\` \`#physicssimulation\` \`#python\` \`#controlsystems\` \`#robotarm\`

## Author

Built by [Aurick Anwar] — part of a summer robotics self-study project.
