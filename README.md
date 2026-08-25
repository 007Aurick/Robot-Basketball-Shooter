# 🏀 G1 Free Throw Sim
Simulated Unitree G1 humanoid sinking free throws and straight-on shots from the top of the key in MuJoCo. Uses projectile motion to compute release angle/velocity, drives the humanoid through a scripted throwing motion, and releases the ball at the right instant to sink the shot. Built to learn MuJoCo + humanoid control.

![MuJoCo](https://img.shields.io/badge/MuJoCo-Physics%20Sim-orange)
![Python](https://img.shields.io/badge/Python-3.x-yellow)
![NumPy](https://img.shields.io/badge/NumPy-Math-013243)
![Status](https://img.shields.io/badge/status-in%20progress-brightgreen)

## Overview
This project simulates a Unitree G1 humanoid performing a basketball shot entirely in physics simulation. Given a fixed distance and hoop height, the system computes the required release angle and velocity using projectile motion, then drives the humanoid through a scripted throwing motion in [MuJoCo](https://mujoco.org/), releasing the ball at the correct point to sink the shot. Currently supports straight-on shots only — free throws and top-of-the-key attempts — as a baseline before adding richer court geometry.

## Features
- 🎯 Analytical projectile-motion solver for release angle/velocity
- 🦾 Unitree G1 humanoid (via [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)) driven through a scripted joint-space throw
- 🏀 Basketball, hoop, backboard, and court modeled in MuJoCo (MJCF)
- 📏 Parameterized straight-line shot positions (free throw line, top of the key) using one solver
- ⚡ Lightweight stack — pure MuJoCo + Python + NumPy, no middleware
- 📈 Planned: arbitrary court positions, lateral offsets, and release angles beyond straight-on

## Tech Stack
- **Simulation:** MuJoCo
- **Language:** Python
- **Math:** NumPy
- **Robot Model:** MuJoCo Menagerie (Unitree G1)

## Getting Started
```bash
# clone the repo
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>

# install dependencies
pip install -r requirements.txt

# run the free-throw shooter
python free_throw_shooter.py
```

## Roadmap
- [x] G1 + ball + hoop + backboard + court scene in MuJoCo
- [x] Analytical projectile-motion solver
- [x] Scripted throw motion with computed release timing
- [x] Straight-on shots from free throw line and top of the key
- [x] Arbitrary court positions / lateral offsets
- [x] Non-straight-on release angles
- [x] RL-based shot policy (stretch goal)

## Tags
`#robotics` `#mujoco` `#humanoid` `#unitreeg1` `#simulation` `#basketball` `#roboticsengineering` `#physicssimulation` `#python` `#projectilemotion`

## Author
Built by [Your Name] — part of a summer robotics self-study project.
