# PDDL Single-Agent and Multi-Agent Path Planning

This project explores automated planning for grid-based robot path finding using PDDL.

It contains two parts:

1. Single-agent path planning
2. Collision-aware multi-agent path planning

## Project Overview

The single-agent model focuses on planning a path for one robot while treating other robots as immovable obstacles. The model includes robot locations, directional states, movement actions, and goal conditions.

The multi-agent model extends the planning problem to multiple robots moving in the same grid environment. It considers collision-aware planning, including vertex collisions and edge(swapping) collisions.

## Key Features

- PDDL domain modelling
- Single-agent robot path planning
- Multi-agent path finding
- Directional movement and rotation
- Vertex collision avoidance
- Edge collision avoidance
- Planner output examples

## Repository Structure

```text
single-agent/
  domain-single.pddl
  domain-singleRobotRunner.pddl
  problem1.pddl
  plan-single-1.txt
  

multi-agent/
  domain-multiple.pddl
  domain-multipleRobotRunner.pddl
  problem2.pddl
  problem3.pddl
  problem4.pddl
  plan-multi-1.txt
  plan-multi-2.txt
  plan-multi-3.txt