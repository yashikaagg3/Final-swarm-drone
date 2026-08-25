# Autonomous Multi-Drone Swarm Simulation

A ROS 2 Humble and Gazebo based simulation of an autonomous multi-drone swarm for intelligent area coverage using a leader-follower architecture.

## Overview

This project implements an autonomous multi-drone swarm in simulation, where multiple drones cooperatively cover a predefined area.

The swarm consists of one leader drone and multiple follower drones. The number of drones is configurable through a YAML file.

The leader is responsible for coordinating the swarm, dividing the coverage area into approximately equal regions, assigning each region to a drone, and monitoring the mission. The leader also covers its own assigned region.

Each drone autonomously navigates to its assigned region and follows a systematic coverage path to cover the region.

The project is designed with a modular architecture so that the high-level swarm logic can later be integrated with PX4 and MAVROS.

---

## Key Features

- Autonomous multi-drone swarm simulation
- Leader-follower swarm architecture
- Configurable number of drones
- Automatic area division
- Approximately equal-area task allocation
- Autonomous waypoint generation
- Systematic area coverage using a lawnmower pattern
- ROS 2 topic and service based communication
- Unique ID for every drone
- Individual drone testing using drone IDs
- Leader also participates in area coverage
- RGB camera on every drone
- RViz2 visualization
- `MarkerArray` based visualization
- Modular URDF/Xacro drone model
- Gazebo simulation environment
- YAML-based configuration
- Architecture prepared for future PX4/MAVROS integration

---

## System Architecture

The swarm follows a leader-follower architecture.

```text
                         ┌──────────────────┐
                         │     DRONE 0      │
                         │      LEADER      │
                         └────────┬─────────┘
                                  │
                    Task Allocation /
                    Region Assignment
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
              ▼                   ▼                   ▼
       ┌────────────┐      ┌────────────┐      ┌────────────┐
       │  DRONE 1   │      │  DRONE 2   │      │  DRONE 3   │
       │  FOLLOWER  │      │  FOLLOWER  │      │  FOLLOWER  │
       └────────────┘      └────────────┘      └────────────┘
