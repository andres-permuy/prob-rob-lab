# Probabilistic Robotics Labs

## Lab 4: Odometry — Complete

> **Status (November 2, 2026):** All nine assignments are complete. The report, evidence, and video are in [`submission/lab4_Odometry/`](submission/lab4_Odometry/) (start with `START_HERE.txt`).

| Assignment | Status |
|---|---|
| **1. Zero-command velocity** | Bag, analysis script, histograms, and statistics in [`assignment1/`](submission/lab4_Odometry/assignment1/). |
| **2. Commanded motion** | PlotJuggler/RViz screenshots in [`assignment2/`](submission/lab4_Odometry/assignment2/); videos linked in the report. |
| **3. Robot dynamics** | Step-command script, bag, and annotated step plots in [`assignment3/`](submission/lab4_Odometry/assignment3/). |
| **4–5. EKF models** | Motion model, Jacobian, and measurement model (R = 0.1435 m for the simulated Waffle Pi) in the report. |
| **6. EKF odometry node** | [`ekf_odom.py`](prob_rob_labs/src/lab4_odom/ekf_odom.py), `ros2 run prob_rob_labs ekf_odom`; verified in simulation. |
| **7–9. Evaluation** | Error node [`ekf_error.py`](prob_rob_labs/src/lab4_odom/ekf_error.py) (`ros2 run prob_rob_labs ekf_error`), drive pattern, evaluation bag, video, and covariance analysis in [`assignment7/`](submission/lab4_Odometry/assignment7/)–[`assignment9/`](submission/lab4_Odometry/assignment9/). |

---

## Overview

This repository contains the ROS 2 packages, Gazebo simulation assets, and lab submissions for **Columbia University's EEME E6911: Probabilistic Robotics**. It builds on [Professor Ilija Hadzic's course repository](https://github.com/ihadzic/prob_rob_labs_ros_2).

The labs use a simulated TurtleBot3 to explore ROS nodes, sensing, Bayesian estimation, and odometry. We use the professor's existing simulation and package architecture wherever possible.

## Repository Layout

| Path | Contents |
|---|---|
| [`prob_rob_labs/`](prob_rob_labs/) | Main ROS 2 Python package: launch files, worlds, controllers, ground-truth publishers, and Lab 4 EKF node. |
| [`models/`](models/) | Gazebo simulation models. |
| [`prob_rob_msgs/`](prob_rob_msgs/) | Course-specific ROS message definitions. |
| [`vision/`](vision/) | Course vision-processing package. |
| [`tools/`](tools/) | Professor-provided setup scripts, templates, and utilities. |
| [`submission/`](submission/) | Lab submissions and supporting data, separated by lab. |

Previous work is under [`submission/lab2_hw3214/`](submission/lab2_hw3214/) and [`submission/lab3_Bayes_Filter/`](submission/lab3_Bayes_Filter/). Lab 4 source code stays in the installable ROS package; its experimental data and eventual report belong under [`submission/lab4_Odometry/`](submission/lab4_Odometry/).

## Environment and Usage

The course environment uses **Ubuntu 24.04, ROS 2 Jazzy, Gazebo, and TurtleBot3**. Use the professor's scripts in [`tools/`](tools/) to set up a personal Ubuntu workstation or a Columbia lab workstation. The repository is expected at:

```text
$MY_WORKSPACE/src/prob_rob_labs_ros_2
```

For a **new** configured workspace, build the supporting packages and then the main package:

```bash
source ~/.bashrc
cc_prebuild
cc_build
source "$MY_WORKSPACE/install/setup.bash"
```

For subsequent changes to the Python ROS package, `cc_build` and re-sourcing the workspace are generally sufficient.

Launch the Lab 4 empty-world simulation:

```bash
ros2 launch prob_rob_labs turtlebot3_empty_launch.py
```

In another terminal, start the EKF node (and `ekf_error` for evaluation):

```bash
ros2 run prob_rob_labs ekf_odom --ros-args -p use_sim_time:=true
```

The node publishes `nav_msgs/msg/Odometry` on `/ekf_odom`, using `/cmd_vel`, `/imu`, and `/joint_states` as inputs. Ground truth is available on `/tb3/ground_truth/pose` and `/tb3/ground_truth/twist` for evaluation.

## Notes

- Keep modifications focused on the assigned lab; avoid changing professor-provided simulation components unnecessarily.
- `submission/` contains deliverables and supporting evidence, not installable ROS nodes.
- The Lab 4 report is being prepared in **LaTeX on Overleaf**. Its equations and variable names are kept consistent with the lecture material and ROS implementation.