# Probabilistic Robotics Labs

## Lab 5: EKF Localization · Vision

**Status (November 13, 2026):** Assignments 1–3 are complete. Assignment 4 remains. The report is in progress on Overleaf; its final PDF and LaTeX source have not yet been added to the repository.

| Assignment | Current state |
|---|---|
| **1. Landmark pixel geometry** | [`landmark_measurement.py`](prob_rob_labs/src/lab5_vision/landmark_measurement.py) implements `landmark_pixel_geometry()` and was checked using example corner points. |
| **2. Distance and bearing** | [`landmark_distance_bearing.py`](prob_rob_labs/src/lab5_vision/landmark_distance_bearing.py) imports the Assignment 1 function and publishes camera measurements. Simulation tests and the corrected detection checks are complete. Bags, plot, and video are in [`assignment2/`](submission/lab5_EKF_vision/assignment2/). |
| **3. Measurement error** | [`landmark_measurement_error.py`](prob_rob_labs/src/lab5_vision/landmark_measurement_error.py) compares synchronized measurements with camera-frame ground truth. Replay testing is complete; the output bag, error plots, ROS graph, and video are in [`assignment3/`](submission/lab5_EKF_vision/assignment3/). |
| **4. Measurement variance** | Not started. Collect systematic distance/bearing coverage and implement a simple variance model supported by the data. |

The current Lab 5 evidence is under [`submission/lab5_EKF_vision/`](submission/lab5_EKF_vision/):

- [`vision_test_01`](submission/lab5_EKF_vision/assignment2/data/vision_test_01/) contains the initial run; [`vision_test_02`](submission/lab5_EKF_vision/assignment2/data/vision_test_02/) contains the corrected-node run.
- The second run's [PlotJuggler figure](submission/lab5_EKF_vision/assignment2/figures/lab5_assignment2_test2.png) and [screen recording](submission/lab5_EKF_vision/assignment2/video/lab5_assignment2_test2.mp4) support Assignment 2.
- Assignment 3 includes the paired [measurement/error bag](submission/lab5_EKF_vision/assignment3/data/error_replay_01/), [error plots](submission/lab5_EKF_vision/assignment3/figures/), [ROS computation graph](submission/lab5_EKF_vision/assignment3/figures/rqt.png), and [screen recording](submission/lab5_EKF_vision/assignment3/videos/lab5_assignment3.mp4).

## Overview

This repository contains the ROS 2 packages, Gazebo simulation assets, and submissions for **Columbia University's EEME E6911: Probabilistic Robotics**. It builds on [Professor Ilija Hadzic's course repository](https://github.com/ihadzic/prob_rob_labs_ros_2).

The labs use a simulated TurtleBot3 and the professor's existing package architecture, node templates, and simulation tools.

| Path | Contents |
|---|---|
| [`prob_rob_labs/`](prob_rob_labs/) | Main Python package, launch files, worlds, and student nodes, including Lab 4 odometry and Lab 5 measurement and error nodes. |
| [`models/`](models/) | Gazebo simulation models. |
| [`prob_rob_msgs/`](prob_rob_msgs/) | Course-specific ROS messages, including stamped corner-point arrays. |
| [`vision/`](vision/) | Professor-provided color filtering and feature detection. |
| [`tools/`](tools/) | Workspace setup scripts, node templates, and utilities. |
| [`submission/`](submission/) | Reports and experimental evidence, organized by lab. |

Previous submissions are in [`lab2_hw3214/`](submission/lab2_hw3214/), [`lab3_Bayes_Filter/`](submission/lab3_Bayes_Filter/), and [`lab4_Odometry/`](submission/lab4_Odometry/). **Lab 4 is complete**, with its [report PDF](submission/lab4_Odometry/Lab4_Odometry_Report.pdf), EKF source, and evaluation evidence retained.

## Environment and Build

Use the configured course environment: **Ubuntu 24.04, ROS 2 Jazzy, Gazebo, and TurtleBot3 Waffle Pi**. The setup scripts are in [`tools/`](tools/); the expected repository location is `$MY_WORKSPACE/src/prob_rob_labs_ros_2`.

For a new workspace, build the supporting packages and then the main package:

```bash
source ~/.bashrc
cc_prebuild
cc_build
source "$MY_WORKSPACE/install/setup.bash"
```

After adding nodes or changing executable registrations, run `cc_build` and re-source the workspace. In each new terminal, enter the same course environment and source the workspace before running commands.

## Run Lab 5

Launch the landmark world and its vision pipeline:

```bash
ros2 launch prob_rob_labs turtlebot3_among_landmarks_launch.py
```

In another sourced terminal, start the Assignment 2 node:

```bash
ros2 run prob_rob_labs landmark_distance_bearing --ros-args \
  -p use_sim_time:=true \
  -p landmark_color:=cyan \
  -p landmark_height:=0.5 \
  -p edge_margin_px:=5.0
```

For cyan, the node reads `/camera/camera_info` and `/vision_cyan/corners` and publishes `/vision_cyan/measurement`. Its `Vector3Stamped` fields encode distance in `vector.x` (m), bearing in `vector.y` (rad, positive left), and zero in `vector.z`. The header preserves the source image timestamp and camera frame.

For GUI steering without a joystick:

```bash
ros2 run rqt_robot_steering rqt_robot_steering --ros-args -p use_sim_time:=true
```

Select **`/cmd_vel`** and enable **Stamped** in the GUI. Return both velocity commands to zero when stopping.

For visualization, launch each tool in a separate sourced terminal:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

```bash
ros2 run plotjuggler plotjuggler
```

In RViz, add an Image display for `/vision_cyan/image_raw`. In PlotJuggler, use the ROS2 Topic Subscriber and plot `/vision_cyan/measurement/vector/x` and `/vision_cyan/measurement/vector/y`.

### Assignment 3: Measurement Error

For a live experiment, keep the landmark world and Assignment 2 node running, then start the error node in another sourced terminal:

```bash
ros2 run prob_rob_labs landmark_measurement_error --ros-args \
  -p use_sim_time:=true \
  -p landmark_color:=cyan
```

The node uses ground truth and the robot-to-camera TF transform. It publishes matched observations on `/vision_cyan/evaluation/measurement` and signed errors on `/vision_cyan/evaluation/error`, with matching measurement timestamps. For both topics, `vector.x` is distance or distance error (m), and `vector.y` is bearing or bearing error (rad).

To reuse the existing recorded motion instead, leave Gazebo and the Assignment 2 node closed. Start the error node and PlotJuggler, then launch playback in another terminal:

```bash
ros2 bag play \
  submission/lab5_EKF_vision/assignment2/data/vision_test_02 \
  --start-paused
```

The bag supplies the original measurements, ground truth, TF, and `/clock`. In PlotJuggler, subscribe to the evaluation topics, then press **Space in the playback terminal** to resume. Plot the error topic's `vector.x` and `vector.y`; the fields appear once messages arrive.

## Working Conventions

- Keep installable code under `prob_rob_labs/src/` and submission evidence under `submission/`.
- Keep each assignment's code in its own script. Reuse the Assignment 1 geometry function from the Assignment 2 node; keep Assignment 3 evaluation separate.
- Follow the professor's templates and lecture notation, keep changes focused on the lab, and preserve earlier submissions.
- Continue the report on Overleaf as each assignment is completed. Add the final PDF, `.tex` source, and figures to the Lab 5 submission folder when finalized.