# Probabilistic Robotics Labs

## Lab 5: EKF Localization · Vision

**Status:** Assignments 1–4 are complete, including simulation experiments and variance-model replay verification. Source code, recorded bags, figures, and videos are included. The report can be found here:

[FINAL REPORT](/src/prob_rob_labs_ros_2/submission/lab5_EKF_vision/Lab__5__EKF_Localization___Vision.pdf)

Lab 5 implements camera-based landmark measurements, ground-truth error evaluation, and an empirical measurement-variance model.

| Assignment | Implementation and result |
|---|---|
| **1. Landmark pixel geometry** | [`landmark_measurement.py`](prob_rob_labs/src/lab5_vision/landmark_measurement.py) calculates pixel height and horizontal center. Checked with example corner points. |
| **2. Distance and bearing** | [`landmark_distance_bearing.py`](prob_rob_labs/src/lab5_vision/landmark_distance_bearing.py) uses Assignment 1 geometry and camera calibration to publish distance and bearing. Visibility checks and live simulation tests are complete. |
| **3. Measurement error** | [`landmark_measurement_error.py`](prob_rob_labs/src/lab5_vision/landmark_measurement_error.py) compares synchronized measurements with camera-frame ground truth and publishes signed errors. Recorded-motion replay is complete. |
| **4. Measurement variance** | [`landmark_measurement_variance.py`](prob_rob_labs/src/lab5_vision/landmark_measurement_variance.py) evaluates the fitted variance model. The Assignment 3 node calls this function and publishes its outputs; replay verification is complete. |

### Submission Evidence

All Lab 5 evidence is under [`submission/lab5_EKF_vision/`](submission/lab5_EKF_vision/):

- **Assignment 2:** corrected-node [measurement bag](submission/lab5_EKF_vision/assignment2/data/vision_test_02/), [PlotJuggler figure](submission/lab5_EKF_vision/assignment2/figures/lab5_assignment2_test2.png), and [video](submission/lab5_EKF_vision/assignment2/video/lab5_assignment2_test2.mp4). The initial run is retained in the same assignment directory.
- **Assignment 3:** [measurement/error bag](submission/lab5_EKF_vision/assignment3/data/error_replay_01/), [error plots and ROS graph](submission/lab5_EKF_vision/assignment3/figures/), and [video](submission/lab5_EKF_vision/assignment3/videos/lab5_assignment3.mp4).
- **Assignment 4:** [trajectory collection bag](submission/lab5_EKF_vision/assignment4/data/variance_test_01/), [variance-model replay bag](submission/lab5_EKF_vision/assignment4/data/variance_model_replay_02/), [collection plot](submission/lab5_EKF_vision/assignment4/figures/lab5_a4.png), [variance-output plot](submission/lab5_EKF_vision/assignment4/figures/variance.png), and [video](submission/lab5_EKF_vision/assignment4/videos/lab5_a4.mp4).

## Overview

This repository contains the ROS 2 packages, Gazebo simulation assets, and submissions for **Columbia University's EEME E6911: Probabilistic Robotics**. It builds on [Professor Ilija Hadzic's course repository](https://github.com/ihadzic/prob_rob_labs_ros_2).

| Path | Contents |
|---|---|
| [`prob_rob_labs/`](prob_rob_labs/) | Main Python package, launch files, worlds, and student nodes, including Lab 4 odometry and Lab 5 vision measurements and evaluation. |
| [`models/`](models/) | Gazebo simulation models. |
| [`prob_rob_msgs/`](prob_rob_msgs/) | Course-specific ROS messages, including stamped corner-point arrays. |
| [`vision/`](vision/) | Professor-provided color filtering and feature detection. |
| [`tools/`](tools/) | Workspace setup scripts, node templates, and utilities. |
| [`submission/`](submission/) | Reports and experimental evidence organized by lab. |

Previous submissions remain in [`lab2_hw3214/`](submission/lab2_hw3214/), [`lab3_Bayes_Filter/`](submission/lab3_Bayes_Filter/), and [`lab4_Odometry/`](submission/lab4_Odometry/). Lab 4 is complete, with its [report PDF](submission/lab4_Odometry/Lab4_Odometry_Report.pdf), EKF source, and evaluation evidence retained.

## Environment and Build

Use the configured course environment: **Ubuntu 24.04, ROS 2 Jazzy, Gazebo, and TurtleBot3 Waffle Pi**. Workspace setup scripts are in [`tools/`](tools/); the expected repository location is `$MY_WORKSPACE/src/prob_rob_labs_ros_2`.

For a new workspace, build the supporting packages and then the main package:

```bash
source ~/.bashrc
cc_prebuild
cc_build
source "$MY_WORKSPACE/install/setup.bash"
```

After adding nodes or changing executable registrations, run `cc_build` and re-source the workspace. Run the following commands from the repository root in separate sourced terminals.

## Run Lab 5

### Live Measurements and Evaluation

Launch the landmark world and its vision pipeline:

```bash
ros2 launch prob_rob_labs turtlebot3_among_landmarks_launch.py
```

Start the Assignment 2 measurement node:

```bash
ros2 run prob_rob_labs landmark_distance_bearing --ros-args \
  -p use_sim_time:=true \
  -p landmark_color:=cyan \
  -p landmark_height:=0.5 \
  -p edge_margin_px:=5.0
```

Start the Assignment 3 evaluation node, which also publishes Assignment 4 variances:

```bash
ros2 run prob_rob_labs landmark_measurement_error --ros-args \
  -p use_sim_time:=true \
  -p landmark_color:=cyan
```

The measurement node reads `/camera/camera_info` and `/vision_cyan/corners`. Its measurement topic and the evaluator outputs use `Vector3Stamped` as scalar-pair messages:

| Topic | `vector.x` | `vector.y` |
|---|---|---|
| `/vision_cyan/measurement` | Measured distance (m) | Measured bearing (rad, positive left) |
| `/vision_cyan/evaluation/measurement` | Matched distance (m) | Matched bearing (rad) |
| `/vision_cyan/evaluation/error` | Signed distance error (m) | Signed bearing error (rad) |
| `/vision_cyan/evaluation/variance` | Distance variance (m²) | Bearing variance (rad²) |

All four topics preserve the source measurement timestamp and camera frame; `vector.z` is unused. The evaluator uses ground-truth robot pose and the fixed camera mounting transform. The variance function uses measured distance, with `distance_variance = 1.6e-4 * distance**2` and `bearing_variance = 1.0e-6`.

For manual steering, launch the GUI and select `/cmd_vel` with **Stamped** enabled:

```bash
ros2 run rqt_robot_steering rqt_robot_steering --ros-args -p use_sim_time:=true
```

The [collection helper](prob_rob_labs/src/lab5_vision/collect_variance_data.py) reproduces the automated cyan-landmark trajectory. With the live nodes and a bag recorder running, launch it instead of the steering GUI:

```bash
python3 prob_rob_labs/src/lab5_vision/collect_variance_data.py \
  --ros-args -p use_sim_time:=true
```

### Replay the Completed Assignment 4 Run

Start the evaluation node above and PlotJuggler. Leave Gazebo, the Assignment 2 node, and command publishers closed. Replay only the original input topics so previously recorded evaluation outputs do not mix with the updated node's outputs:

```bash
ros2 bag play \
  submission/lab5_EKF_vision/assignment4/data/variance_test_01 \
  --rate 2.0 \
  --topics \
  /clock \
  /tf_static \
  /tb3/ground_truth/pose \
  /vision_cyan/measurement
```

This replays the recorded `/clock`; no generated playback clock is needed. The variance model is evaluated during replay; its coefficients are already fitted.

### Visualization

Launch PlotJuggler in its own sourced terminal:

```bash
ros2 run plotjuggler plotjuggler
```

Use the ROS2 Topic Subscriber to graph the measurement, error, or variance fields listed above. For the variance replay, use a 1000-second buffer, automatic horizontal limits, and vertical limits of **0–0.020 m²** for distance variance and **0–0.000002 rad²** for bearing variance.

For live image visualization:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Add an Image display for `/vision_cyan/image_raw`.

## Working Conventions

- Keep installable code under `prob_rob_labs/src/` and submission evidence under `submission/`.
- Keep assignment calculations in separate scripts and reuse their functions from the ROS nodes.
- Follow the professor's templates and lecture notation, keep changes focused on the lab, and preserve earlier submissions.
- Export the final Overleaf PDF, `.tex` source, and report figures to the Lab 5 submission folder for archiving.