# Vision-Based Autonomous Parking

An end-to-end autonomous box-parking pipeline for a 2-wheeled differential-drive robot implemented in **MATLAB**. Developed as a university project for **Smart Systems Engineering (Fall 2026)**, this system integrates Computer Vision, Path Planning with Model Predictive Control (MPC), and Low-Level Motion Control.

---

## Project Overview

The goal of this project is to autonomously detect an empty parking bay using camera feedback, compute a collision-free trajectory, and park a 2-wheeled robot accurately inside the target box.

* **Minimum Viable Product (MVP):** Rule-based computer vision for bay detection, pixel-to-world coordinate projection, kinematic Model Predictive Control (MPC) for trajectory generation, and closed-loop wheel velocity control.
* **Stretch Extensions:** Custom U-Net semantic segmentation for parking line detection and a custom Convolutional Neural Network (CNN) for parking spot occupancy classification.

---

# Autonomous Box Parking — Roadmap

## 1. System Setup
- [x] Establish MATLAB project structure and shared Git repository.
- [ ] Define module interfaces, data structures, and coordinate frames.
- [ ] Establish MATLAB-to-robot communication.
- [x] Build the physical parking bay and configure the camera.

## 2. Perception
- [ ] Calibrate camera and determine intrinsic/extrinsic parameters.
- [ ] Implement image undistortion and Bird's-Eye View transformation.
- [ ] Detect parking bay boundaries and extract four corners.
- [ ] Transform pixel coordinates into robot/world coordinates.
- [ ] Calculate target parking position and orientation.
- [ ] Implement temporal filtering for stable detection.
- [ ] Implement optional CNN/U-Net-based detection and occupancy classification.

## 3. Planning & MPC
- [ ] Implement smooth reference path generation between initial and target poses.
- [x] Develop the four-state kinematic robot model.
- [x] Implement MPC prediction model, cost function, and constraints.
- [ ] Generate velocity and angular velocity control commands.
- [ ] Build MATLAB/Simulink simulation environment.
- [ ] Tune MPC parameters and validate different parking approaches.
- [ ] Optimize real-time execution and implement optional dynamic replanning.

## 4. Motion Control
- [ ] Configure motors, encoders, and determine robot parameters.
- [ ] Implement differential-drive inverse kinematics.
- [ ] Implement encoder-based robot odometry.
- [ ] Develop and tune independent PI/PID wheel velocity controllers.
- [ ] Implement PWM control, anti-windup, and feedforward compensation.
- [ ] Provide continuous robot state feedback to MPC.

## 5. System Integration
- [ ] Connect Perception, Planning/MPC, and Motion Control.
- [ ] Integrate all modules into a master MATLAB control loop.
- [ ] Validate coordinate frames, communication, and data flow.
- [ ] Verify real-time execution and closed-loop feedback.
- [ ] Test complete autonomous parking operation.

## 6. Testing & Validation
- [ ] Perform physical parking trials from different initial positions and orientations.
- [ ] Evaluate parking accuracy, trajectory tracking, and heading error.
- [ ] Analyze perception stability and controller performance.
- [ ] Tune system parameters based on physical test results.
- [ ] Validate repeatability and reliability of autonomous parking.

## 7. Documentation & Deliverables
- [ ] Compile experimental results and performance plots.
- [ ] Document system architecture and module implementations.
- [ ] Prepare final technical report.
- [ ] Prepare final presentation.

---

## System Architecture

    ┌─────────────────────────────────────────────────────────────┐
    │                                                             │
    │   ┌──────────────┐                                          │
    │   │    CAMERA    │                                          │
    │   │  Image Input │                                          │
    │   └──────┬───────┘                                          │
    │          │                                                  │
    │          ▼                                                  │
    │   ┌───────────────────────┐                                 │
    │   │      PERCEPTION       │                                 │
    │   │                       │                                 │
    │   │                       │                                 │
    │   │  • Image acquisition  │                                 │
    │   │  • Parking bay        │                                 │
    │   │    detection          │                                 │
    │   │  • Coordinate mapping │                                 │
    │   └───────────┬───────────┘                                 │
    │               │                                             │
    │               │ target_pose [x, y, θ]                       │
    │               ▼                                             │
    │   ┌───────────────────────┐                                 │
    │   │       PLANNING        │                                 │
    │   │                       │                                 │
    │   │                       │                                 │
    │   │  • Trajectory         │                                 │
    │   │    generation         │                                 │
    │   │  • Robot dynamics     │                                 │
    │   │  • Model Predictive   │                                 │
    │   │    Control (MPC)      │                                 │
    │   └───────────┬───────────┘                                 │
    │               │                                             │
    │               │ control_cmd [v, ω]                          │
    │               ▼                                             │
    │   ┌───────────────────────┐                                 │
    │   │    MOTION CONTROL     │                                 │
    │   │                       │                                 │
    │   │                       │                                 │
    │   │  • Differential drive │                                 │
    │   │    mapping            │                                 │
    │   │  • Wheel velocity PID │                                 │
    │   │  • Motor commands                                       │
    │   └───────────┬──────────┘                                  │
    │               │                                             │
    │               │ left/right PWM                              │
    │               ▼                                             │
    │   ┌───────────────────────┐                                 │
    │   │     MOTOR DRIVER      │                                 │
    │   └───────────┬───────────                                  │
    │               │                                             │
    │               ▼                                             │
    │   ┌───────────────────────┐                                 │
    │   │    ROBOT PLATFORM     │                                 │
    │   │                       │                                 │
    │   │  • Differential drive │                                 │
    │   │  • Wheel encoders     │                                 │
    │   └───────────┬───────────┘                                 │
    │               │                                             │
    │               │ Feedback: robot state                       │
    │               │ [x, y, θ, v, ω]                             │
    │               └──────────────────────────────┐              │
    │                                              │              │
    └──────────────────────────────────────────────┼──────────────┘
                                                   │
                                                   ▼
                                      ┌────────────────────────┐
                                      │   CLOSED-LOOP UPDATE   │
                                      │                        │
                                      │  • State estimation    │
                                      │  • Tracking error      │
                                      │  • Control update      │
                                      │  • Parking validation  │
                                      └────────────┬───────────┘
                                                   │
                                                   └──────▶ Repeat
