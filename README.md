# Vision-Based Autonomous Parking

An end-to-end autonomous box-parking pipeline for a 2-wheeled differential-drive robot implemented in **MATLAB**. Developed as a university project for **Smart Systems Engineering (Fall 2026)**, this system integrates Computer Vision, Path Planning with Model Predictive Control (MPC), and Low-Level Motion Control.

---

## Project Overview

The goal of this project is to autonomously detect an empty parking bay using camera feedback, compute a collision-free trajectory, and park a 2-wheeled robot accurately inside the target box.

* **Minimum Viable Product (MVP):** Rule-based computer vision for bay detection, pixel-to-world coordinate projection, kinematic Model Predictive Control (MPC) for trajectory generation, and closed-loop wheel velocity control.
* **Stretch Extensions:** Custom U-Net semantic segmentation for parking line detection and a custom Convolutional Neural Network (CNN) for parking spot occupancy classification.

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
    │   │       Kiesse          │                                 │
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
    │   │       Julian          │                                 │
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
    │   │       Ibrahim         │                                 │
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