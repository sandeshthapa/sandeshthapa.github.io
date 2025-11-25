---
layout: archive
title: " Bio "
permalink: /cv/
author_profile: true
redirect_from:
  - /resume
---

{% include base_path %}

# Education
## B.S. in Mechanical Engineering, McNeese State University, 2015
## M.S. in Mechanical & Aerospace Engineering, Oklahoma State University, 2018
  * Robotics & Controls 

# Work experience
## Technical Lead, Controls Software
**Amogy INC**, Brooklyn, NY | *Aug 2022 – Present*

* Control Algorithms development, testing, and validation of the fully autonomous Amogy System.
* Developed novel **PID Control Algorithms** for trajectory tracking that reduce actuator chattering and account for sensor error accuracy.
* Lead Controls Engineering including Hardware & Software Development, Controls Algorithms, State Machine Development, and Network Architecture.
* Developed advanced control algorithms: **Nonlinear control, Hybrid controls, Filter design, and Model Predictive Control (MPC)**.
* Designed advanced networking for redundancy, communications, and data processing.
* Designed HMI systems and implemented noise reduction, filtering, and fault management.
* Developed data structures to integrate different devices for advanced communication, control, and HMI.
* Configured and integrated communication protocols (CAN, Ethernet TCP, Modbus TCP).
* Spearheaded hardware design, including VCU, Raspberry Pi, and dSPACE systems.
* Guided a multidisciplinary team, ensuring compliance with **functional safety standards**.
* Advise and mentor junior controls, HIL, embedded, and electrical engineers.
* **Tools:** C/C++, Python, MATLAB, CAN, CANape, CANalyzer, dSPACE, Code Generation, TCP/IP Networking.

## Research Engineer (Autonomous Driving - Planning and Controls)
**Ford Motor Company**, Dearborn, MI | *Jan 2021 – Jun 2022*

* Research in advanced controls and planning; designed low-speed path planning and control algorithms for autonomous parallel parking, auto-hitch, nudge maneuvers, state machines, and V2V localization/collision-avoidance.
* Designed and implemented a **continuous curvature-based path planner** and **nonlinear rear-wheel feedback lateral controller** for auto-hitch and parallel parking (L2 autonomy).
* Created state machines for complex state transitions and management during parking and auto-hitch maneuvers.
* Created **Vehicle-to-Vehicle (V2V) networking** and TCP/IP UDP data interfaces for real-time data streaming between multiple Ford vehicles for localization and collision avoidance.
* Conducted testing, validation, and refinement of autonomous driving features.
* **Tools:** Simulink, Stateflow, ROS, C/C++, TCP/IP, dSPACE, CANape, CANalyzer.

## Senior Controls Engineer, Autonomy
**The Drone Racing League**, New York, NY | *Jul 2019 – Jan 2021*

* Led research, development, and implementation of control algorithms for autonomous drones, focusing on position and attitude control for trajectory tracking.
* Designed and implemented advanced control strategies, including **State-Dependent LQR, LQR/LQG, Nonlinear PID, Adaptive Control (MRAC), Geometric Control, and MPC**, ensuring robust trajectory tracking and attitude stabilization.
* Developed and analyzed trajectory tracking performance using ROS/Gazebo and PX4 SITL simulations with PID and LQR controllers.
* Implemented solvers for **Algebraic Riccati Equations in C++** to support control algorithm development.
* Conducted real-time flight testing, algorithm testing, and debugging for autonomous flight.
* **Tools:** C/C++, ROS, PX4, Gazebo, Eigen, CMake, Python, MATLAB/Simulink.

## Robotics Controls Engineer - March 2019- July 2019 
  * Deka Research and Development , Manchester, NH 
  * Autonomous robots offline path planning and controls
# Skills

* C/C++ 
* MATLAB/Simulink, State flow 
* Python
* Communication: CAN, Ethernet UDP, TCIP 

# Publications

  <ul>{% for post in site.publications %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul>
  
Talks
======
  <ul>{% for post in site.talks %}
    {% include archive-single-talk-cv.html %}
  {% endfor %}</ul>
  
Teaching
======
  <ul>{% for post in site.teaching %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul>
  
Service and leadership
======
* Currently signed in to 43 different slack teams
