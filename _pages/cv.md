---
layout: archive
title: "Curriculum Vitae"
permalink: /cv/
author_profile: true
redirect_from:
  - /resume
---

{% include base_path %}

<a href="{{ base_path }}/files/Thapa_Sandesh_CV.pdf">Download CV (PDF)</a>

**Robotics & Controls Research Engineer** — safety-critical control and motion planning for autonomous systems and robotics.

# Research Interests

Safety-critical control and motion planning of autonomous and robotic systems; distributed control and estimation; learning and decision making under uncertainty. Methods span nonlinear, adaptive, and optimal control, control barrier functions and Hamilton–Jacobi reachability, trajectory optimization and planning, and multi-vehicle coordination for aerial and ground robot autonomy.

# Education

## M.S., Mechanical & Aerospace Engineering — Oklahoma State University, 2016–2018
* Concentration: **Control Theory & Robotics** (GPA 3.9/4.0)
* Advisor: [Dr. He Bai](https://ceat.okstate.edu/mae/faculty-staff/faculty-bios/he-bai.html); Co-Advisor: [Dr. Jose A. Acosta](https://orcid.org/0000-0003-0040-338X); Committee: [Dr. Rushikesh Kamalapurkar](https://scc-lab.github.io/)
* Thesis: *Cooperative Aerial Manipulation with Force Control and Attitude Stabilization*
* Award: Best Graduate Research, MAE Research Symposium, 2018

## B.S., Mechanical Engineering — McNeese State University, 2011–2015
* Minor: Mathematics (Major GPA 3.8/4.0)

# Publications

<ul>{% for post in site.publications reversed %}
  {% unless post.venue contains "preparation" %}
    {% include archive-single-cv.html %}
  {% endunless %}
{% endfor %}</ul>

## In Preparation

<ul>{% for post in site.publications reversed %}
  {% if post.venue contains "preparation" %}
    {% include archive-single-cv.html %}
  {% endif %}
{% endfor %}</ul>

# Research Experience

## Visiting Researcher — University of Texas at Arlington, ETAIC Lab
*Department of Electrical Engineering | Sep 2026 – Present*

* Research host: Dr. H. Eric Tseng (NAE Member, Distinguished University Professor).
* Design, implementation, and experimental validation of safety-critical control algorithms for autonomous systems and robotics.
* Compositional safety for vehicle dynamics: composing actuator-level guard architectures (input boundedness, integrator boundedness, bumpless transfer) with state-space safety filters (CBF-QP, reachability-based filters) for end-to-end safety under embedded deployment constraints.

## Independent Research — Autonomous Systems
*Jersey City, NJ | Feb 2026 – Aug 2026*

* Design, MATLAB/Simulink implementation, and real-time validation of quadrotor trajectory tracking controllers (PID, state-dependent LQR, geometric SE(3), nonlinear adaptive) in Flightmare, Gazebo, and ROS.
* Formulation of a closed-form single-trial feasibility predicate for clothoid-based planning in low-speed automated parking.
* Preparation of three papers for submission to IEEE RA-L, IEEE IV, and arXiv.

## Tech Lead & Senior Controls Research Engineer — Amogy Inc.
*Brooklyn, NY | Aug 2022 – Feb 2026*

* Designed and deployed a safety-aware discrete-event state-machine wrapper for legacy PID control loops, establishing analytical properties of input boundedness, integrator boundedness, and bumpless mode transfer. Deployed across multiple safety-critical autonomous systems with ISO 26262 functional-safety considerations.
* Head-to-head evaluation of the state-machine wrapper against control-barrier-function safety filters on systems where a clean analytical CBF construction is unavailable due to lack of model or invariant set.
* Research on nonlinear, hybrid, and predictive control frameworks for safety-critical autonomous operation; implementation on automotive-grade real-time hardware (dSPACE, VCU).
* **Tools:** C/C++, Python, MATLAB/Simulink, Stateflow, dSPACE, CAN, TCP/IP.

## Research Engineer, Autonomous Driving (Planning & Controls) — Ford Motor Company, Research & Advanced Engineering
*Dearborn, MI | Jan 2021 – Jun 2022*

* Advisor: Dr. H. Eric Tseng (NAE Member).
* Developed a continuous-curvature (clothoid-based) path planner and a nonlinear rear-wheel feedback lateral controller for autonomous parallel parking and auto-hitch (SAE L2/L3 features).
* Prototyped trajectory tracking controllers including pure-pursuit, LQR, PD, backstepping, and nonlinear feedback control; validated in simulation and on-vehicle in real time.
* Designed state machines and control-barrier-function based safety controllers for autonomous function transitions and fault handling.
* Developed V2V networking and real-time TCP/UDP interfaces for cooperative localization and collision avoidance.
* **Tools:** Simulink, Stateflow, ROS, C/C++, TCP/IP, dSPACE, CANape, CANalyzer.

## Senior Controls Engineer, Autonomy — The Drone Racing League
*New York, NY | Jul 2019 – Jan 2021*

* Led research, design, and implementation of trajectory tracking, attitude stabilization, and trajectory generation algorithms for autonomous quadrotor UAVs.
* Implemented state-dependent LQR/LQG, nonlinear PID, model reference adaptive control (MRAC), geometric SE(3) control, and model predictive control (MPC).
* Built ROS/Gazebo and PX4 SITL simulation environments and conducted real flight tests on quadrotor hardware.
* Implemented C++ real-time solvers for the algebraic Riccati equation using Schur decomposition, eigenvalue decomposition, and Newton iteration.
* **Tools:** C/C++, ROS, PX4, Gazebo, Eigen, CMake, Python, MATLAB/Simulink.

## Graduate Research Assistant, CoRAL Lab — Oklahoma State University
*Stillwater, OK | Aug 2016 – Jun 2019*

* Research on cooperative control and estimation for multi-robot systems, focused on multi-UAV cooperative manipulation of a shared unknown payload.
* Developed decentralized adaptive controllers for cooperative aerial payload transport with no direct inter-robot communication, handling unknown payload mass and drag.
* Developed a concurrent-learning adaptive control algorithm for real-time parameter estimation, relaxing the persistent-excitation requirement while preserving parameter convergence.
* Five peer-reviewed publications: IEEE L-CSS (with CDC presentation), JINT, ACC, ICUAS, IFAC NAASS.

## Robotics Controls Engineer — DEKA Research & Development
*Manchester, NH | Mar 2019 – Jul 2019*

* Autonomous robots, offline path planning, and controls.

# Teaching

<ul>{% for post in site.teaching reversed %}
  {% include archive-single-cv.html %}
{% endfor %}</ul>

# Technical Skills

* **Programming:** C/C++, Python, MATLAB/Simulink, Stateflow
* **Robotics & Simulation:** ROS, Gazebo, PX4-SITL, Flightmare, Crazyflie UAVs, Eigen
* **Real-Time Hardware:** dSPACE, MicroAutoBox II, VCU, Raspberry Pi, Jetson TX2/Xavier
* **Automotive & Communication:** CAN, CANape, CANalyzer, CarSim, TCP/UDP networking, Git

# Certifications

* Functional Safety Certification, 2024
* Udacity: Intro to Self-Driving Cars, 2021; Flying Car and Autonomous Flight Engineer (in progress)

# Awards & Fellowships

* Ross Fellowship, Purdue Aerospace Engineering PhD, Fall 2021 (deferred, later declined)
* Dean's Fellowship, CU Boulder ECE PhD, Fall 2021 (declined)
* Full GRA-supported PhD admissions at Georgia Tech (ECE), UIUC (Aerospace), University of Maryland (Aerospace), and University of Delaware (Mechanical), Fall 2021 (declined)
* Best Graduate Research, MAE Research Symposium, Oklahoma State University, 2018
* Winner, Nautilus Engineering Design Challenge, 2014–2015
* First Place, McNeese Undergraduate Research Competition, 2015
* H.R. Smith Engineering Scholarship

# Graduate Coursework

Linear Systems · Nonlinear System Analysis and Control · Robotics: Kinematics, Dynamics, and Control · Digital Control Systems · Atmospheric Flight Control · Guidance and Control of Aerospace Vehicles · Stochastic Systems · Intro to Modern Analysis · Optimal Control (audit) · Adaptive Control (audit)

# Peer Reviewer

* American Control Conference (ACC): 2017–2022, 2025
* IEEE Conference on Decision and Control (CDC): 2018, 2019, 2021, 2022
* IEEE International Conference on Unmanned Aircraft Systems (ICUAS): 2018, 2021
* IEEE International Conference on Robotics and Automation (ICRA): 2018, 2020, 2021
* Journal of Intelligent & Robotic Systems (JINT): 2020

# References

Available upon request.
