---
title: "Research"
permalink: /research/
author_profile: true
---

{% include base_path %}

My research develops **safety-critical control and motion planning for robots and multi-agent systems**, pairing rigorous control theory with hands-on validation on real hardware. The projects below are ordered from my current work back to my earliest research.

Table of Contents
* Auto generated table of contents
{:toc}

# Safety-Critical Control — Current Research
*Visiting Researcher, ETAIC Lab, UT Arlington · Tech Lead & Senior Controls Research Engineer, Amogy*

Composing actuator-level safety architectures with state-space safety filters for end-to-end safety under embedded, real-time constraints.

* Safety certificates for low-level control loops.
* Safety-critical control for multi-agent systems.
* Safe control in the presence of model uncertainty and unmodeled dynamics.
* Safety-aware state-machine wrapper for legacy PID loops with provable input-boundedness, integrator-boundedness, and bumpless-transfer properties, evaluated head-to-head against control-barrier-function safety filters *(in preparation, IEEE RA-L)*.

![Safety-embedded PID control](../images/safe_pid.png)
![Safety-embedded cooperative control](../images/safe_control_v4.png)
![Safe cooperative multi-agent control](../images/safe_coop.png)
![Control barrier functions](../images/CBFS.png)

# Autonomous Vehicle Planning and Control
*Research Engineer, Ford Motor Company, Research & Advanced Engineering (advisor: Dr. H. Eric Tseng, NAE Member)*

* Continuous-curvature (clothoid-based) path planner and **nonlinear rear-wheel feedback lateral controller** for autonomous parallel parking and auto-hitch (SAE L2/L3).
* State machines and **control-barrier-function safety certificates** for autonomous function transitions and fault handling.
* Trajectory-tracking benchmarks (pure-pursuit, LQR, PD, backstepping, nonlinear feedback); validated in simulation and on dSPACE real-time hardware.

![Parallel parking setup](../images/parallel_parking.png)

## Clothoid-Based Path Planning
![Clothoid path planning](../images/planning.png)

## Non-linear Rear-Wheel Feedback Control
![Control schematic](../images/control.png)
![Results](../images/results.png)

## Vehicle Dynamics with Cruise and Lateral Control
<video width="100%" controls autoplay loop muted>
  <source src="../images/Vehicle_Dynamics_and_Cruise_Control.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Pure-Pursuit Control
<video width="100%" controls autoplay loop muted>
  <source src="../images/pure_pursuit.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

# Autonomous Quadrotor UAV Control
*Senior Controls Engineer, The Drone Racing League · Independent Research*

* Cascaded PID, state-dependent LQR, nonlinear Lyapunov-based, sliding-mode, backstepping, geometric SE(3), and Cartesian impedance controllers for quadrotor trajectory tracking.
* Geometric attitude control and differential-flatness-based minimum-snap trajectory generation.
* End-to-end autopilot integrating state estimation, planning, and control, validated in MATLAB/Simulink, Gazebo, PX4 SITL, ROS, and on real quadrotor hardware.

![UAV Autonomy](../images/drone_achitecture.png)

## Cascaded PID Control in PX4
<video width="100%" controls autoplay loop muted>
  <source src="../images/PX4_PID_Control.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## State-Dependent LQR for Trajectory Tracking
Full-state time-varying LQR designed and implemented in real time in Gazebo and PX4.
![LQR Control](../images/LQR_Control%20.png)

<video width="100%" controls autoplay loop muted>
  <source src="../images/LQR_control.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Offboard Velocity Control
<video width="100%" controls autoplay loop muted>
  <source src="../images/offboard_velocity_px4_gazebo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Minimum-Snap 3D Trajectory Generation
![Trajectory Generation](../images/trajGen.png)

<video width="100%" controls autoplay loop muted>
  <source src="../images/Minimum_Snap_Trajectory_Generation_Simulation.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

# Distributed Control, Coordination, and Manipulation
*M.S. research, Oklahoma State University (CoRAL Lab, Dr. He Bai), jointly with Dr. J. Á. Acosta (University of Seville, Spain)*

* Decentralized multi-robot control framework for cooperative aerial manipulation: multiple quadrotor UAVs transport a shared payload **without constant inter-robot communication**.
* Adaptive force-sharing controllers regulate payload forces while all agents coordinate their motion, with stable transport under unknown payload mass and external disturbances — validated in simulation and physical flight tests.

![Cooperative control](../images/newagents4.png)

**Related publications:**
* **[J2]** Thapa S., Bai H., Acosta J.A. *Cooperative Aerial Manipulation with Decentralized Adaptive Force-Consensus Control.* Journal of Intelligent & Robotic Systems (JINT), 2020.
* **[C1]** Thapa S., Bai H., Acosta J.A. *Cooperative Aerial Load Transport with Attitude Stabilization.* American Control Conference (ACC), 2018.
* **[C2]** Thapa S., Bai H., Acosta J.A. *Force Control in Cooperative Aerial Manipulation.* IEEE ICUAS, 2018.
* **[C3]** Thapa S., Bai H., Acosta J.A. *Cooperative Aerial Load Transport with Force Control.* IFAC NAASS, 2018.

## Cooperative Manipulation of an Unknown Payload
<video width="100%" controls autoplay loop muted>
  <source src="../images/KnownMass5.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Cooperative Attitude Control
<video width="100%" controls autoplay loop muted>
  <source src="../images/Anim_new_control.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Cooperative Control with Time-Varying Velocity
<div align="center">
  <iframe width="560" height="315"
  src="https://www.youtube.com/embed/tDgRc_d6Nqo"
  title="Cooperative control with time-varying velocity"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
  allowfullscreen>
  </iframe>
</div>

## Aerial Manipulator Flight Test
<div align="center">
  <iframe width="560" height="315"
  src="https://www.youtube.com/embed/vBqVEjUz4NM"
  title="Aerial manipulator flight test"
  frameborder="0"
  allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
  </iframe>
</div>

# Learning for Control and Estimation
*M.S. research, Oklahoma State University (with Dr. Rushikesh Kamalapurkar)*

* **Concurrent-learning** adaptive control for a team of robots transporting a common load, enabling real-time estimation of unknown parameters (payload mass and drag) while driving all agents and the payload to a desired trajectory.
* Guarantees parameter convergence and improves transient performance by reusing past data to relax excitation requirements, with accurate force regulation and synchronized motion in simulation.

* **[J1]** Thapa S., Self R., Bai H., Kamalapurkar R. *Cooperative Manipulation of an Unknown Payload with Concurrent Mass and Drag Force Estimation.* IEEE Control Systems Letters (L-CSS), with CDC presentation option, 2019.

## Drag Force Estimation
![Drag force estimation](../images/VelLoadB.png)

## Contact Force on the Payload
![Contact force estimation](../images/f1dTildeB.png)

## Non-linear Adaptive Geometric Control
![Adaptive control schematic](../images/adaptivecontrol.png)
![Adaptive control results](../images/adapresult.png)
