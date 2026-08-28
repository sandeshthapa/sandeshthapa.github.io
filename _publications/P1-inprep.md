---
title: "A Practical State-Machine Based Event PID Controller"
collection: publications
permalink: /publication/2026-acc-statemachine-event-pid
date: 2026-09-25
venue: 'In preparation for the American Control Conference (ACC) 2027'
citation: 'Thapa S. "A Practical State-Machine Based Event PID Controller." <i>In preparation for the American Control Conference (ACC) 2027</i>.'
---
An event-triggered PID in which the hold is a state of a finite state machine rather than a compensation term, so the integral term is held whenever the command is held and no maximum sampling period is required. The hold band is calibrated from the actuator deadband and the sensor resolution measured in a single constant-command field test, requiring no plant model. On the published benchmarks of Durand and Marchand the controller reproduces the reference update counts exactly and matches the hybrid event-based PID at identical gains, with a further reduction in updates under measurement noise. Includes a field deployment of the code-generated controller on production hardware.