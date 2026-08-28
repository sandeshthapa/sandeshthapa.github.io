---
title: "Certified Handoff Between Global Search and Local Planning for Automated Parallel Parking"
collection: publications
permalink: /publication/2026-acc-certified-handoff-parking
date: 2026-09-25
venue: 'In preparation for the American Control Conference (ACC) 2027'
citation: 'Thapa S. "Certified Handoff Between Global Search and Local Planning for Automated Parallel Parking." <i>In preparation for the American Control Conference (ACC) 2027</i>.'
---
A closed-form feasibility certificate for the single-trial reverse parking maneuver, reducing the existence of a curvature-continuous solution from a given entry pose to the sign of a scalar quadratic discriminant — an O(1) test with no iteration or root-finding, and equivalently a line–circle intersection. Evaluated backward from the parked pose, the certificate defines the set of viable entry poses, and that set becomes the goal region of a forward hybrid-A* search, so the global-to-local handoff is feasible by construction rather than by chance. The local planner re-solves from the pose actually reached, absorbing grid-snap error. Validated in a Monte Carlo study against tracked, collision-checked outcomes, with a nonlinear path-frame law executing the maneuver in reverse under steering saturation.