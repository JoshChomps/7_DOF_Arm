# 23: Singularities and cyclic coordinate descent (a study, 26 Oct)

Josh asked to look into singularities and "cycloid coordinate descent" while the parts ship. Read as **cyclic
coordinate descent (CCD)**, the IK method that turns one joint at a time; there is no IK method named after the
cycloidal gearboxes. Everything here was run in simulation with `tools/ik_study.py`; the derivations and the full tables
are in [14](14_math_guide.md) §21. Nothing in `deskarm` was changed.

## Short version

- **With all seven joints free, the arm has fewer singularities than the twin warns about.** The wrist singularity
  ($q_6=0$) and the shoulder one (wrist centre on the J1 axis) are real only with J3 locked. With J3 free, the arm is
  singular only when the elbow is straight ($q_4=0$) or when **$q_2=0$ and $q_6=0$ at the same time** (J1 in line with
  J3, and J5 in line with J7). The twin's `singularity_warnings` checks the three 6-DOF cases in both modes, so in 7-DOF
  mode it raises two false alarms and misses the combined case.
- **The fixed damping (λ = 0.02) is fine.** An adaptive rule that damps only near a singularity changed the worst joint
  step on a path through $q_6=0$ from 2.40° to 2.36°. In 6-DOF mode the wrist has to flip J5 and J7 near $q_6=0$ however
  the step is damped; in 7-DOF mode the same line never gets near a singularity (0.27° per step at most).
- **CCD is not worth adopting for this arm.** For the full pose it solved half the targets that DLS solved, about 45×
  slower (140 ms against 3 ms), because its convergence is linear and the parallel pitch axes fight each other. As a
  warm start it lifts far-seed success from 77.5% to 84.5%, but `solve_ik`'s existing restart from the closed-form 6-DOF
  solutions already reaches 100%. For position-only targets it converges, but swings the joints by ~180° where DLS moves
  them 38-72°.

## 1. Singularities with seven joints

The test builds 300 poses that sit exactly on each candidate condition (other joints random within 90% of their limits)
and takes the smallest singular value of the weighted pose Jacobian, the same Jacobian `solve_dls` inverts. Zero means
singular.

| Condition | All 7 joints | J3 locked (6-DOF mode) |
|---|---|---|
| random poses | 0.036 | 0.026 |
| $q_4=0$, arm straight | **0** | **0** |
| $q_6=0$, J5 and J7 in line | 0.016 | **0** |
| $q_2=0$, J1 and J3 in line | 0.012 | 0.011 |
| $q_2=0$ and $q_6=0$ | **0** | **0** |
| wrist centre on the J1 axis | 0.051 | **0** |

Why ([14](14_math_guide.md) §21 Steps 1-2): two joints whose axes are the same line have identical Jacobian columns.
With J3 locked, $q_6=0$ leaves six columns but five directions. With J3 free, J3 supplies the sixth direction, unless
$q_2=0$ makes J3's axis J1's as well. A straight elbow is singular either way: no joint can move the wrist centre
along the line from the shoulder.

This matches the general result for arms of this layout (Kreutz-Delgado, Long and Seraji, 1992).

## 2. Damping through a singularity

A straight tool line from $q=(10, 40, 0, 70, 30, 12, -20)$° to $(10, 40, 0, 70, -30, -12, 20)$° makes the IK cross
$q_6=0$. Solved in 120 steps, warm-started each time:

| Solver | Largest joint step | Largest tool error | Smallest $\sigma_{\min}$ on the way |
|---|---|---|---|
| 6-DOF, λ = 0.02 (today) | 2.40° | 0.10 mm | 0.0064 |
| 6-DOF, λ = 0.005 | 2.84° | 0.10 mm | 0.0064 |
| 6-DOF, adaptive ($\varepsilon=\lambda_{\max}=0.04$) | 2.36° | 0.10 mm | 0.0064 |
| 7-DOF, λ = 0.02 | 0.27° | 0.04 mm | 0.039 |

The average step is about 0.5°, so 6-DOF mode spikes about five times above it near the crossing. `linear_move`
re-times the path so no joint exceeds its speed limit (§8 of the guide), so the spike costs time, not safety.

## 3. Cyclic coordinate descent

`ik_study.ccd` turns each joint, from the tool inward, by the exact angle that best matches the TCP position and the
tool's three axes to the target ([14](14_math_guide.md) §21 Step 4: $t^\ast=\operatorname{atan2}(C, B)$), clipped to the
joint's limits, and repeats the sweep. 200 random reachable targets per row; success means within 0.1 mm and 1 mrad;
medians over the solved targets:

| Task, seed distance | DLS (`solve_dls`) | CCD | CCD to 5 mm, then DLS |
|---|---|---|---|
| pose, 0.6 rad | 99.0%, 14 iterations, 3.1 ms | 50.5%, 160 sweeps, 140 ms | 98.0%, 47 ms |
| pose, 2.0 rad | 77.5%, 35 iterations, 7.9 ms | 43.0%, 192 sweeps, 154 ms | 84.5%, 49 ms |
| position, 0.6 rad | 100%, 8 iterations, 1.8 ms | 97.5%, 16 sweeps, 12.5 ms | 99.5%, 6.0 ms |
| position, 2.0 rad | 96.5%, 16 iterations, 3.8 ms | 95.0%, 24 sweeps, 20.5 ms | 97.0%, 9.7 ms |

`solve_ik` (what the twin calls: DLS, then DLS restarted from each closed-form 6-DOF solution if it fails) solved
**all 200** of the 2.0 rad pose targets, median 10.9 ms, 95th percentile 88 ms.

What CCD is good at: every step lowers the error, it needs no matrix inverse, and its first sweep gains a lot (488 →
181 mm in the guide's figure, against DLS's 462 mm, which is held to 0.2 rad per step). That is why it is common in
animation and for long chains. What it's bad at here: near the answer it is linear (101 sweeps in that figure, against
13 DLS iterations), it has no null space to steer the elbow or keep joints off their limits, and with the orientation
free it moves the distal joints far. This arm has a closed-form 6-DOF solution to seed from, so the case where CCD
helps most, a far seed with no good guess, is already covered.

## 4. Recommendations

1. **Keep DLS with the closed-form restarts** as the solver. Don't add CCD.
2. **Keep λ = 0.02.** The adaptive rule made no measurable difference.
3. **Make the singularity warnings mode-aware** (proposed, not done). In 6-DOF mode keep the three checks. In 7-DOF mode
   warn on the straight elbow and on $q_2\approx0$ together with $q_6\approx0$, and drop the wrist-only and shoulder
   warnings, which are false alarms there. A simpler and complete alternative: warn when $\sigma_{\min}$ of the
   lock-reduced Jacobian (`ik_study.sigma_min`) falls below ~0.005, and name the cause from the conditions above. That
   also covers joint locks other than J3, which the fixed checks ignore.
4. For straight-line moves near the wrist singularity, prefer 7-DOF mode.

## 5. What this does not cover

- Joint limits as a source of trouble (the IK clips at limits; near a limit DLS can stall where the pose is reachable
  another way). `solve_ik`'s null-space centering exists for that and was not studied here.
- The closed-form arm-angle solution (Shimizu et al., 2008), still planned ([05](05_software_kinematics.md)).
- Anything on hardware: these are kinematics only, with no timing on the Pi.
