# Process Checklist

> **Global rule**: Before marking any story complete, write
> `artifacts/<step_id>/execution_log.md` and invoke `paper-alignment-verifier`
> with the execution log path, artifact paths, and paper PDF path.
> A story may only be marked complete when the verifier returns PASS or ACCEPTED DEVIATION.

## S01 — Setup Formation Control Framework
- **executor**: agent
- **description**: Implement the bearing-based formation control framework including graph topology definition, bearing computation, and projection matrix construction. Output: Python module with core classes/functions.
- **paper_source**: Section II, pp. 3-4 (Problem formulation, bearing definition, projection matrix)
- **dependencies**: []
- **type**: process
- **artifact_path**: artifacts/S01/formation_framework.py

---

## S02 — Implement Bearing Laplacian Matrix
- **executor**: agent
- **description**: Implement the bearing Laplacian matrix B and its submatrix B_ff. Compute eigenvalues and verify nonsingularity condition for target formation uniqueness. Output: bearing_laplacian.py with matrix construction functions.
- **paper_source**: Section II-B, pp. 4-5 (Bearing Laplacian definition, Lemma 1, Lemma 2, Theorem 1)
- **dependencies**: [S01]
- **type**: process
- **artifact_path**: artifacts/S02/bearing_laplacian.py

---

## S03 — Implement Control Law (7) for Constant Leader Velocity
- **executor**: agent
- **description**: Implement the bearing-based formation control law (7) for double-integrator dynamics with constant leader velocity. Output: control_constant_velocity.py with control function.
- **paper_source**: Section III-A, pp. 5-6 (Control law 7, Theorem 3)
- **dependencies**: [S01, S02]
- **type**: process
- **artifact_path**: artifacts/S03/control_constant_velocity.py

---

## S04 — Simulate Fig. 4: Square Formation with Constant Leader Velocity
- **executor**: agent
- **description**: Run numerical simulation reproducing Fig. 4: 2D square formation (2 leaders, 2 followers) with control law (7). Generate trajectory plot, bearing error plot, and velocity plot. Output: figures matching Fig. 4(a), (b), (c).
- **paper_source**: Section III-A, Fig. 4 (`./extracted/images/figure_24.png`, `./extracted/images/figure_7.png`, `./extracted/images/figure_32.png`, `./extracted/images/figure_19.png`), pp. 6-7
- **dependencies**: [S03]
- **type**: result
- **artifact_path**: artifacts/S04/fig4_simulation.py

---

## S05 — Implement Control Law (10) for Time-Varying Leader Velocity
- **executor**: agent
- **description**: Implement the bearing-based formation control law (10) with acceleration feedback for time-varying leader velocity. Output: control_time_varying.py with control function.
- **paper_source**: Section III-B, pp. 7-8 (Control law 10, Lemma 3, Theorem 4)
- **dependencies**: [S01, S02]
- **type**: process
- **artifact_path**: artifacts/S05/control_time_varying.py

---

## S06 — Simulate Fig. 5: 3D Cube Formation with Time-Varying Leader Velocity
- **executor**: agent
- **description**: Run numerical simulation reproducing Fig. 5: 3D cube formation (2 leaders, 6 followers) with control law (10). Generate trajectory plot, bearing error plot, and velocity plot. Output: figures matching Fig. 5(a), (b), (c).
- **paper_source**: Section III-B, Fig. 5 (`./extracted/images/figure_68.png`, `./extracted/images/figure_8.png`, `./extracted/images/figure_4.png`, `./extracted/images/figure_18.png`, `./extracted/images/figure_14.png`), pp. 8-9
- **dependencies**: [S05]
- **type**: result
- **artifact_path**: artifacts/S06/fig5_simulation.py

---

## S07 — Implement Control Law (12) with Integral Control for Input Disturbances
- **executor**: agent
- **description**: Implement the bearing-based formation control law (12) with integral term to handle constant input disturbances. Output: control_integral.py with control function.
- **paper_source**: Section IV-A, pp. 9-10 (Control law 12, Theorem 5, Theorem 6)
- **dependencies**: [S01, S02]
- **type**: process
- **artifact_path**: artifacts/S07/control_integral.py

---

## S08 — Implement Control Law (15) with Saturation for Acceleration Saturation
- **executor**: agent
- **description**: Implement the bearing-based formation control law (15) with saturation function for bounded acceleration input. Output: control_saturated.py with control function.
- **paper_source**: Section IV-B, pp. 10-12 (Control law 15, Lemma 4, Lemma 5, Theorem 7, Theorem 8)
- **dependencies**: [S01, S02]
- **type**: process
- **artifact_path**: artifacts/S08/control_saturated.py

---

## S09 — Verify Collision Avoidance Conditions
- **executor**: agent
- **description**: Implement and verify the collision-free conditions from Theorems 9 and 10. Check if initial conditions satisfy the sufficient conditions for collision avoidance. Output: collision_check.py with verification functions.
- **paper_source**: Section IV-C, pp. 12-13 (Theorem 9, Theorem 10)
- **dependencies**: [S03, S05]
- **type**: process
- **artifact_path**: artifacts/S09/collision_check.py

---

## S10 — Generate Result Comparison Report
- **executor**: agent
- **description**: Compare all reproduced figures and results against the original paper. Generate structured comparison report with PASS/FAIL verdicts. Output: result_comparison_report.md.
- **paper_source**: All simulation figures (Fig. 4 (`./extracted/images/figure_24.png`, `./extracted/images/figure_7.png`, `./extracted/images/figure_32.png`, `./extracted/images/figure_19.png`), Fig. 5 (`./extracted/images/figure_68.png`, `./extracted/images/figure_8.png`, `./extracted/images/figure_4.png`, `./extracted/images/figure_18.png`, `./extracted/images/figure_14.png`))
- **dependencies**: [S04, S06, S09]
- **type**: result
- **artifact_path**: artifacts/S10/result_comparison_report.md
