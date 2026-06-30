# Add GIF Animations to Simulation Scripts

## TL;DR

> **Quick Summary**: Add animated GIF output with trajectory trails to the two existing simulation scripts (fig4 2D square formation, fig5 3D cube formation).
>
> **Deliverables**:
> - Modified `fig4_simulation.py` with GIF generation (2D, 4 agents)
> - Modified `fig5_simulation.py` with GIF generation (3D, 8 agents)
>
> **Estimated Effort**: Quick
> **Parallel Execution**: YES - 2 waves
> **Critical Path**: Task 1 → Task 2 → F1

---

## Context

### Original Request
User wants to add GIF animation features to `fig4_simulation.py` and `fig5_simulation.py` that show agents moving over time with fading trajectory trails.

### Interview Summary
**Key Discussions**:
- Animation style: Moving agents with fading trajectory trails behind each agent
- Quality: Standard (15 fps, ~10 seconds)
- Both scripts already have `positions_history` arrays and static PNG output

**Research Findings**:
- Pillow is available in the environment for GIF frame assembly
- matplotlib is available for rendering frames
- Both scripts use `positions_history` with shape `(n_steps, n_agents, dims)`

---

## Work Objectives

### Core Objective
Add a `generate_gif()` function to each simulation script that produces an animated GIF showing the formation moving over time with fading trajectory trails.

### Concrete Deliverables
- Modified `outputs/artifacts/S04/fig4_simulation.py` — adds `generate_gif()` function and GIF call in `__main__`
- Modified `outputs/artifacts/S06/fig5_simulation.py` — adds `generate_gif()` function and GIF call in `__main__`
- Output GIF: `outputs/artifacts/S04/fig4_simulation.gif`
- Output GIF: `outputs/artifacts/S06/fig5_simulation.gif`

### Definition of Done
- [ ] `fig4_simulation.py` produces `fig4_simulation.gif` when run
- [ ] `fig5_simulation.py` produces `fig5_simulation.gif` when run
- [ ] GIFs show agents moving with trajectory trails fading from opaque to transparent
- [ ] Leaders are red triangles, followers are blue circles (matching existing style)
- [ ] Edges connect agents at each frame

### Must Have
- Fading trajectory trails behind each agent
- Leaders as red triangles, followers as blue circles
- Edges connecting agents at each frame
- 15 fps, ~10 second duration
- GIF saved to same directory as existing PNG outputs

### Must NOT Have (Guardrails)
- Do NOT modify the existing `run_simulation()` or `run_simulation_3d()` functions
- Do NOT modify the existing `plot_results()` or `plot_results_3d()` functions
- Do NOT change the simulation logic or parameters
- Do NOT add new dependencies beyond matplotlib and Pillow (both already available)
- Do NOT create separate animation files — add functions to existing scripts

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** - ALL verification is agent-executed.

### Test Decision
- **Infrastructure exists**: NO (no test framework for these scripts)
- **Automated tests**: None
- **Framework**: none

### QA Policy
Every task includes agent-executed QA scenarios.
Evidence saved to `.omo/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Scripts**: Use Bash — run the script, verify GIF output exists and has reasonable size

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately):
├── Task 1: Add GIF animation to fig4_simulation.py (2D) [quick]
└── Task 2: Add GIF animation to fig5_simulation.py (3D) [quick]

Wave FINAL (After ALL tasks):
├── F1: Verify both scripts produce valid GIFs [quick]

Critical Path: Task 1/2 → F1
Parallel Speedup: ~50% (2 tasks in parallel)
Max Concurrent: 2
```

### Dependency Matrix

- **1**: None → F1
- **2**: None → F1
- **F1**: 1, 2 → None

### Agent Dispatch Summary

- **Wave 1**: 2 tasks — T1 → `quick`, T2 → `quick`
- **Wave FINAL**: 1 task — F1 → `quick`

---

## TODOs

- [x] 1. Add GIF animation to fig4_simulation.py (2D square formation)

  **What to do**:
  - Add a `generate_gif()` function after the existing `plot_results()` function
  - The function should:
    1. Run the simulation to get `positions_history`
    2. Create frames at intervals (sample ~150 frames for 15fps × 10s)
    3. For each frame:
       - Draw edges connecting agents (black lines, matching `snapshot_edges` from `plot_results`)
       - Draw fading trajectory trails: for each agent, plot the past N positions with decreasing alpha
       - Draw current agent positions: leaders as red `>` markers, followers as blue `o` markers
    4. Use `PIL.Image` to assemble frames into animated GIF
  - Add `generate_gif()` call in `__main__` block after `plot_results()`
  - Use trail length of ~50 frames (0.5 seconds of history visible at any time)
  - Trail alpha: linearly fade from 0.8 to 0.0 over the trail length

  **Must NOT do**:
  - Do NOT modify `run_simulation()`, `plot_results()`, or any existing functions
  - Do NOT change simulation parameters or logic

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single file modification, clear requirements, no complex logic
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Task 2)
  - **Blocks**: F1
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `outputs/artifacts/S04/fig4_simulation.py:175-265` — Existing `plot_results()` function showing how agents are plotted (red `>` for leaders, blue `o` for followers, edge drawing pattern)
  - `outputs/artifacts/S04/fig4_simulation.py:181` — `snapshot_edges` list defining which edges to draw
  - `outputs/artifacts/S04/fig4_simulation.py:104-172` — `run_simulation()` showing `positions_history` shape `(n_steps, 4, 2)`

  **API/Type References**:
  - `positions_history` shape: `(n_steps, 4, 2)` — agents × xy coordinates
  - `time_array` — evenly spaced time values

  **External References**:
  - `PIL.Image.fromarray().save('out.gif', save_all=True, append_images=[...], duration=67)` — GIF assembly at ~15fps (67ms per frame)

  **WHY Each Reference Matters**:
  - `plot_results()` shows the exact marker styles and edge drawing pattern to replicate
  - `snapshot_edges` defines which agent pairs to connect
  - `positions_history` shape determines how to index frames

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Run fig4_simulation.py and verify GIF output
    Tool: Bash
    Preconditions: Script exists at outputs/artifacts/S04/fig4_simulation.py
    Steps:
      1. cd /home/ronghai/bearing_control && python3 outputs/artifacts/S04/fig4_simulation.py
      2. ls -la outputs/artifacts/S04/fig4_simulation.gif
      3. python3 -c "from PIL import Image; img = Image.open('outputs/artifacts/S04/fig4_simulation.gif'); print(f'Frames: {img.n_frames}, Size: {img.size}')"
    Expected Result: GIF exists, has ~150 frames, size is reasonable (>100KB, <50MB)
    Failure Indicators: File not found, 0 frames, ImportError for Pillow
    Evidence: .omo/evidence/task-1-fig4-gif-output.txt

  Scenario: Verify GIF shows trajectory trails
    Tool: Bash
    Preconditions: GIF generated from previous step
    Steps:
      1. python3 -c "
         from PIL import Image
         img = Image.open('outputs/artifacts/S04/fig4_simulation.gif')
         img.seek(75)  # middle frame
         img.save('/tmp/fig4_frame75.png')
         print('Frame 75 saved')
         "
    Expected Result: Frame 75 is a valid PNG image showing agents with trails
    Failure Indicators: Exception on seek, empty image
    Evidence: .omo/evidence/task-1-fig4-frame75.png
  ```

  **Evidence to Capture:**
  - [ ] task-1-fig4-gif-output.txt — terminal output from running script
  - [ ] task-1-fig4-frame75.png — sample frame from middle of GIF

  **Commit**: YES
  - Message: `feat(S04): add animated GIF with trajectory trails to fig4 simulation`
  - Files: `outputs/artifacts/S04/fig4_simulation.py`
  - Pre-commit: `python3 outputs/artifacts/S04/fig4_simulation.py`

---

- [x] 2. Add GIF animation to fig5_simulation.py (3D cube formation)

  **What to do**:
  - Add a `generate_gif()` function after the existing `plot_curves()` function
  - The function should:
    1. Run the simulation to get `positions_history`
    2. Create frames at intervals (~150 frames for 15fps × 10s)
    3. For each frame in 3D:
       - Draw edges connecting agents (black lines, matching `snapshot_edges` from `plot_results_3d`)
       - Draw fading trajectory trails: for each agent, plot past N positions with decreasing alpha
       - Draw current agent positions: leaders as red `>` markers, followers as blue `o` markers
       - Draw the obstacle wall (reuse `draw_obstacle()` function)
    4. Use `PIL.Image` to assemble frames into animated GIF
  - Add `generate_gif()` call in `__main__` block after `plot_curves()`
  - Use trail length of ~50 frames
  - Trail alpha: linearly fade from 0.8 to 0.0
  - Keep the same 3D view angle (`elev=15, azim=-20`) for all frames

  **Must NOT do**:
  - Do NOT modify `run_simulation_3d()`, `plot_results_3d()`, `plot_curves()`, or any existing functions
  - Do NOT change simulation parameters or logic
  - Do NOT modify `draw_obstacle()` — reuse it as-is

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single file modification, clear requirements, mirrors Task 1 but in 3D
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Task 1)
  - **Blocks**: F1
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `outputs/artifacts/S06/fig5_simulation.py:251-323` — Existing `plot_results_3d()` showing 3D plotting pattern, marker styles, edge drawing
  - `outputs/artifacts/S06/fig5_simulation.py:260-268` — `snapshot_edges` for the 3D cube
  - `outputs/artifacts/S06/fig5_simulation.py:219-248` — `draw_obstacle()` function to reuse for wall rendering
  - `outputs/artifacts/S06/fig5_simulation.py:154-216` — `run_simulation_3d()` showing `positions_history` shape `(n_steps, 8, 3)`

  **API/Type References**:
  - `positions_history` shape: `(n_steps, 8, 3)` — agents × xyz coordinates
  - `Poly3DCollection` from `mpl_toolkits.mplot3d.art3d` — used for obstacle panels

  **External References**:
  - `PIL.Image.fromarray().save('out.gif', save_all=True, append_images=[...], duration=67)` — GIF assembly

  **WHY Each Reference Matters**:
  - `plot_results_3d()` shows the exact 3D plotting pattern with scatter markers
  - `snapshot_edges` defines which agent pairs to connect in 3D
  - `draw_obstacle()` must be included in each frame for context
  - `positions_history` shape `(n_steps, 8, 3)` determines indexing

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Run fig5_simulation.py and verify GIF output
    Tool: Bash
    Preconditions: Script exists at outputs/artifacts/S06/fig5_simulation.py
    Steps:
      1. cd /home/ronghai/bearing_control && python3 outputs/artifacts/S06/fig5_simulation.py
      2. ls -la outputs/artifacts/S06/fig5_simulation.gif
      3. python3 -c "from PIL import Image; img = Image.open('outputs/artifacts/S06/fig5_simulation.gif'); print(f'Frames: {img.n_frames}, Size: {img.size}')"
    Expected Result: GIF exists, has ~150 frames, size is reasonable (>100KB, <50MB)
    Failure Indicators: File not found, 0 frames, ImportError for Pillow
    Evidence: .omo/evidence/task-2-fig5-gif-output.txt

  Scenario: Verify 3D GIF shows obstacle and trails
    Tool: Bash
    Preconditions: GIF generated from previous step
    Steps:
      1. python3 -c "
         from PIL import Image
         img = Image.open('outputs/artifacts/S06/fig5_simulation.gif')
         img.seek(75)
         img.save('/tmp/fig5_frame75.png')
         print('Frame 75 saved')
         "
    Expected Result: Frame 75 shows 3D cube formation with obstacle wall and trails
    Failure Indicators: Exception on seek, empty image
    Evidence: .omo/evidence/task-2-fig5-frame75.png
  ```

  **Evidence to Capture:**
  - [ ] task-2-fig5-gif-output.txt — terminal output from running script
  - [ ] task-2-fig5-frame75.png — sample frame from middle of GIF

  **Commit**: YES
  - Message: `feat(S06): add animated GIF with trajectory trails to fig5 simulation`
  - Files: `outputs/artifacts/S06/fig5_simulation.py`
  - Pre-commit: `python3 outputs/artifacts/S06/fig5_simulation.py`

---

## Final Verification Wave

- [x] F1. **Both GIFs Valid** — `quick`
  Run both scripts. Verify both GIFs exist, have ~150 frames, show agents moving with trails. Check file sizes are reasonable.
  Output: `fig4 GIF [PASS/FAIL] | fig5 GIF [PASS/FAIL] | VERDICT`

---

## Commit Strategy

- **Task 1**: `feat(S04): add animated GIF with trajectory trails to fig4 simulation`
- **Task 2**: `feat(S06): add animated GIF with trajectory trails to fig5 simulation`

---

## Success Criteria

### Verification Commands
```bash
python3 outputs/artifacts/S04/fig4_simulation.py  # Produces fig4_simulation.gif
python3 outputs/artifacts/S06/fig5_simulation.py  # Produces fig5_simulation.gif
python3 -c "from PIL import Image; img = Image.open('outputs/artifacts/S04/fig4_simulation.gif'); print(f'fig4: {img.n_frames} frames')"
python3 -c "from PIL import Image; img = Image.open('outputs/artifacts/S06/fig5_simulation.gif'); print(f'fig5: {img.n_frames} frames')"
```

### Final Checklist
- [ ] fig4_simulation.gif exists with ~150 frames
- [ ] fig5_simulation.gif exists with ~150 frames
- [ ] Both show agents moving with fading trajectory trails
- [ ] Leaders are red triangles, followers are blue circles
- [ ] Edges connect agents at each frame
- [ ] Existing static PNG outputs still generated
- [ ] No changes to simulation logic
