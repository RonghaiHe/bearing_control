# Fix Formation Simulation - Work Plan

## TL;DR

> Fix bearing-based formation control simulation to match original paper figures.
> Root cause: Graph edges must be bidirectional for followers.

**Deliverables**:
- Fixed Fig. 4 simulation matching original (2D square formation)
- Fixed Fig. 5 simulation matching original (3D cube formation)

**Estimated Effort**: Short (1-2 hours)
**Parallel Execution**: YES - 2 waves

---

## Context

### Original Request
User pointed out that reproduced figures don't match original paper figures:
- Original Fig. 4 (figure_24.png): 2D trajectory expanding left to right
- Original Fig. 5 (figure_68.png): 3D trajectory with obstacle

### Root Cause Analysis
The control law implementation has a critical bug:
1. **Graph edges are directed** - But paper assumes bidirectional for followers
2. **Bearing error increasing** - Should converge to zero
3. **Formation not tracking** - Followers not following leaders correctly

### Key Insight from Paper
> "We assume that the information flow between any two followers is bidirectional."

This means edges among followers must be bidirectional: (i,j) AND (j,i).

---

## Work Objectives

### Core Objective
Fix simulation scripts so bearing error converges to zero and figures match original paper.

### Concrete Deliverables
- `outputs/artifacts/S04/fig4_simulation.py` - Fixed 2D simulation
- `outputs/artifacts/S04/fig4_simulation.png` - Updated figure
- `outputs/artifacts/S06/fig5_simulation.py` - Fixed 3D simulation
- `outputs/artifacts/S06/fig5_simulation.png` - Updated figure

### Definition of Done
- [ ] Bearing error converges to 0 in Fig. 4
- [x] Bearing error converges to 0 in Fig. 5
- [ ] Trajectories match paper descriptions

### Must Have
- Bidirectional edges for all follower connections
- Correct desired bearings for each edge direction
- Proper leader motion functions

### Must NOT Have
- Directed edges among followers
- Incorrect bearing signs

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (Python + matplotlib)
- **Automated tests**: NO (manual verification)
- **Framework**: Python scripts

### QA Policy
- Run simulation scripts
- Verify bearing error converges to 0
- Compare visual output with original paper figures

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Fix graph structure):
├── Task 1: Fix Fig. 4 simulation graph edges
└── Task 2: Fix Fig. 5 simulation graph edges

Wave 2 (Verify):
├── Task 3: Run and verify Fig. 4
└── Task 4: Run and verify Fig. 5
```

---

## TODOs

- [x] 1. Fix Fig. 4 simulation graph edges

  **What to do**:
  - Change edges from directed to bidirectional
  - Add both (i,j) and (j,i) for each connection
  - Ensure desired_bearings has entries for all edge directions
  - For edge (i,j) with bearing g, add (j,i) with bearing -g

  **Key changes needed**:
  ```python
  # OLD (incorrect):
  edges = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]
  
  # NEW (correct):
  edges = [
      (0, 1), (1, 0),  # Bidirectional
      (0, 2), (2, 0),  # Bidirectional
      (1, 3), (3, 1),  # Bidirectional
      (2, 3), (3, 2),  # Bidirectional
      (0, 3), (3, 0),  # Bidirectional
      (1, 2), (2, 1),  # Bidirectional
  ]
  ```

  **References**:
  - Paper Section II: "information flow between any two followers is bidirectional"
  - Control law (7) sums over j ∈ N_i where N_i includes all accessible agents

  **Acceptance Criteria**:
  - [ ] Bearing error converges to 0
  - [ ] Formation tracks leader motion

  **Commit**: YES
  - Message: `fix: make graph edges bidirectional for follower connections`

---

- [x] 2. Fix Fig. 5 simulation graph edges

  **What to do**:
  - Same fix as Task 1 but for 3D cube formation
  - Ensure all 12 cube edges are bidirectional
  - Add both (i,j) and (j,i) for each cube edge

  **Key changes needed**:
  ```python
  # For cube, each edge needs both directions
  edges = []
  for i in range(8):
      for j in range(8):
          if i < j:
              diff = i ^ j
              if diff in [1, 2, 4]:
                  edges.append((i, j))
                  edges.append((j, i))  # Add reverse edge
  ```

  **References**:
  - Paper Section III-B: 3D cube formation example
  - Fig. 2(c): Cube with 8 agents

  **Acceptance Criteria**:
  - [ ] Bearing error converges to 0
  - [ ] Formation converges to cube pattern

  **Commit**: YES
  - Message: `fix: make 3D cube graph edges bidirectional`

---

- [x] 3. Run and verify Fig. 4 simulation

  **What to do**:
  - Run updated fig4_simulation.py
  - Verify bearing error plot shows convergence to 0
  - Compare trajectory with original paper figure

  **Acceptance Criteria**:
  - [ ] Bearing error < 0.1 at t=20s
  - [ ] Formation maintains square pattern
  - [ ] Visual match with paper

  **Commit**: NO

---

- [x] 4. Run and verify Fig. 5 simulation

  **What to do**:
  - Run updated fig5_simulation.py
  - Verify bearing error plot shows convergence to 0
  - Compare trajectory with original paper figure

  **Acceptance Criteria**:
  - [ ] Bearing error < 1.0 at t=15s
  - [ ] Formation converges to cube pattern
  - [ ] Visual match with paper

  **Commit**: NO

---

## Success Criteria

### Verification Commands
```bash
cd outputs/artifacts/S04 && python3 fig4_simulation.py
cd outputs/artifacts/S06 && python3 fig5_simulation.py
```

### Final Checklist
- [ ] Bearing error converges to 0 in both simulations
- [ ] Figures qualitatively match original paper
- [ ] All edges are bidirectional for followers
