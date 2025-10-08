# Exercise: 1D Particle Filter Localization with a Binary Side Sensor (Continuous State)

## Goal
Implement a **particle filter (PF)** in Python that localizes a robot moving on a **1D continuous track** using a **binary side ultrasonic sensor** 

---

## World
- Track length: **8 tiles × 40 cm = 3.20 m**. Let the continuous coordinate be **x ∈ [0, 3.20)** meters.
- Tile indices: **0..7**, where tile `i` covers **[0.40*i, 0.40*(i+1))**.
- Side map (objects beside tiles):  
  `0 0 1 0 0 1 1 0`  
  Meaning: tiles **2, 5, 6** have a side object; all others do not.

---

## Robot & Sensor
- The robot can move **forward and backward** along the track.
- Control input at step *t*: a desired **displacement** `u_t` in meters (signed).
- Motion is **noisy** (you must model process noise).
- The **side ultrasonic sensor** is simplified to a **binary** measurement `z_t ∈ {0,1}`:
  - `1` means the sensor believes there is an object on the side of the current tile.
  - `0` means it believes there is no object.
- Sensor is **imperfect** (you must model measurement noise with hit/false‑alarm probabilities).

---

## State, Controls, and Measurements
- **State**: the robot’s continuous 1D position `x_t` on the track.
- **Control**: signed displacement command `u_t` (meters).
- **Measurement**: binary `z_t` from the side sensor.

---

## Particle Filter Representation
- Use **M particles** (e.g., 500–2000). Each particle `x_t^[m]` is a hypothesis of position.
- Maintain **weights** `w_t^[m]` that sum to 1.
- PF steps each time *t*:
  1. **Predict (Motion update)**: sample a new position for each particle using the control `u_t` and process noise.
  2. **Correct (Measurement update)**: evaluate a **likelihood** for `z_t` given each particle’s tile; multiply weights and normalize.
  3. **Resample**: apply **low‑variance resampling** (systematic resampling) to focus particles on likely regions.
  4. **Estimate**: compute an estimate of position (e.g., weighted mean) and spread (e.g., weighted variance).

---

## Models You Must Design
### 1) Motion model (process)
- Deterministic part: `x' = x + u_t`.
- Stochastic part: add zero‑mean noise, e.g., `N(0, σ_u^2)`.
- **Boundary handling** (choose and justify one):
  - **Clamp** to `[0, 3.20]` (particles hitting the end stop remain at the boundary).
  - **Reflect** (mirror) at the boundaries to mimic bouncing.
  - **Wrap** around (circular track)—**not allowed** for this exercise.
- Document your chosen model and parameter(s).

### 2) Measurement model (binary side map)
- Let `m(i)` be the map value (0/1) of tile `i`.
- For a particle position `x`, compute **tile index**: `i(x) = floor(x / 0.40)` (clamp to 0..7).
- Define **hit probability** `p_hit = P(z=1 | m(i)=1)` and **false‑alarm** `p_fa = P(z=1 | m(i)=0)` with `0 < p_fa < p_hit < 1`.
- Then for each particle:
  - If `m(i)=1`: `P(z=1|x) = p_hit`, `P(z=0|x) = 1 - p_hit`.
  - If `m(i)=0`: `P(z=1|x) = p_fa`,  `P(z=0|x) = 1 - p_fa`.
- Choose reasonable numeric values (e.g., `p_hit ≈ 0.85–0.95`, `p_fa ≈ 0.05–0.15`) and report them.

---

## Required Implementation Tasks (No Solutions Provided)
1. **Data structures**
   - Particles array of shape `(M,)` (float positions in meters).
   - Weights array of shape `(M,)`, initially uniform.
2. **Initialization**
   - Uniformly sample particle positions over `[0, 3.20)`.
3. **Motion update**
   - Given `u_t`, propagate each particle with process noise and apply boundary rule.
4. **Measurement update**
   - Given observed `z_t ∈ {0,1}`, compute likelihood per particle from the binary side map and update weights.
   - Normalize weights carefully (guard against underflow; consider log‑weights if needed).
5. **Low‑variance resampling**
   - Implement systematic resampling to produce a new equally‑weighted set of particles.
6. **State estimation**
   - Report the **weighted mean** `μ_t` and **weighted variance** `σ_t^2` of the particle set after resampling.
7. **Loop**
   - Process a sequence of controls `{u_t}` and measurements `{z_t}` for `t=1..T`.
8. **Plots**
   - Plot the **track** `[0,3.20)` with **tile boundaries** every 0.40 m.
   - Overlay the **binary map** (e.g., draw shaded bands on tiles 2,5,6).
   - At each step (or selected steps), visualize particle positions (e.g., scatter along the track) and the mean estimate.
   - Plot the time series of `μ_t` and `σ_t`.
9. **Experiments**
   - Design at least **two** motion/measurement scenarios:
     - **Forward‑then‑backward** motion with realistic noise.
     - **Start with high uncertainty** (uniform) and show convergence near a unique map signature (e.g., around tiles with objects).

---

## Interface & I/O (Suggested)
- Functions (names are suggestions; signatures are up to you):
  - `init_particles(M, L=3.20)`
  - `predict(particles, u, sigma_u, L=3.20, mode="reflect")`
  - `binary_likelihood(z, particles, tile_map, p_hit, p_fa)`
  - `update_weights(weights, likelihoods)`
  - `low_variance_resample(particles, weights)`
  - `estimate(particles, weights)`
- Main loop should accept a list of `u_t` and `z_t` and return `μ_t, σ_t` history.

---

## Notes & Constraints
- **Continuous PF** (no histograms). Particles represent continuous positions.
- **Binary sensor** abstraction is **intentional** to emphasize PF mechanics without distance‑to‑wall geometry.
- Use a **fixed random seed** for reproducibility in plots you submit.
- Guard against **weight degeneracy** (e.g., renormalize, add tiny epsilon, or temper likelihoods if necessary).
- Consider **particle impoverishment**: optionally add a small percentage of random particles after resampling to maintain diversity (briefly report if you do).

---

## What to Submit
1. **Code** (`.py` or notebook) implementing the PF with the models above.
2. **A short PDF report** (≤ 3 pages) that includes:
   - Your model choices (`σ_u`, `p_hit`, `p_fa`, boundary mode) and justification.
   - Plots (particle clouds, mean/variance over time, and the track/map overlay).
   - A brief discussion of **convergence behavior**, failures (if any), and how the binary map structure influences localization.
3. **(Optional)** A short video/gif of the particle evolution across time.

---

## Grading Rubric (100 pts)
- Correct PF pipeline: predict, update, resample, estimate (30 pts)
- Sound motion & measurement models with documented parameters (20 pts)
- Robust weight handling & resampling (15 pts)
- Clear, well‑labeled plots and track/map overlay (15 pts)
- Experimental design and analysis/discussion (15 pts)
- Code clarity, structure, and comments (5 pts)

---

## Hints (Not Solutions)
- Start with small `σ_u` and widen it if particles lag behind the robot.
- If weights collapse, check your likelihoods and normalization; consider computing in log‑space then exponentiating after subtracting `max` log‑likelihood.
- Low‑variance resampling indices can be generated with a single random offset plus cumulative sums of weights.

---

## Constants (for reference)
- **Tile length**: 0.40 m
- **Track length**: 3.20 m
- **Side map**: `m = [0,0,1,0,0,1,1,0]`  (objects beside tiles 2, 5, 6)

