# Exercise: 1D Particle Filter Localization with a Binary Side Sensor (Continuous State)

## Goal
Implement a **particle filter (PF)** in Python that localizes a robot moving on a **1D continuous track** using a **binary side ultrasonic sensor** 

---

## World
- Track length: **8 tiles × 40 cm = 3.20 m**. Let the continuous coordinate be **x ∈ [0, 3.20)** meters.
- Side map (objects beside tiles):  
  `0 0 1 0 0 1 1 0`  
  Meaning: tiles **3, 6, 7** have a side object; all others do not.

---

## Robot & Sensor
- The robot can move **forward and backward** along the track.
- Control input at step *t*: a desired **displacement** `u_t` in meters (signed).
- The **side ultrasonic sensor** is simplified to a **binary** measurement `z_t ∈ {0,1}`:
  - `1` means the sensor believes there is an object on the side of the current tile.
  - `0` means it believes there is no object.
- OPTIONAL: Sensor is **imperfect** (you must model measurement noise with hit/false‑alarm probabilities).

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
  5. **Plot**: plot in real time

---



