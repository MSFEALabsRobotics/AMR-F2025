# Histogram Filter (1D Localization)

## 🎯 Objective
Implement a simple **histogram filter** to estimate the position of a robot moving along a 1-D corridor with landmarks.

---

## 🧩 Concept
- The **belief** represents the probability of the robot being in each cell.
- The **motion model** shifts this belief based on the commanded motion.
- The **measurement model** updates the belief based on sensor readings (e.g., detecting landmarks).

---

## 🧠 Steps
1. **Initialize** a uniform belief distribution (equal probability everywhere).  
2. **Predict** step — update the belief after motion.  
3. **Update** step — modify the belief based on a noisy measurement.  
4. **Normalize** the belief so probabilities sum to 1.  
5. **Plot** the evolving belief over time.

---

## 💻 Python Code
```python
import numpy as np
import matplotlib.pyplot as plt

# Step 1: Initialize uniform belief over 10 cells
bel = np.ones(10) / 10

# Step 2: Motion model (predict step)
def predict(bel, u=1):
    p_move = 0.8           # probability of successful move
    p_undershoot = 0.1     # robot moves less than commanded
    p_overshoot = 0.1      # robot moves more than commanded
    n = len(bel)
    new_bel = np.zeros(n)
    for i in range(n):
        new_bel[i] = (
            bel[(i - u) % n] * p_move +
            bel[(i - u - 1) % n] * p_overshoot +
            bel[(i - u + 1) % n] * p_undershoot
        )
    return new_bel / new_bel.sum()  # normalize

# Step 3: Measurement model (update step)
def update(bel, z, landmarks=[2, 4, 7]):
    p_hit, p_miss = 0.9, 0.1
    likelihood = np.array([
        p_hit if i in landmarks else p_miss for i in range(len(bel))
    ])
    if not z:
        likelihood = 1 - likelihood
    new_bel = bel * likelihood
    return new_bel / new_bel.sum()  # normalize

# Step 4: Loop: motion + measurement
plt.figure()
for step in range(4):
    bel = predict(bel)          # move one cell forward
    bel = update(bel, True)     # measure (landmark detected)
    plt.bar(range(10), bel)
    plt.title(f"Step {step+1}")
    plt.pause(0.5)

plt.xlabel("Cell position")
plt.ylabel("Belief probability")
plt.show()
```

---


