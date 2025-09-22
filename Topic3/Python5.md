# Bayes Filter Example in Python

This example shows **histogram-based localization** using a **discrete Bayes filter**.  
We use a 1D world with `N=10` cells. The robot moves forward with some uncertainty and senses positions with Gaussian noise.

---

```python
import numpy as np
import matplotlib.pyplot as plt

# Random number generator with fixed seed (for reproducibility)
rng = np.random.default_rng(11)

# Number of discrete cells in the world
N = 10

def normalize(p):
    """
    Normalize a probability distribution so that sum(p) = 1.
    If sum is zero, return a uniform distribution.
    """
    s = p.sum()
    return p/s if s>0 else np.ones_like(p)/len(p)

def predict(bel, u, p_move=0.8):
    """
    Prediction step of Bayes filter.

    bel' (x_t) = (1 - p_move) * bel(x_t) + p_move * bel(x_{t-1})
    with control u (shift), wrap-around using modulo.

    Args:
        bel : current belief (array over N cells)
        u   : control (movement in cells)
        p_move : probability of successful movement
    Returns:
        new belief distribution (normalized)
    """
    N = len(bel)
    new = np.zeros_like(bel)
    for i in range(N):
        stay = bel[i] * (1 - p_move)                  # stayed in place
        move_from = bel[(i - u) % N] * p_move         # moved from neighbor
        new[i] = stay + move_from
    return normalize(new)

def likelihood(z, N, sigma=1.2):
    """
    Measurement model (likelihood).

    p(z | x) ~ exp(-(x - z)^2 / (2 * sigma^2))

    Args:
        z     : observed measurement (cell index)
        N     : number of cells
        sigma : sensor noise standard deviation
    Returns:
        normalized weight distribution
    """
    xs = np.arange(N)
    d2 = (xs - z)**2
    w = np.exp(-0.5 * d2 / (sigma**2))     # Gaussian around z
    return normalize(w)

# -----------------------------
# INITIALIZATION
# -----------------------------

# Initial belief: uniform (no knowledge of position)
bel = np.ones(N) / N

# Controls (robot moves forward 1 step each time)
controls = [1,1,1,1]

# Noisy sensor measurements
measurements = [2,3,4,5]

# Store history of beliefs
history = [bel.copy()]

# -----------------------------
# BAYES FILTER LOOP
# -----------------------------
for u, z in zip(controls, measurements):
    bel = predict(bel, u)                      # prediction step
    bel = normalize(bel * likelihood(z, N))    # correction step (Bayes update)
    history.append(bel.copy())

# -----------------------------
# PLOT RESULTS
# -----------------------------
steps = len(history)
fig, axes = plt.subplots(1, steps, figsize=(15, 3), sharey=True)

for i, bel in enumerate(history):
    axes[i].stem(np.arange(N), bel)
    axes[i].set_title(f"Step {i}")
    axes[i].set_xlabel("cell")
    if i == 0:
        axes[i].set_ylabel("probability")

plt.tight_layout()
plt.show()
```

---

# 📌 Formulas (Bayes Filter)

1. **Prediction step (motion update)**  

\[
\text{bel}^-(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \, \text{bel}(x_{t-1})
\]

2. **Correction step (measurement update)**  

\[
\text{bel}(x_t) = \eta \, p(z_t \mid x_t) \, \text{bel}^-(x_t)
\]

where  
- \( \eta \) = normalization constant (ensures probabilities sum to 1)  
- \( u_t \) = control action (movement)  
- \( z_t \) = measurement (sensor reading)  
- \( p(z_t \mid x_t) \) = likelihood (measurement model)  
- \( p(x_t \mid u_t, x_{t-1}) \) = motion model  

---
