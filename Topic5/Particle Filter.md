# Particle Filter (1D Tracking)

## 🎯 Objective
Implement a **particle filter** that tracks a robot’s position in 1-D using noisy motion and sensor data.

---

## 🧩 Concept
- Each **particle** represents one possible hypothesis of the robot’s state.
- The filter performs three main steps:
  1. **Prediction:** move all particles based on control input.
  2. **Update:** reweight particles according to how well they match the measurement.
  3. **Resample:** select new particles based on weights to focus on high-probability regions.

---

## 🧠 Steps
1. Initialize particles uniformly over the 1-D world.
2. Predict motion with Gaussian noise.
3. Compute weights using a noisy measurement of distance to a landmark.
4. Resample using **low-variance sampling**.
5. Plot particle distribution after each iteration.

---

## 💻 Python Code
```python
import numpy as np
import matplotlib.pyplot as plt

# Step 1: Initialization
N = 1000  # number of particles
particles = np.random.uniform(0, 100, N)
weights = np.ones(N) / N
true_x = 0  # true robot position

# Step 2: Motion model
def predict(particles, u, noise=0.5):
    return particles + u + np.random.normal(0, noise, len(particles))

# Step 3: Measurement update
def update(particles, weights, z, landmark=50, sigma=2):
    distances = np.abs(particles - landmark)
    weights *= np.exp(-0.5 * ((distances - z) / sigma) ** 2)
    weights += 1e-300  # avoid zeros
    weights /= weights.sum()  # normalize
    return weights

# Step 4: Low-variance resampling
def resample(particles, weights):
    N = len(particles)
    indexes = np.zeros(N, dtype=int)
    cumulative_sum = np.cumsum(weights)
    step = 1.0 / N
    r = np.random.rand() * step
    i = 0
    for j in range(N):
        u = r + j * step
        while u > cumulative_sum[i]:
            i += 1
        indexes[j] = i
    return particles[indexes]

# Step 5: Recursive estimation
plt.figure()
for t in range(20):
    # true robot motion
    true_x += 1 + np.random.randn() * 0.5
    z = np.abs(true_x - 50) + np.random.randn() * 2  # noisy measurement

    particles = predict(particles, 1)
    weights = update(particles, weights, z)
    particles = resample(particles, weights)

    plt.clf()
    plt.hist(particles, bins=30, color='skyblue')
    plt.axvline(true_x, color='r', label='True position')
    plt.title(f"Step {t+1}")
    plt.pause(0.2)

plt.xlabel("Position (cm)")
plt.ylabel("Particle count")
plt.show()
```

---
