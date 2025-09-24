# Exercise 1 – Kalman Filter (1D Position Tracking)

In this exercise, we implement a simple **Kalman Filter (KF)** to estimate the 1D position of a robot moving in a straight line.  
The goal is to see how the KF fuses **predictions** and **noisy measurements** into a smoother estimate.

---

## 📘 Step 1: Formulas

We assume the state is **position** \(x_t\).  
Measurements are noisy observations \(z_t\).  

**Prediction:**  
\[
x_t^- = x_{t-1}
\]  
\[
P_t^- = P_{t-1} + Q
\]  

**Update (Correction):**  
\[
K_t = \frac{P_t^-}{P_t^- + R}
\]  
\[
x_t = x_t^- + K_t \, (z_t - x_t^-)
\]  
\[
P_t = (1 - K_t) P_t^-
\]  

---

## 📝 Step 2: Generate Synthetic Data

We simulate a robot moving from 0 → 10 in 50 steps.  
Measurements are noisy with Gaussian noise.

```python
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)
T = 50
true_pos = np.linspace(0, 10, T)
measurements = true_pos + np.random.normal(0, 0.5, T)

print("First 5 true positions:", true_pos[:5])
print("First 5 noisy measurements:", measurements[:5])

plt.plot(true_pos, label="True Position")
plt.plot(measurements, "o", alpha=0.5, label="Measurements")
plt.legend()
plt.title("True vs Noisy Measurements")
plt.show()
```

---

## 📝 Step 3: Initialize the Filter

We define arrays for the estimated state `x_est` and its uncertainty `P`.  
We also set noise values `Q` (process) and `R` (measurement).

```python
x_est = np.zeros(T)
P = np.zeros(T)
x_est[0] = 0      # initial guess
P[0] = 1          # initial uncertainty
Q = 0.01          # process noise
R = 0.25          # measurement noise

print("Initial guess x0 =", x_est[0])
print("Initial uncertainty P0 =", P[0])
```

---

## 📝 Step 4: Recursive Kalman Loop

We apply **predict** then **update** at each time step.

```python
for t in range(1, T):
    # Predict
    x_pred = x_est[t-1]
    P_pred = P[t-1] + Q

    # Update
    K = P_pred / (P_pred + R)
    x_est[t] = x_pred + K * (measurements[t] - x_pred)
    P[t] = (1 - K) * P_pred

    # Print first few iterations for clarity
    if t < 5:
        print(f"Step {t}: Prediction = {x_pred:.2f}, Measurement = {measurements[t]:.2f}, Update = {x_est[t]:.2f}, K = {K:.2f}")
```

---

## 📝 Step 5: Plot Results

We compare true position, noisy measurements, and KF estimates.

```python
plt.plot(true_pos, label="True Position")
plt.plot(measurements, "o", alpha=0.5, label="Measurements")
plt.plot(x_est, label="KF Estimate")
plt.legend()
plt.title("Kalman Filter Result")
plt.show()
```

---

## 🎯 Exercises

1. Change the measurement noise **R**.  
   - What happens when R is very large? Very small?  

2. Add a **constant velocity model** (instead of just position).  
   - How does the KF change?  

3. Try increasing **process noise Q**.  
   - How does the estimate respond?  

---
