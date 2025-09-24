# Topic 4 – Gaussian Filters

This tutorial covers **Gaussian Filters** for recursive state estimation in robotics.  
Based on your lecture slides (Ch. 3).

---

## 1. Introduction

Gaussian filters are among the earliest implementations of the **Bayes filter** for continuous spaces.  
They remain widely used due to their efficiency, even though they are limited by their unimodal Gaussian representation.

- **Belief Representation**: Multivariate Normal Distribution  
- **Parameterizations**:  
  - **Moments**: mean (μ), covariance (Σ)  
  - **Canonical**: information matrix (Ω = Σ⁻¹), information vector (ξ = Σ⁻¹ μ)

---

## 2. Kalman Filter (KF)

### Theory
The **Kalman Filter** applies when:
1. The system dynamics are **linear** with Gaussian noise.  
2. The measurement model is **linear** with Gaussian noise.  
3. The initial belief is Gaussian.  

Equations:

Prediction:  
$$
x_t^- = A x_{t-1} + B u_t
$$
$$
P_t^- = A P_{t-1} A^T + Q
$$

Correction:  
$$
K_t = P_t^- H^T (H P_t^- H^T + R)^{-1}
$$
$$
x_t = x_t^- + K_t (z_t - H x_t^-)
$$
$$
P_t = (I - K_t H) P_t^-
$$

### Example: 1D Position Tracking

```python
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)
T = 50
true_pos = np.linspace(0, 10, T)
measurements = true_pos + np.random.normal(0, 0.5, T)

x_est = np.zeros(T)
P = np.zeros(T)
x_est[0] = 0
P[0] = 1
Q = 0.01
R = 0.25

for t in range(1, T):
    # Predict
    x_pred = x_est[t-1]
    P_pred = P[t-1] + Q

    # Update
    K = P_pred / (P_pred + R)
    x_est[t] = x_pred + K * (measurements[t] - x_pred)
    P[t] = (1 - K) * P_pred

plt.plot(true_pos, label="True Position")
plt.plot(measurements, "o", alpha=0.5, label="Measurements")
plt.plot(x_est, label="KF Estimate")
plt.legend()
plt.show()
```

![KF Example](kf_example.png)

#### Exercise
- Change the measurement noise **R**. What happens when R is very large? Very small?
- Try adding a constant velocity term to the motion model.

---

## 3. Extended Kalman Filter (EKF)

### Theory
Real-world systems are **nonlinear**. EKF linearizes around the current estimate using a **first-order Taylor expansion** (Jacobian).

Prediction:  
$$
x_t^- = g(x_{t-1}, u_t)
$$

Correction:  
$$
z_t = h(x_t) + \epsilon
$$

Where \\( g \\) is the nonlinear motion model, \\( h \\) is the nonlinear measurement model.

### Example: Circular Robot Motion

```python
def motion_model(x, u):
    dt = 0.1
    return np.array([
        x[0] + u[0] * np.cos(x[2]) * dt,
        x[1] + u[0] * np.sin(x[2]) * dt,
        x[2] + u[1] * dt
    ])

def measurement_model(x):
    return np.array([np.sqrt(x[0]**2 + x[1]**2)])

# EKF prediction and update as implemented in NumPy...
```

![EKF Example](ekf_example.png)

#### Exercise
- Modify the angular velocity in the motion model. How does EKF perform?  
- Compare EKF estimates when initial uncertainty (Σ₀) is large vs. small.

---

## 4. Unscented Kalman Filter (UKF)

### Theory
Instead of linearization via Taylor expansion, UKF uses a set of **sigma points** to approximate the Gaussian.  
These sigma points are propagated through the nonlinear function, capturing up to 2nd-order effects.

Advantages:
- No Jacobians needed.  
- More accurate than EKF in many nonlinear problems.  
- Similar complexity to EKF.

### Example: Circular Robot Motion with Sigma Points

```python
# UKF steps:
# 1. Generate sigma points
# 2. Propagate through motion model
# 3. Compute predicted mean & covariance
# 4. Incorporate measurements with Kalman-like update
```

![UKF Example](ukf_example.png)

#### Exercise
- Compare UKF vs EKF trajectories for the same problem. Which is closer to ground truth?  
- Increase measurement noise — which filter degrades slower?

---

## 5. Information Filter (IF)

### Theory
The **dual of KF**. Instead of mean & covariance, it uses:

- Information matrix: \\( \Omega = \Sigma^{-1} \\)  
- Information vector: \\( \xi = \Sigma^{-1} \mu \\)

Belief:  
$$
p(x) = \eta \exp\left(-\frac{1}{2} x^T \Omega x + \xi^T x \right)
$$

### Example: 1D Tracking

```python
Omega = np.array([[1.0]])
xi = np.array([0.0])

for z in measurements:
    # Prediction and update in canonical form
    Omega_new = Omega + 1.0/R
    xi_new = xi + z/R
    mu_new = xi_new / Omega_new
```

![IF Example](if_example.png)

#### Exercise
- Compare IF and KF results for the same problem. Are they equivalent?  
- Why might IF be more numerically stable in large-scale problems?

---

# ✅ Summary

- **KF**: Linear systems only.  
- **EKF**: Nonlinear via Jacobian linearization.  
- **UKF**: Nonlinear via sigma points (better approximation).  
- **IF**: Canonical form, dual to KF, stable for large systems.

---
