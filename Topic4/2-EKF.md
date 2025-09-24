# Exercise 2 – Extended Kalman Filter (EKF) with Circular Motion

In this exercise, we implement the **Extended Kalman Filter (EKF)** to estimate the pose of a robot following a circular trajectory.  
The EKF handles **nonlinear motion models** by linearizing them with a **Jacobian**.

⚠️ **Note on visualization:**  
Although the robot's state is **2D position + orientation**, we show the results in a **2D plot** of $(x,y)$ positions.  
This makes it easier to compare true path, predictions, and EKF estimates.

---

## 📘 Step 1: Formulas

Prediction (nonlinear motion model $g$):  

$$
x_t^- = g(x_{t-1}, u_t)
$$  

$$
P_t^- = F_t P_{t-1} F_t^T + Q
$$  

where $F_t$ is the Jacobian of $g$ w.r.t. the state.  

Update (nonlinear measurement model $h$):  

$$
z_t = h(x_t) + \epsilon
$$  

$$
K_t = P_t^- H_t^T (H_t P_t^- H_t^T + R)^{-1}
$$  

$$
x_t = x_t^- + K_t (z_t - h(x_t^-))
$$  

$$
P_t = (I - K_t H_t) P_t^-
$$  

where $H_t$ is the Jacobian of $h$ w.r.t. the state.  

---

## 📝 Step 2: Generate Synthetic Data

We simulate a robot moving in a circle with velocity and angular velocity.  
Measurements are noisy range values from the origin.

```python
import numpy as np
import matplotlib.pyplot as plt

def motion_model(x, u, dt=0.1):
    return np.array([
        x[0] + u[0]*np.cos(x[2])*dt,
        x[1] + u[0]*np.sin(x[2])*dt,
        x[2] + u[1]*dt
    ])

def measurement_model(x):
    return np.array([np.sqrt(x[0]**2 + x[1]**2)])

T = 100
true_states = np.zeros((T,3))
true_states[0] = [0,0,0]
u = [1.0, 0.1]

for t in range(1,T):
    true_states[t] = motion_model(true_states[t-1], u)

measurements = np.array([measurement_model(s)[0] + np.random.normal(0,0.5) for s in true_states])

plt.plot(true_states[:,0], true_states[:,1], label="True Path")
plt.scatter(true_states[:,0], true_states[:,1], c="blue", s=5)
plt.title("True Circular Motion Path")
plt.legend()
plt.show()
```

---

## 📝 Step 3: Initialize the Filter

```python
x_est = np.array([0,0,0])
P = np.eye(3)
Q = np.diag([0.1,0.1,0.01])
R = np.array([[0.25]])
estimates = []
predictions = []
```

---

## 📝 Step 4: EKF Loop

```python
for z in measurements:
    # Predict
    Fx = np.eye(3)
    Fx[0,2] = -u[0]*np.sin(x_est[2])*0.1
    Fx[1,2] =  u[0]*np.cos(x_est[2])*0.1
    x_pred = motion_model(x_est, u)
    P_pred = Fx @ P @ Fx.T + Q

    predictions.append(x_pred)

    # Update
    H = np.array([[x_pred[0]/np.sqrt(x_pred[0]**2+x_pred[1]**2+1e-6),
                   x_pred[1]/np.sqrt(x_pred[0]**2+x_pred[1]**2+1e-6),
                   0]])
    z_pred = measurement_model(x_pred)
    y = z - z_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_est = x_pred + (K @ y).ravel()
    P = (np.eye(3) - K @ H) @ P_pred

    estimates.append(x_est)

print("First prediction:", predictions[0])
print("First update:", estimates[0])
```

---

## 📝 Step 5: Plot Results

```python
estimates = np.array(estimates)
predictions = np.array(predictions)

plt.plot(true_states[:,0], true_states[:,1], label="True Path")
plt.plot(predictions[:,0], predictions[:,1], "--", label="Predictions (before update)")
plt.plot(estimates[:,0], estimates[:,1], label="EKF Estimate (after update)")
plt.legend()
plt.title("Extended Kalman Filter (Circular Motion)")
plt.show()
```

---

## 🎯 Exercises

1. Change the angular velocity. How does EKF behave?  
2. Increase measurement noise $R$. What happens to Kalman Gain $K$?  
3. Compare prediction vs update paths. What do you notice?  

---
