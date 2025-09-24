# Exercise 3 – Unscented Kalman Filter (UKF)

In this exercise, we implement the **Unscented Kalman Filter (UKF)** to estimate the pose of a robot following a nonlinear path.  
The UKF avoids Jacobians by using **sigma points** to approximate distributions.

---

## 📘 Step 1: Formulas

1. Generate sigma points around mean $\mu$ with covariance $\Sigma$.  
2. Propagate sigma points through motion model $g$.  
3. Compute predicted mean and covariance.  
4. Propagate sigma points through measurement model $h$.  
5. Compute Kalman gain, update mean and covariance.  

---

## 📝 Step 2: Generate Synthetic Data

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
```

---

## 📝 Step 3: Initialize UKF

```python
mu = np.array([0.,0.,0.])
Sigma = np.eye(3)
Q = np.diag([0.1,0.1,0.01])
R = np.array([[0.25]])

predictions = []
estimates = []
```

---

## 📝 Step 4: UKF Loop

```python
def sigma_points(mu, Sigma, alpha=1e-3, beta=2, kappa=0):
    n = mu.size
    lam = alpha**2*(n+kappa)-n
    Sigma_root = np.linalg.cholesky((n+lam)*Sigma)
    pts = [mu]
    for i in range(n):
        pts.append(mu+Sigma_root[:,i])
        pts.append(mu-Sigma_root[:,i])
    return np.array(pts), lam

def unscented_weights(n, lam, alpha=1e-3, beta=2):
    wm = np.full(2*n+1, 1/(2*(n+lam)))
    wc = np.full(2*n+1, 1/(2*(n+lam)))
    wm[0] = lam/(n+lam)
    wc[0] = lam/(n+lam) + (1-alpha**2+beta)
    return wm,wc

for z in measurements:
    n = mu.size
    sigma_pts, lam = sigma_points(mu, Sigma)
    wm, wc = unscented_weights(n, lam)
    
    propagated = np.array([motion_model(pt,u) for pt in sigma_pts])
    mu_pred = np.sum(wm[:,None]*propagated,axis=0)
    Sigma_pred = Q.copy()
    for i in range(2*n+1):
        diff = propagated[i]-mu_pred
        Sigma_pred += wc[i]*np.outer(diff,diff)
    predictions.append(mu_pred)
    
    Z_sigma = np.array([measurement_model(pt) for pt in propagated])
    z_pred = np.sum(wm[:,None]*Z_sigma,axis=0)
    S = R.copy()
    for i in range(2*n+1):
        diff = Z_sigma[i]-z_pred
        S += wc[i]*np.outer(diff,diff)
    cross_cov = np.zeros((n,1))
    for i in range(2*n+1):
        cross_cov += wc[i]*np.outer(propagated[i]-mu_pred, Z_sigma[i]-z_pred)
    K = cross_cov @ np.linalg.inv(S)
    mu = mu_pred + (K @ (z-z_pred)).ravel()
    Sigma = Sigma_pred - K @ S @ K.T
    estimates.append(mu)

print("First prediction:", predictions[0])
print("First update:", estimates[0])
```

---

## 📝 Step 5: Plot Results

```python
predictions = np.array(predictions)
estimates = np.array(estimates)

plt.plot(true_states[:,0], true_states[:,1], label="True Path")
plt.plot(predictions[:,0], predictions[:,1], "--", label="Predictions (before update)")
plt.plot(estimates[:,0], estimates[:,1], label="UKF Estimate (after update)")
plt.legend()
plt.title("Unscented Kalman Filter (Circular Motion)")
plt.show()
```

---

## 🎯 Exercises

1. Compare UKF vs EKF estimates. Which is closer to truth?  
2. Increase process noise $Q$. What happens to predictions?  
3. Why does UKF perform better in nonlinear systems?  

---
