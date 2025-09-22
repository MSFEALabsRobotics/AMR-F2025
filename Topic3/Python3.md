# Exercise 3 – 2D Gaussian — Correlated Random Variables

## Problem Statement
Generate a 2D Gaussian with correlation between two variables, visualize the scatter, and compute mean and covariance.

## Explanation
1. Generate `x1` from a normal distribution.
2. Define `x2` as a linear function of `x1` plus noise → introduces correlation.
3. Scatter plot shows elliptical distribution (correlation).
4. Compute sample mean and covariance matrix.
5. Correlation coefficient shows strength of relationship.

- **Why:** Understanding correlation and covariance is key for multivariate probability in robotics.

## Python Code
```python
import numpy as np
import matplotlib.pyplot as plt

N = 3000

# STEP 1: make x1 as normal random numbers
x1 = np.random.normal(0, 1, N)

# STEP 2: make x2 depend on x1
x2 = 0.7 * x1 + np.random.normal(0, 1, N)

# STEP 3: scatter plot
plt.scatter(x1, x2, s=5, alpha=0.5)
plt.axis("equal")
plt.title("2D Gaussian (correlated variables)")
plt.xlabel("x1")
plt.ylabel("x2")
plt.show()

# STEP 4: sample statistics
sample_Sigma = np.cov([x1, x2])

print("Sample mean:", [x1.mean(), x2.mean()])
print("Sample covariance matrix:\n", sample_Sigma)

```

## Reflection
- Scatter plot shows elongated ellipse, confirming correlation.
- Covariance matrix off-diagonal values capture dependency.
