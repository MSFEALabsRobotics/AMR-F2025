# Bayes Filter Example in Python (Line-by-Line Explanation)

This example implements a **histogram-based Bayes filter** in a discrete 1D world with 10 cells.  
We will go **line by line**, explaining each part of the code.

---

```python
import numpy as np
import matplotlib.pyplot as plt
```
- Import **NumPy** for arrays and math, and **Matplotlib** for plotting.

```python
rng = np.random.default_rng(11)
```
- Create a random number generator with seed = 11 (so results are reproducible).

```python
N = 10
```
- Define the number of discrete cells in the world (0–9).

---

## Normalize function

```python
def normalize(p):
    s = p.sum()
    return p/s if s>0 else np.ones_like(p)/len(p)
```
- Ensures a probability distribution sums to 1.  
- If the sum is zero, return a uniform distribution.

Formula:  
\[ p'(x) = \frac{p(x)}{\sum p(x)} \]

---

## Prediction step

```python
def predict(bel, u, p_move=0.8):
    N = len(bel)
    new = np.zeros_like(bel)
    for i in range(N):
        stay = bel[i] * (1 - p_move)
        move_from = bel[(i - u) % N] * p_move
        new[i] = stay + move_from
    return normalize(new)
```
- Implements the **motion model**.  
- With probability `p_move`, robot moves `u` cells.  
- With probability `1 - p_move`, robot stays.  
- Wrap-around with `%N` for cyclic world.  
- Normalizes result.

Formula (prediction):  
\[
\text{bel}^-(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \, \text{bel}(x_{t-1})
\]

---

## Likelihood function

```python
def likelihood(z, N, sigma=1.2):
    xs = np.arange(N)
    d2 = (xs - z)**2
    w = np.exp(-0.5 * d2 / (sigma**2))
    return normalize(w)
```
- Implements the **sensor model**.  
- Gaussian centered at the measurement `z`.  
- `sigma` controls noise width.  
- Returns normalized weights.

Formula (likelihood):  
\[
p(z \mid x) \propto e^{-\frac{(x-z)^2}{2\sigma^2}}
\]

---

## Initialization

```python
bel = np.ones(N) / N
```
- Start with **uniform belief** (robot equally likely in any cell).

```python
controls = [1,1,1,1]
measurements = [2,3,4,5]
```
- Robot moves 1 step each time.  
- Sensor provides noisy measurements.

```python
history = [bel.copy()]
```
- Keep track of belief history for plotting.

---

## Recursive Bayes Filter Loop

```python
for u, z in zip(controls, measurements):
    bel = predict(bel, u)
    bel = normalize(bel * likelihood(z, N))
    history.append(bel.copy())
```
- For each step:  
  1. **Predict**: use control input to update belief.  
  2. **Correct**: update with sensor measurement using Bayes rule.  
  3. Save the belief.

Formula (correction):  
\[
\text{bel}(x_t) = \eta \, p(z_t \mid x_t) \, \text{bel}^-(x_t)
\]

---

## Plotting

```python
steps = len(history)
fig, axes = plt.subplots(1, steps, figsize=(15, 3), sharey=True)
```
- Create subplot for each belief distribution.

```python
for i, bel in enumerate(history):
    axes[i].stem(np.arange(N), bel)
    axes[i].set_title(f"Step {i}")
    axes[i].set_xlabel("cell")
    if i == 0:
        axes[i].set_ylabel("probability")
```
- Plot beliefs as stem plots (discrete bars).  
- Step 0 = uniform, later steps localize.

```python
plt.tight_layout()
plt.show()
```
- Display the results.

---

# 📌 Summary of Formulas

1. **Prediction step**:  
\[
\text{bel}^-(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \, \text{bel}(x_{t-1})
\]

2. **Correction step**:  
\[
\text{bel}(x_t) = \eta \, p(z_t \mid x_t) \, \text{bel}^-(x_t)
\]

---
