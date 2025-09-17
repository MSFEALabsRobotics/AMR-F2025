# Matplotlib Tutorial

Matplotlib is a library for creating visualizations in Python. It allows plotting of data in various styles.

---

## 1. Importing Matplotlib
```python
import matplotlib.pyplot as plt
```

---

## 2. Basic Plot
```python
x = [1, 2, 3, 4]
y = [10, 20, 25, 30]

plt.plot(x, y)
plt.show()
```

---

## 3. Adding Labels and Title
```python
plt.plot(x, y)
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.title("Simple Plot")
plt.show()
```

---

## 4. Multiple Plots
```python
x = [1, 2, 3, 4]
y1 = [1, 4, 9, 16]
y2 = [1, 2, 3, 4]

plt.plot(x, y1, label="y = x^2")
plt.plot(x, y2, label="y = x")
plt.legend()
plt.show()
```

---

## 5. Scatter Plot
```python
import numpy as np
x = np.random.rand(50)
y = np.random.rand(50)
plt.scatter(x, y)
plt.show()
```

---

## 6. Bar Chart
```python
categories = ["A", "B", "C"]
values = [3, 7, 5]
plt.bar(categories, values)
plt.show()
```

---

## 7. Histogram
```python
data = np.random.randn(1000)  # 1000 samples from normal distribution
plt.hist(data, bins=30)
plt.show()
```

---

## 8. Subplots
```python
x = np.linspace(0, 2*np.pi, 100)
y1 = np.sin(x)
y2 = np.cos(x)

plt.subplot(2, 1, 1)
plt.plot(x, y1)
plt.title("Sine")

plt.subplot(2, 1, 2)
plt.plot(x, y2)
plt.title("Cosine")

plt.tight_layout()
plt.show()
```

---

## 9. Saving Figures
```python
plt.plot([1, 2, 3], [4, 5, 6])
plt.savefig("plot.png")
```

---

## 10. Example: Styled Plot
```python
x = np.linspace(0, 10, 100)
y = np.sin(x)

plt.plot(x, y, 'r--', label="sin(x)")
plt.xlabel("x")
plt.ylabel("sin(x)")
plt.title("Sine Wave")
plt.legend()
plt.grid(True)
plt.show()
```
