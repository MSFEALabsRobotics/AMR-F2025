# NumPy Tutorial

NumPy (Numerical Python) is a library for numerical computing in Python. It provides fast operations on arrays, matrices, and numerical data.

---

## 1. Importing NumPy
```python
import numpy as np
```

---

## 2. Creating Arrays
```python
a = np.array([1, 2, 3])           # 1D array
b = np.array([[1, 2], [3, 4]])    # 2D array
zeros = np.zeros((3, 3))          # 3x3 zeros
ones = np.ones((2, 2))            # 2x2 ones
identity = np.eye(3)              # 3x3 identity matrix
random = np.random.rand(2, 3)     # Random 2x3 array
```

---

## 3. Array Properties
```python
a.shape     # Shape of array
a.ndim      # Number of dimensions
a.size      # Total number of elements
a.dtype     # Data type
```

---

## 4. Indexing and Slicing
```python
arr = np.array([10, 20, 30, 40, 50])
print(arr[0])      # First element
print(arr[-1])     # Last element
print(arr[1:4])    # Elements from index 1 to 3
```

For 2D arrays:
```python
matrix = np.array([[1, 2, 3], [4, 5, 6]])
print(matrix[0, 1])     # Row 0, Col 1 -> 2
print(matrix[:, 1])     # All rows, Col 1
print(matrix[1, :])     # Row 1, all cols
```

---

## 5. Array Operations
```python
x = np.array([1, 2, 3])
y = np.array([4, 5, 6])

print(x + y)    # [5, 7, 9]
print(x * y)    # [4, 10, 18]
print(x.dot(y)) # Dot product = 32
print(np.sum(x)) # Sum = 6
print(np.mean(x)) # Mean = 2.0
```

---

## 6. Reshaping and Transposing
```python
arr = np.arange(6)       # [0 1 2 3 4 5]
mat = arr.reshape(2, 3)  # 2x3 matrix
print(mat.T)             # Transpose
```

---

## 7. Useful Functions
```python
np.linspace(0, 1, 5)      # [0. 0.25 0.5 0.75 1.]
np.arange(0, 10, 2)       # [0 2 4 6 8]
np.max(arr), np.min(arr)  # Max & Min
np.argmin(arr), np.argmax(arr) # Index of min & max
```

---

## 8. Example: Matrix Multiplication
```python
A = np.array([[1, 2], [3, 4]])
B = np.array([[2, 0], [1, 3]])
C = A @ B   # Matrix multiplication
print(C)
```
