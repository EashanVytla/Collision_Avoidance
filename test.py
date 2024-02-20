import numpy as np

max = 20
a = np.array([[1,20,3],[4,50,6],[7,80,9]])
print(np.linalg.norm(a, axis=-2))
a = np.delete(a, np.linalg.norm(a, axis=-2)>max, -1)
print(a)