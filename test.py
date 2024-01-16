import numpy as np

a = np.array([[1,2,3],[4,5,6]]).T
b = np.array([[],[],[]])
c = np.concatenate((b, a),1)
print(c)
print(np.shape(c))