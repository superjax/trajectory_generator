import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(linewidth=200)

original = np.reshape(np.fromfile('../logs/original.bin', dtype=np.float64), (-1,4))
downsampled = np.reshape(np.fromfile('../logs/downsampled.bin', dtype=np.float64), (-1,4))
optimized = np.reshape(np.fromfile('../logs/optimized_states.bin', dtype=np.float64), (-1,14))
inputs = np.reshape(np.fromfile('../logs/optimized_inputs.bin', dtype=np.float64), (-1,4))
dynamic_costs = np.reshape(np.fromfile('../logs/dynamics_costs.bin', dtype=np.float64), (-1,4))

# plot 3D trajectory
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')
ax.plot(original[:,0], original[:,1], original[:,2], label="original")
ax.plot(downsampled[:,0], downsampled[:,1], downsampled[:,2], label="downsampled")
ax.plot(optimized[:,0], optimized[:,1], optimized[:,2], label="optimized")
ax.legend()

ax = None
fig = plt.figure(figsize=(12, 9))
plt.suptitle('position')
for i in range(3):
    ax = plt.subplot(3, 1, i+1, sharex=ax)
    ax.plot(optimized[:,i], label="optimized")

ax = None
labels = ["qw", "qx", "qy", "qz"]
fig = plt.figure(figsize=(12, 9))
plt.suptitle('quaternion')
for i in range(4):
    ax = plt.subplot(4, 1, i+1, sharex=ax)
    ax.plot(optimized[:,i+3],  label=labels[i])
    ax.legend()

ax = None
labels = ["x", "y", "z"]
fig = plt.figure(figsize=(12, 9))
plt.suptitle('velocity')
for i in range(3):
    ax = plt.subplot(3, 1, i+1, sharex=ax)
    ax.plot(optimized[:,i+7],  label=labels[i])
    ax.legend()

ax = None
labels = ["wx", "wy", "wz", "F"]
fig = plt.figure(figsize=(12, 9))
plt.suptitle('inputs')
for i in range(4):
    ax = plt.subplot(4, 1, i + 1, sharex=ax)
    ax.plot(inputs[:, i], label=labels[i])
    ax.legend()

ax = None
labels = ['dpx', 'dpy', 'dpz', 'u']
fig = plt.figure(figsize=(12, 9))
plt.suptitle('dynamics_costs')
for i in range(4):
    ax = plt.subplot(4, 1, i + 1, sharex=ax)
    ax.plot(dynamic_costs[:, i], label=labels[i])
    ax.legend()

plt.show()  