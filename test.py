import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D plotting tool
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt
from ahrs.filters import Mahony
from ahrs.common.orientation import q2R  # Function to convert quaternion to rotation matrix

# Load CSV Data
df = pd.read_csv('data.csv')
gyro = df[['gyroscopex', 'gyroscopey', 'gyroscopez']].values * np.pi / 180  # Convert to radians
acc = df[['accelerometerx', 'accelerometery', 'accelerometerz']].values

# Sample Period
samplePeriod = 1/256

# AHRS Algorithm
ahrs = Mahony(sample_period=samplePeriod, kp=1)
num_samples = len(gyro)
Q = np.tile([1., 0., 0., 0.], (num_samples, 1))  # Initialize quaternion array

# Update the quaternion based on IMU data
for t in range(1, num_samples):
    Q[t] = ahrs.updateIMU(Q[t-1], gyro[t], acc[t])

# Convert Quaternions to Rotation Matrices
R = np.array([q2R(q) for q in Q])

# Tilt-Compensated Accelerometer
tcAcc = np.array([R[i].dot(acc[i]) for i in range(len(acc))])

# Calculate Linear Acceleration in Earth Frame
linAcc = (tcAcc - np.array([0, 0, 1])) * 9.81

# Integrate to get Linear Velocity
linVel = np.cumsum(linAcc * samplePeriod, axis=0)

# High-Pass Filter for Linear Velocity
order = 1
filtCutOff = 0.1
b, a = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high')
linVelHP = filtfilt(b, a, linVel, axis=0)

# Integrate to get Linear Position
linPos = np.cumsum(linVelHP * samplePeriod, axis=0)

# High-Pass Filter for Linear Position
linPosHP = filtfilt(b, a, linPos, axis=0)

# Setup for 3D animated plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('High-pass filtered Linear Position')
line, = ax.plot([], [], [], lw=2)

# Setting the axes properties
ax.set_xlim3d([np.min(linPosHP[:,0]), np.max(linPosHP[:,0])])
ax.set_ylim3d([np.min(linPosHP[:,1]), np.max(linPosHP[:,1])])
ax.set_zlim3d([np.min(linPosHP[:,2]), np.max(linPosHP[:,2])])

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')

# Initialization function: plot the background of each frame
def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

# Animation function which updates figure data. This is called sequentially
def animate(i):
    line.set_data(linPosHP[:i, 0], linPosHP[:i, 1])
    line.set_3d_properties(linPosHP[:i, 2])
    return line,

# Call the animator. blit=True means only re-draw the parts that have changed.
ani = FuncAnimation(fig, animate, init_func=init, frames=num_samples, interval=20, blit=True)

plt.show()

# Plotting the results
# plt.figure()
# plt.title('Tilt-Compensated Accelerometer')
# plt.plot(tcAcc)
# plt.legend(['X', 'Y', 'Z'])

# plt.figure()
# plt.title('Linear Acceleration')
# plt.plot(linAcc)
# plt.legend(['X', 'Y', 'Z'])

# plt.figure()
# plt.title('Linear Velocity')
# plt.plot(linVel)
# plt.legend(['X', 'Y', 'Z'])

# plt.figure()
# plt.title('High-pass filtered Linear Velocity')
# plt.plot(linVelHP)
# plt.legend(['X', 'Y', 'Z'])

# plt.figure()
# plt.title('Linear Position')
# plt.plot(linPos)
# plt.legend(['X', 'Y', 'Z'])

# plt.figure()
# plt.title('High-pass filtered Linear Position')
# plt.plot(linPosHP)
# plt.legend(['X', 'Y', 'Z'])

# plt.show()
# 3D Plot begins ------------------
# # High-Pass Filter for Linear Position
# linPosHP = filtfilt(b, a, linPos, axis=0)

# # Plotting the results in 3D
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_title('High-pass filtered Linear Position')

# # Create a color map to reflect time progression
# colors = plt.cm.jet(np.linspace(0, 1, num_samples))

# # Scatter plot with color changing over time
# scatter = ax.scatter(linPosHP[:, 0], linPosHP[:, 1], linPosHP[:, 2], c=colors)

# ax.set_xlabel('X Position')
# ax.set_ylabel('Y Position')
# ax.set_zlabel('Z Position')
# plt.show()
# 3D Plot ends ------------------