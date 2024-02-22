import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt
from ahrs.filters import Mahony
from ahrs.common.orientation import q2R
import time

def stream_data(file_name, chunk_size=1):
    """ Generator to simulate streaming data from a CSV file. """
    for chunk in pd.read_csv(file_name, chunksize=chunk_size):
        gyro = chunk[['gyroscopex', 'gyroscopey', 'gyroscopez']].values * np.pi / 180
        acc = chunk[['accelerometerx', 'accelerometery', 'accelerometerz']].values
        yield gyro, acc

# Sample Period
samplePeriod = 1/256


# Initialize the AHRS algorithm
ahrs = Mahony(sample_period=1/256, kp=1)
Q = np.array([1., 0., 0., 0.])  # Initial quaternion


# High-pass filter setup
order = 1
filtCutOff = 0.1  # Cutoff frequency in Hz
fs = 1 / samplePeriod  # Sampling frequency in Hz
nyquist_freq = fs / 2  # Nyquist frequency

# Normalize the cutoff frequency
Wn = filtCutOff / nyquist_freq

# Now create the filter coefficients
b, a = butter(order, Wn, 'high')

# Setup for 3D animated plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Real-time High-pass filtered Linear Position')
line, = ax.plot([], [], [], lw=2)

# Setting the axes properties dynamically
ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([-1, 1])

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')

# Lists to store computed values
linPosHP_list = []

# Setup for 3D animated plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Real-time High-pass filtered Linear Position')

# Initial point
point, = ax.plot([], [], [], 'ro')

# Setting the axes properties dynamically
ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([-1, 1])

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')

def init():
    point.set_data([], [])
    point.set_3d_properties([])
    return point,

def animate(i):
    try:
        gyro, acc = next(data_stream)
        global Q
        Q = ahrs.updateIMU(Q, gyro[0], acc[0])
        R = q2R(Q)
        tcAcc = R.dot(acc[0])
        linAcc = (tcAcc - np.array([0, 0, 1])) * 9.81
        linVel = linAcc * samplePeriod
        linPos = np.cumsum(linVel)
        linPosHP_list.append(linPos)

        if len(linPosHP_list) > 300:  # Ensure enough data for filtfilt
            linPosHP = filtfilt(b, a, np.array(linPosHP_list), axis=0)
        else:
            linPosHP = np.array(linPosHP_list)

        if len(linPosHP) > 0:
            # Update the point's position to the latest data
            latest_point = linPosHP[-1]
            point.set_data([latest_point[0]], [latest_point[1]])
            point.set_3d_properties([latest_point[2]])

            # Update the axes limits dynamically if needed
            # ax.set_xlim3d([np.min(linPosHP[:,0]), np.max(linPosHP[:,0])])
            # ax.set_ylim3d([np.min(linPosHP[:,1]), np.max(linPosHP[:,1])])
            # ax.set_zlim3d([np.min(linPosHP[:,2]), np.max(linPosHP[:,2])])

    except StopIteration:
        return point,
    
    return point,


# Stream data from the CSV
data_stream = stream_data('data.csv', chunk_size=1)

# Call the animator. blit=True means only re-draw the parts that have changed.
ani = FuncAnimation(fig, animate, init_func=init, frames=200, interval=200, blit=True, repeat=False)

plt.show()
