import serial.tools.list_ports
import serial
import socket
import sys
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define the column names based on the sensor data
columns = [
    'Orientation_X', 'Orientation_Y', 'Orientation_Z',
    'AngVelocity_X', 'AngVelocity_Y', 'AngVelocity_Z',
    'LinearAccel_X', 'LinearAccel_Y', 'LinearAccel_Z',
    'Magnetometer_X', 'Magnetometer_Y', 'Magnetometer_Z',
    'Accelerometer_X', 'Accelerometer_Y', 'Accelerometer_Z',
    'Gravity_X', 'Gravity_Y', 'Gravity_Z',
    'Temperature',
    'Calibration_System', 'Calibration_Gyro', 'Calibration_Accel', 'Calibration_Mag'
]

# Function to get the local IP address
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print("Could not get local IP:", e)
        return None

# Function to update the plots
def update(frame_data, df, lines, axs, window_size):
    t, sensor_values = frame_data
    new_row = dict(zip(columns, sensor_values))
    df.loc[t] = new_row  # Update the DataFrame with new data
    if not df.empty:
        for ax_idx, ax in enumerate(axs):
            ax.set_xlim(max(0, t - window_size), max(t, window_size))

            # Update each line with new data within each subplot
            for line_idx in range(3):  # 3 lines per subplot
                component = columns[ax_idx * 3 + line_idx]
                lines[ax_idx][line_idx].set_data(df.index[-window_size:], df[component][-window_size:])

    return [line for group in lines for line in group]  # Flatten the list of lines


def main(verbose=False, gui=False):
    # List all available serial ports
    ports = list(serial.tools.list_ports.comports())
    for i, p in enumerate(ports):
        print(f"[{i}] {p}")

    # Prompt the user to select a port
    port_index = int(input("Select the port for the IMU (e.g., 0, 1, 2, ...): "))
    arduino_port_name = ports[port_index].device

    try:
        # Open the selected serial port with baud rate 115200
        ser = serial.Serial(arduino_port_name, 115200, timeout=1)
        if verbose:
            print(f"Connected to {arduino_port_name}")
    except Exception as e:
        print(f"Error opening serial port {arduino_port_name}: {e}")
        return

    local_ip = get_local_ip()
    if local_ip is None:
        return

    if verbose:
        print(f"Local IP Address: {local_ip}")

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (local_ip, 12345)

    # Prepare DataFrame and Plot if GUI mode is enabled
    if gui:
        try:
            df = pd.DataFrame(columns=columns)
            fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 8))  # Adjust figsize as needed
            window_size = 100
            # Create line objects for each component in each subplot
            lines = [[ax.plot([], [], lw=2, label=col)[0] for col in columns[i*3:(i+1)*3]] for i, ax in enumerate(axs)]
            colors = ['b', 'g', 'r']
            for i, (group, ax) in enumerate(zip(lines, axs)):
                for line, color in zip(group, colors):
                    line.set_color(color)
                ax.legend(loc='upper left')
                ax.set_ylim(-180, 180) if i == 0 else ax.set_ylim(-20, 20)  # Orientation subplot has different y-limits

            def data_gen():
                t = 0
                while True:
                    if ser.in_waiting > 0:
                        line = ser.readline()
                        try:
                            decoded_line = line.decode().rstrip()
                            if verbose:
                                print("Received:", decoded_line)  # Debugging line
                            sensor_values = [float(v) for v in decoded_line.split(',')]
                            if len(sensor_values) == len(columns):
                                yield t, sensor_values
                            else:
                                print("Data length mismatch")  # Debugging line
                            t += 1
                        except Exception as e:
                            print(f"Error processing data: {e}")
            ani = FuncAnimation(fig, update, data_gen, fargs=(df, lines, axs, window_size), blit=False, interval=5)
            plt.show()
        # Add keyboard interrupt handling
        except KeyboardInterrupt:
            # Close all matplotlib windows
            plt.close("all")
            # Close the serial port and exit
            ser.close()
            print("Keyboard interrupt")
            # Exit
            sys.exit(0)

if __name__ == "__main__":
    verbose = "--verbose" in sys.argv
    gui = "--gui" in sys.argv
    main(verbose, gui)
