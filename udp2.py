import serial.tools.list_ports
import serial
import socket
import sys
import time

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

def main(verbose=False):
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

    try:
        while True:
            # Read data from the serial port
            if ser.in_waiting > 0:
                line = ser.readline()
                try:
                    decoded_line = line.decode().rstrip()  # Use 'ignore' to skip undecodable bytes
                    if verbose:
                        print(f"Serial: {decoded_line}")
                    # Send decoded data to the UDP server
                    sock.sendto(decoded_line.encode(), server_address)
                except UnicodeDecodeError as e:
                    print(f"Decode error: {e}")

                if verbose:
                    print(f"Sent to UDP: {decoded_line}")

                # Delay to prevent overwhelming the UDP server
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program interrupted by the user.")
    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    verbose = "--verbose" in sys.argv
    main(verbose)
