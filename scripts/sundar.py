#!/usr/bin/env python3
import re
import rospy
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import String

# Global variables to store data for plotting
timestamps = []
pressure_data = []
altitude_data = []
temperature_data = []
acceleration_data_x = []
acceleration_data_y = []
acceleration_data_z = []
orient_data_x = []
orient_data_y = []
orient_data_z = []
data_lock = threading.Lock()

def extract_values(serial_string):
    pattern = (
        r"Temperature:([\d.-]+),"
        r"Pressure:([\d.-]+),"
        r"Altitude:([\d.-]+),"
        r"AccelX:([\d.-]+),"
        r"AccelY:([\d.-]+),"
        r"AccelZ:([\d.-]+),"
        r"GyroX:([\d.-]+),"
        r"GyroY:([\d.-]+),"
        r"GyroZ:([\d.-]+)"
    )
    match = re.match(pattern, serial_string)

    if match:
        temperature = float(match.group(1))
        pressure = float(match.group(2))
        altitude = float(match.group(3))
        a_x = float(match.group(4))
        a_y = float(match.group(5))
        a_z = float(match.group(6))
        g_x = float(match.group(7))
        g_y = float(match.group(8))
        g_z = float(match.group(9))
        return [temperature, pressure, altitude, (a_x, a_y, a_z), (g_x, g_y, g_z)]
    else:
        return None

def callback(data):
    rospy.loginfo("Received: %s", data.data)
    raw_data = extract_values(data.data)
    if raw_data:
        pressure, temperature, altitude, accel, orientation = raw_data
        with data_lock:
            timestamps.append(rospy.Time.now().to_sec())
            pressure_data.append(pressure)
            altitude_data.append(altitude)
            temperature_data.append(temperature)
            acceleration_data_x.append(accel[0])
            acceleration_data_y.append(accel[1])
            acceleration_data_z.append(accel[2])
            orient_data_x.append(orientation[0])
            orient_data_y.append(orientation[1])
            orient_data_z.append(orientation[2])

def update_plot(frame):
    with data_lock:
        plt.clf()
        max_display_points = 100
        start_index = max(0, len(timestamps) - max_display_points)

        plot_timestamps = timestamps[start_index:]
        plot_pressure = pressure_data[start_index:]
        plot_altitude = altitude_data[start_index:]
        plot_temperature = temperature_data[start_index:]
        plot_acceleration_x = acceleration_data_x[start_index:]
        plot_acceleration_y = acceleration_data_y[start_index:]
        plot_acceleration_z = acceleration_data_z[start_index:]
        plot_orient_x = orient_data_x[start_index:]
        plot_orient_y = orient_data_y[start_index:]
        plot_orient_z = orient_data_z[start_index:]

        # plt.suptitle('Sensor Data Visualization', fontsize=18, fontweight='bold')
        
        # Subplot 1
        plt.subplot(5, 1, 1)
        plt.plot(plot_timestamps, plot_pressure, 'b-', label='Pressure (Pa)')
        plt.ylabel('Pressure (Pa)')
        plt.title('Pressure Data', fontsize=14)
        plt.legend()
        
        # Subplot 2
        plt.subplot(5, 1, 2)
        plt.plot(plot_timestamps, plot_altitude, 'g-', label='Altitude (m)')
        plt.ylabel('Altitude (m)')
        plt.title('Altitude Data', fontsize=14)
        plt.legend()
        
        # Subplot 3
        plt.subplot(5, 1, 3)
        plt.plot(plot_timestamps, plot_temperature, 'r-', label='Temperature (°C)')
        plt.ylabel('Temperature (°C)')
        plt.title('Temperature Data', fontsize=14)
        plt.legend()
        
        # Subplot 4
        plt.subplot(5, 1, 4)
        plt.plot(plot_timestamps, plot_acceleration_x, 'g-', label='a_x')
        plt.plot(plot_timestamps, plot_acceleration_y, 'b--', label='a_y')
        plt.plot(plot_timestamps, plot_acceleration_z, 'r-.', label='a_z')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Acceleration Data', fontsize=14)
        plt.legend(loc='upper right')
        
        # Subplot 5
        plt.subplot(5, 1, 5)
        plt.plot(plot_timestamps, plot_orient_x, 'g-', label='g_x')
        plt.plot(plot_timestamps, plot_orient_y, 'b--', label='g_y')
        plt.plot(plot_timestamps, plot_orient_z, 'r-.', label='g_z')
        plt.ylabel('Orientation (°)')
        plt.title('Orientation Data', fontsize=14)
        plt.legend(loc='upper right')

        plt.grid(True)
        plt.tight_layout()
        plt.xlabel('Time (s)')

def receiver():
    rospy.init_node('receiver_node')
    rospy.Subscriber('arduino_data', String, callback)

    plt.figure(figsize=(12, 10))
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1)

    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Value', fontsize=12)
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    receiver()

