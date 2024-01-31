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
    pattern = r"Pressure : (\d+), Temperature : (\d+) , Altitude : (\d+), Acceleration : \((\d+),(\d+),(\d+)\) Orientation : \((\d+), (\d+), (\d+)\)"
    match = re.match(pattern, serial_string)
    
    if match:
        pressure = int(match.group(1))
        temperature = int(match.group(2))
        altitude = int(match.group(3))
        a_x = int(match.group(4))
        a_y = int(match.group(5))
        a_z = int(match.group(6))
        g_x = int(match.group(7))
        g_y = int(match.group(8))
        g_z = int(match.group(9))
        print([pressure, temperature, altitude, (a_x, a_y, a_z), (g_x, g_y, g_z)])
        return [pressure, temperature, altitude, (a_x, a_y, a_z), (g_x, g_y, g_z)]
    else:
        return None

def callback(data):
    # rospy.loginfo("Received: %s", data.data)
    raw_data = extract_values(data.data)
    if raw_data:
        pressure, temperature, altitude, accel, orientation = raw_data
        with data_lock:
            timestamps.append(rospy.Time.now().to_sec())
            pressure_data.append(pressure)
            altitude_data.append(altitude)
            temperature_data.append(temperature)
            acceleration_data_x.append(accel[0])  # Use the X-axis acceleration
            acceleration_data_y.append(accel[1])  # Use the Y-axis acceleration
            acceleration_data_z.append(accel[2])  # Use the Z-axis acceleration
            orient_data_x.append(orientation[0])  # Use the X-axis orientation
            orient_data_y.append(orientation[1])  # Use the Y-axis orientation
            orient_data_z.append(orientation[2])  # Use the Z-axis orientation



def update_plot(frame):
    with data_lock:
        plt.clf()
        
        # Calculate the maximum number of data points to display on the plot
        max_display_points = 100          
        # Calculate the starting index based on the number of data points
        start_index = max(0, len(timestamps) - max_display_points)
        
        # Select the relevant data for the plot
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
        
        # Create subplots for each variable
        plt.subplot(5, 1, 1)  # 5 rows, 1 column, subplot 1
        plt.plot(plot_timestamps, plot_pressure, 'b-')
        plt.ylabel('Pressure')

        plt.subplot(5, 1, 2)  # 5 rows, 1 column, subplot 2
        plt.plot(plot_timestamps, plot_altitude, 'b-')
        plt.ylabel('Altitude')
        
        plt.subplot(5, 1, 3)  # 5 rows, 1 column, subplot 3
        plt.plot(plot_timestamps, plot_temperature, 'r-')
        plt.ylabel('Temperature')
        
        plt.subplot(5, 1, 4)  # 5 rows, 1 column, subplot 4
        plt.plot(plot_timestamps, plot_acceleration_x, 'g-', label='a_x')
        plt.plot(plot_timestamps, plot_acceleration_y, 'b-', label='a_y')
        plt.plot(plot_timestamps, plot_acceleration_z, 'r-', label='a_z')
        plt.ylabel('Acceleration')
        plt.legend()
        
        plt.subplot(5, 1, 5)  # 5 rows, 1 column, subplot 5
        plt.plot(plot_timestamps, plot_orient_x, 'g-', label='g_x')
        plt.plot(plot_timestamps, plot_orient_y, 'b-', label='g_y')
        plt.plot(plot_timestamps, plot_orient_z, 'r-', label='g_z')
        plt.ylabel('Orientation')
        plt.legend()

        # Adjust layout and show the plot
        plt.tight_layout()
        plt.xlabel('Time (s)')


def receiver():
    rospy.init_node('receiver_node')

    rospy.Subscriber('arduino_data', String, callback)

    plt.figure(figsize=(100, 60))
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1)  # Update every 1 ms

    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.legend()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    receiver()