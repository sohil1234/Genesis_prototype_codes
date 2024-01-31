#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rocket_vis.msg import RocketData
from geometry_msgs.msg import Vector3
import re

# Function to extract values from the Arduino data string
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

def arduino_data_callback(data):
    # Extract values from the Arduino data
    extracted_data = extract_values(data.data)

    if extracted_data:
        # Publish the extracted data on the "rocket_data" topic
        rocket_msg = RocketData()
        # rocket_msg.temperature = extracted_data[0]
        # rocket_msg.pressure = extracted_data[1]
        rocket_msg.altitude = extracted_data[2]
        rocket_msg.velocity = Vector3(extracted_data[3][0], extracted_data[3][1], extracted_data[3][2])
        rocket_msg.angular_position = Vector3(extracted_data[4][0], extracted_data[4][1], extracted_data[4][2])
        rocket_data_pub.publish(rocket_msg)

if __name__ == '__main__':
    rospy.init_node('arduino_to_rocket_data', anonymous=True)
    rocket_data_pub = rospy.Publisher('rocket_data', RocketData, queue_size=10)
    rospy.Subscriber('arduino_data', String, arduino_data_callback)
    rospy.spin()
