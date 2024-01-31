#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial

def serial_node():
    rospy.init_node('arduino_serial_node')
    pub = rospy.Publisher('arduino_data', String, queue_size=10)

    serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Replace with your Arduino's serial port and baud rate

    rate = rospy.Rate(500)  # Publishing rate in Hz

    while not rospy.is_shutdown():
        data = serial_port.readline().decode().strip()
        rospy.loginfo("Received data: %s", data)
        pub.publish(data)
        rate.sleep()

    serial_port.close()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
