#!/usr/bin/env python3

import random
import rospy
from std_msgs.msg import String

def data_gen():
    return f"Pressure : {random.randint(0, 10)}, Temperature : {random.randint(0, 10)} , Altitude : {random.randint(0, 10)}, Acceleration : ({random.randint(0, 10)},{random.randint(0, 10)},{random.randint(0, 10)}) Orientation : ({random.randint(0, 10)}, {random.randint(0, 10)}, {random.randint(0, 10)})"

def serial_node():
    rospy.init_node('arduino_serial_node')
    pub = rospy.Publisher('arduino_data', String, queue_size=10)


    rate = rospy.Rate(10)  # Publishing rate in Hz

    while not rospy.is_shutdown():
        data = data_gen()
        rospy.loginfo("Received data: %s", data)
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
