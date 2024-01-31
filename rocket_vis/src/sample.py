#!/usr/bin/env python

import rospy
from rocket_vis.msg import RocketData
from geometry_msgs.msg import Vector3

def publish_rocket_data():
    rospy.init_node('rocket_data_publisher', anonymous=True)
    rocket_data_pub = rospy.Publisher('rocket_data', RocketData, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rocket_msg = RocketData()
        rocket_msg.altitude = 5.0  # Set the altitude to 100.0 meters
        rocket_msg.velocity = Vector3(0.5,0.0, 0.0)  # Se 0t the velocity 
        rocket_msg.angular_position = Vector3(3.14, 0.0, 3.14)  # Set the angular position (roll, pitch, yaw)

        rocket_data_pub.publish(rocket_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_rocket_data()
    except rospy.ROSInterruptException:
        pass
