import rospy
from std_msgs.msg import String
from rocket_vis.msg import RocketData
from geometry_msgs.msg import Vector3
import re
import numpy as np

class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_estimate):
        self.estimate = initial_estimate
        self.estimate_error = 1.0
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def update(self, measurement):
        # Prediction
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance

        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate

class RocketDataCalculator:
    def __init__(self):
        self.previous_linear_velocity = [0.0, 0.0, 0.0]
        self.previous_angular_position = [0.0, 0.0, 0.0]
        self.previous_data_time = None

        # Initialize Kalman filter for altitude
        process_variance = 1.0  # You can adjust this value based on the noise in your data
        measurement_variance = 10.0  # You can adjust this value based on the noise in your data
        initial_estimate = 0.0  # Initial estimate for altitude
        self.altitude_filter = KalmanFilter1D(process_variance, measurement_variance, initial_estimate)

    def extract_values(self, serial_string):
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
            accel_x = float(match.group(4))
            accel_y = float(match.group(5))
            accel_z = float(match.group(6))/100
            gyro_x = float(match.group(7))
            gyro_y = float(match.group(8))
            gyro_z = float(match.group(9))
            return [temperature, pressure, altitude, (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z)]
        else:
            return None

    def calculate_linear_velocity(self, acceleration, time_interval):
        linear_velocity_x = acceleration[0] * time_interval 
        linear_velocity_y = acceleration[1] * time_interval 
        linear_velocity_z = acceleration[2] * time_interval 
        return [linear_velocity_x, linear_velocity_y, linear_velocity_z]

    def calculate_angular_position(self, angular_velocity, time_interval):
        angular_position = [0.0, 0.0, 0.0]
        for i in range(3):
            angular_position[i] = angular_velocity[i] / 57.3248
        return angular_position

    def arduino_data_callback(self, data):
        current_data_time = rospy.get_time()

        extracted_data = self.extract_values(data.data)

        if extracted_data:
            if self.previous_data_time is not None:
                time_interval = current_data_time - self.previous_data_time

                # Apply Kalman filter to altitude
                altitude_measurement = extracted_data[2]
                filtered_altitude = self.altitude_filter.update(altitude_measurement)

                # Use the filtered altitude in the RocketData message
                linear_velocity = self.calculate_linear_velocity(extracted_data[3], time_interval)
                angular_velocity = extracted_data[4]
                angular_position = self.calculate_angular_position(angular_velocity, time_interval)

                rocket_msg = RocketData()
                rocket_msg.altitude = filtered_altitude
                rocket_msg.velocity = Vector3(linear_velocity[0], linear_velocity[1], linear_velocity[2])
                rocket_msg.angular_position = Vector3(angular_position[0], angular_position[1], angular_position[2])
                rocket_data_pub.publish(rocket_msg)

                self.previous_linear_velocity = linear_velocity
                self.previous_angular_position = angular_position

            self.previous_data_time = current_data_time

if __name__ == '__main__':
    rospy.init_node('arduino_to_rocket_data', anonymous=True)
    rocket_data_pub = rospy.Publisher('rocket_data', RocketData, queue_size=10)

    calculator = RocketDataCalculator()

    rospy.Subscriber('arduino_data', String, calculator.arduino_data_callback)
    rospy.spin()
