#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped


class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        # rospy.loginfo("init!")

        self.cone_mode = False

        if self.cone_mode:
            rospy.Subscriber("/relative_cone", ConeLocation,
                             self.relative_cone_callback)
        else:
            rospy.Subscriber("/relative_cone", ConeLocation,
                             self.relative_line_callback)

        # set in launch file; different for simulator vs racecar
        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
                                         AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
                                         ParkingError, queue_size=10)

        self.parking_distance = 0.75  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.prev_time = rospy.Time.now()

        self.prev_theta_err = 0.0
        self.prev_dist_err = 0.0

        self.running_theta_err = 0.0
        self.running_dist_err = 0.0

    def relative_cone_callback(self, msg):
        time = rospy.Time.now()

        dt = (time - self.prev_time).to_sec()

        self.prev_time = time

        # pos x in front
        self.relative_x = msg.x_pos
        # pos y to left
        self.relative_y = msg.y_pos

        dist = np.sqrt((self.relative_x**2.0)+(self.relative_y**2.0))
        dist_err = dist - self.parking_distance

        # pos theta to left
        theta_err = np.arctan2(self.relative_y, self.relative_x)

        self.running_theta_err += theta_err
        self.running_dist_err += dist_err

        d_theta_dt = (theta_err - self.prev_theta_err) / dt
        d_dist_dt = (dist_err - self.prev_dist_err) / dt

        self.prev_dist_err = dist_err
        self.prev_theta_err = theta_err

        # rospy.loginfo("------------")
        # rospy.loginfo("dist %s", dist_err)
        # rospy.loginfo("theta %s", theta_err)
        # rospy.loginfo("x %s", self.relative_x)
        # rospy.loginfo("y %s", self.relative_y)

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.header.frame_id = "base_link"

        drive_cmd.drive.steering_angle = 0.5 * theta_err + \
            0.0 * d_theta_dt + 0.0 * self.running_theta_err

        if dist_err > 1.0:
            drive_cmd.drive.speed = 1.0
        elif dist_err > 0.02:
            drive_cmd.drive.speed = dist_err
        elif dist_err > -0.02:
            drive_cmd.drive.speed = 0.0
        else:
            drive_cmd.drive.speed = dist_err
            drive_cmd.drive.steering_angle = -drive_cmd.drive.steering_angle

        drive_cmd.drive.steering_angle_velocity = 0.0
        drive_cmd.drive.acceleration = 0.0
        drive_cmd.drive.jerk = 0.0
        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def relative_line_callback(self, msg):
        time = rospy.Time.now()

        dt = (time - self.prev_time).to_sec()

        self.prev_time = time

        # pos x in front
        self.relative_x = msg.x_pos
        # pos y to left
        self.relative_y = msg.y_pos

        dist = np.sqrt((self.relative_x**2.0)+(self.relative_y**2.0))
        dist_err = dist

        # pos theta to left
        theta_err = np.arctan2(self.relative_y, self.relative_x)

        self.running_theta_err += theta_err
        self.running_dist_err += dist_err

        d_theta_dt = (theta_err - self.prev_theta_err) / dt
        d_dist_dt = (dist_err - self.prev_dist_err) / dt

        self.prev_dist_err = dist_err
        self.prev_theta_err = theta_err

        # rospy.loginfo("------------")
        # rospy.loginfo("dist %s", dist_err)
        # rospy.loginfo("theta %s", theta_err)
        # rospy.loginfo("x %s", self.relative_x)
        # rospy.loginfo("y %s", self.relative_y)

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.header.frame_id = "base_link"

        drive_cmd.drive.steering_angle = 0.5 * theta_err + \
            0.0 * d_theta_dt + 0.0 * self.running_theta_err

        drive_cmd.drive.speed = 1.0

        drive_cmd.drive.steering_angle_velocity = 0.0
        drive_cmd.drive.acceleration = 0.0
        drive_cmd.drive.jerk = 0.0
        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(
            (self.relative_x**2.0)+(self.relative_y**2.0))

        self.error_pub.publish(error_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
