#!/usr/bin/env python2

import numpy as np

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    MAX_STEERING_ANGLE = 0.34 # rad
    MAX_SPEED = 4 # m/s

    def __init__(self):
        # Publisher for driving commands
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.DISTANCE_THRESHOLD = rospy.get_param("wall_follower/distance_threshold") # When to switch from distance-based error to angle-based error
        self.ANGLE_ERROR_THRESHOLD = 35 # deg, punish overturning

        # PID parameters
        self.Kp = rospy.get_param("wall_follower/Kp") # default: 4.07
        self.Ki = rospy.get_param("wall_follower/Ki") # default: 0.000007
        self.Kd = rospy.get_param("wall_follower/Kd") # default: 0.002
        self.previous_error = 0
        self.previous_time = rospy.get_rostime()
        self.integral_value = 0
        self.INTEGRAL_LIMIT = rospy.get_param("wall_follower/integral_limit") # Limit integrator windup

        # Subscriber for laser scans
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.update_wall_belief)
        self.current_scan = None

        # Store current belief of where the wall is
        self.a = 0 # slope (relative to LIDAR)
        self.b = 0 # y-intercept (relative to LIDAR)
        self.wall_pub = rospy.Publisher("wall_follower/wall_visualization", Marker, queue_size=10)
        self.front_pub = rospy.Publisher("wall_follower/front_visualization", Marker, queue_size=10)

        # Publish current distance from desired wall
        self.dist_pub = rospy.Publisher("wall_follower/distance", Float64, queue_size=10)

    def update_wall_belief(self, scan):
        """
        Updates current belief in where the wall is.
        """
        self.current_scan = scan

        # Check LaserScan range bounds
        if self.SIDE == 1:
            # Left
            if np.deg2rad(20) < scan.angle_min or np.deg2rad(95) > scan.angle_max:
                raise ValueError("Can't see left of robot!")
            angle_start = np.deg2rad(20)
            angle_end = np.deg2rad(95)
        else:
            # Right
            if np.deg2rad(-95) < scan.angle_min or np.deg2rad(-20) > scan.angle_max:
                raise ValueError("Can't see right of robot!")
            angle_start = np.deg2rad(-95)
            angle_end = np.deg2rad(-20)

        # Update current wall belief
        a, b = self.wall_regression(angle_start, angle_end)
        self.a = a
        self.b = b

        # Visualize current wall belief
        x_marker = np.linspace(-1.0, 3.0, num=20) # From 1m behind to 3m ahead of the robot
        y_marker = a*x_marker + b
        VisualizationTools.plot_line(x_marker, y_marker, self.wall_pub, frame="laser", color=(1.,0.,0.))

        # Publish driving command
        cmd = self.get_drive_command() # PID control law
        self.drive_pub.publish(cmd)

    # ----------------------------------------

    def get_desired_index(self, desired_angle):
        """
        Get index for desired_angle.
        If desired_angle is not an exact angle in the range, returns the index just before.
            e.g. if angle_min=0, angle_increment=0.5, then
                self.get_desired_index(0.5) = 1
                self.get_desired_index(0.9) = 1

        desired_angle: rad
        """
        if desired_angle < self.current_scan.angle_min or desired_angle > self.current_scan.angle_max:
            raise IndexError("Desired angle out of bounds.")
        elif desired_angle == self.current_scan.angle_min:
            return 0

        index = float(desired_angle - self.current_scan.angle_min)/self.current_scan.angle_increment
        return int(index)

    def ranges_to_positions(self, ranges, angle_start, angle_end):
        """
        Converts ranges to 2D array of (x,y) positions relative to LIDAR.

        ranges:      array-like
        angle_start: rad
        angle_end:   rad
        """
        angles = np.linspace(angle_start, angle_end, num=len(ranges))

        x = np.multiply(ranges, np.cos(angles))
        y = np.multiply(ranges, np.sin(angles))
        return np.vstack([x, y]).T

    def wall_regression(self, angle_start, angle_end, MAX_DIST_GAIN=2.0):
        """
        Find the linear regression parameters a, b of form (a*x + b) for the LIDAR angle range specified.

        angle_start: rad
        angle_end:   rad
        """
        # Get range of interest
        start = self.get_desired_index(angle_start)
        end = self.get_desired_index(angle_end)+1
        ranges = np.array(self.current_scan.ranges[start:end])

        # Convert to (x,y) points relative to LIDAR
        pos_out = self.ranges_to_positions(ranges, angle_start, angle_end)

        # Do not use far away points
        positions = []
        for i in range(len(pos_out)):
            norm = np.linalg.norm(pos_out[i])
            if norm > 0.10 and norm < MAX_DIST_GAIN*self.DESIRED_DISTANCE:
                positions.append((pos_out[i][0], pos_out[i][1]))
        positions = np.array(positions)

        if len(positions) == 0:
            if MAX_DIST_GAIN > 8.0:
                return (self.a, self.b) # Give up trying to find wall  
            # Otherwise, try increasing MAX_DIST_GAIN
            return self.wall_regression(angle_start, angle_end, MAX_DIST_GAIN*2.0)

        # Least squares regression
        x, y = np.hsplit(positions, 2)
        A = np.vstack([x.T[0], np.ones(len(x))]).T
        a, b = np.linalg.lstsq(A, y, rcond=1e-9)[0] # a*x + b
        return (a, b)

    def get_drive_command(self):
        """
        Uses current wall belief in PID controller to construct a drive command.
        """
        now = rospy.get_rostime()
        dt = (now - self.previous_time).nsecs / 10e9

        # ---- DISTANCE-BASED ERROR ----
        error_here = self.b - self.SIDE*self.DESIRED_DISTANCE # Distance from wall
        error_look_ahead = (self.a*self.DESIRED_DISTANCE + self.b) - self.SIDE*self.DESIRED_DISTANCE # Distance from wall at look ahead distance
        error = (error_here + error_look_ahead)/2
        msg = Float64()
        msg.data = error_here
        self.dist_pub.publish(msg)

        # ---- ANGLE-BASED ERROR ----
        angle_error = np.arccos(np.dot([1,0],[1,self.a])/np.linalg.norm([1,self.a])) # rad
        angle_error = angle_error * sign(np.cross([1,0],[1,self.a])) # signed angle

        # ---- CHECK FOR WALL IN FRONT OF CAR ----
        a_front, b_front = self.wall_regression(np.deg2rad(-2), np.deg2rad(2))
        if a_front != 0 and abs(-b_front/a_front) < self.DESIRED_DISTANCE*self.VELOCITY:
            # Wall right in front of car, turn!
            rospy.loginfo("Wall follower: INACTIVE, detected wall -> turning")
            control_value = np.deg2rad(self.SIDE*-25)
        else:
            # rospy.loginfo("Wall follower: ACTIVE")
            # ---- PID ----
            if dt > 0:
                # Distance-based error
                p = self.Kp * error
                i = self.integral_value + self.Ki * (error * dt)
                d = self.Kd * (error - self.previous_error) / dt
                if abs(error_here) > self.DISTANCE_THRESHOLD:
                    # Only punish angle error if outside distance threshold, otherwise fix
                    if sign(angle_error) != self.SIDE and abs(np.rad2deg(angle_error)) > self.ANGLE_ERROR_THRESHOLD:
                        p = angle_error # Unity feedback
                        i = 0
                        d = 0
            else:
                # Disable controller if time errors
                p = 0
                i = 0
                d = 0

            # Prevent integrator windup
            if abs(i) > self.INTEGRAL_LIMIT:
                i = self.INTEGRAL_LIMIT * sign(i)
            if sign(error) != sign(self.previous_error):
                i = 0
            control_value = p + i + d
        # ----

        # Visualize wall in front of car
        y_marker = np.linspace(-0.5, 0.5, num=20) # From 0.5m left to 0.5m right of the robot
        x_marker = (y_marker - b_front)/a_front
        if min(x_marker) > 0:
            # Only plot if wall is actually in front
            VisualizationTools.plot_line(x_marker, y_marker, self.front_pub, frame="laser", color=(0.,0.,1.))

        # ---- ISSUE COMMAND ----
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        if abs(control_value) > self.MAX_STEERING_ANGLE:
            control_value = self.MAX_STEERING_ANGLE * sign(control_value)
        cmd.drive.steering_angle = control_value # rad
        cmd.drive.steering_angle_velocity = 0
        cmd.drive.speed = self.VELOCITY # m/s
        if abs(cmd.drive.speed) > self.MAX_SPEED:
            cmd.drive.speed = self.MAX_SPEED * sign(cmd.drive.speed)
        cmd.drive.acceleration = 0
        cmd.drive.jerk = 0

        self.previous_error = error
        self.previous_time = now

        return cmd

def sign(x):
    """
    Returns parity of value.
    """
    if x == 0:
        return 0
    return x/abs(x)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
