#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
import numpy as np

class Metrics:
	# Keep track of quantitative metrics relating to the performance of the racecar controls.
	ALPHA = rospy.get_param("metrics/alpha")
	DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
	
	def __init__(self):
		self.n = 0.0 # Number of measurements received so far
		self.loss = 0.0
		self.score = 0.0

		rospy.wait_for_message("wall_follower/distance", Float64, timeout=5)
		rospy.Subscriber("wall_follower/distance", Float64, self.update_loss)
		self.loss_pub = rospy.Publisher("metrics/loss", Float64, queue_size=10)
		self.score_pub = rospy.Publisher("metrics/score", Float64, queue_size=10)

	def update_loss(self, distance):
		"""
		Update current loss metric using new distance from wall measurement.
		"""
		new_loss = self.n * self.loss
		new_loss += abs(distance.data - self.DESIRED_DISTANCE)

		self.n += 1
		new_loss = new_loss/self.n

		self.loss = new_loss
		self.loss_pub.publish(self.loss)
		self.update_score() # Use new loss value

	def update_score(self):
		"""
		Update current score using current loss value.
		"""
		self.score = 1.0/(1 + (self.ALPHA * self.loss)**2)
		self.score_pub.publish(self.score)
		# rospy.loginfo(self.score)

if __name__ == "__main__":
	rospy.init_node('metrics')
	metrics = Metrics()
	rospy.spin()