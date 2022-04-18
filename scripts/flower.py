#!/usr/bin/env python

import rospy
import random
import numpy as np
from copy_when_uncertain.msg import FlowerUpdate, FlowerAlive
from std_msgs.msg import Int64

FLOWER_UPDATE_TOPIC = "flower_updated"
NAME = "flower"

class Flower():
	def __init__(self, flower_id, reward_options, distribution, publisher, prob_reward_change):
		self.id = flower_id
		self.current_reward = 0
		self.reward_options = [0, 1, 2]
		self.reward_distribution = distribution
		self.publisher = publisher
		self.prob_reward_change = prob_reward_change
		

	def update(self, tick):
		rospy.loginfo("Flower %s received tick %s", self.id, tick.data)
		tick_id = tick.data
		if (random.random() < self.prob_reward_change):
			reward = np.random.choice(self.reward_options, 1, p=self.reward_distribution)
			self.current_reward = reward[0]
		status = {
				'tick_id': tick_id, 
				'flower_id': self.id,
				'current_reward': self.current_reward
			}
		if (tick_id % 100 == 0):
			rospy.loginfo("Flower status update: %s", status)

		update = FlowerUpdate(**status)
		self.publisher.publish(update)



def start_flower_node():
	pub = rospy.Publisher(FLOWER_UPDATE_TOPIC, FlowerUpdate, queue_size=10)
	rospy.init_node(NAME, anonymous=True)
	reward_options = [0, 1, 2]
	flower_id = rospy.get_param("~flower_id")
	rospy.loginfo("Starting flower node with id: %s", flower_id)
	reward_change_freq = rospy.get_param("~prob_reward_change")
	prob_r0 = rospy.get_param("~prob_r0")
	prob_r1 = rospy.get_param("~prob_r1")
	prob_r2 = rospy.get_param("~prob_r2")
	distribution = [prob_r0, prob_r1, prob_r2]
	flower = Flower(flower_id, reward_options, distribution, pub, reward_change_freq)
	rospy.Subscriber("/update_flowers", Int64, flower.update)

	alive_pub = rospy.Publisher("flower_alive", FlowerAlive, queue_size=10)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		status = {'flower_id': flower.id}
		alive_pub.publish(FlowerAlive(**status))
		rate.sleep()

if __name__ == "__main__":
	start_flower_node()


