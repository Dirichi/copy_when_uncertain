#!/usr/bin/env python

import rospy
import random
import numpy as np
from copy_when_uncertain.msg import FlowerUpdate, FlowerAlive
from std_msgs.msg import Int64

FLOWER_UPDATE_TOPIC = "flower_updated"
NAME = "flower"

class Flower():
	def __init__(self, flower_id, reward, publisher):
		self.id = flower_id
		self.current_reward = reward
		self.publisher = publisher
		

	def update(self, tick):
		rospy.logdebug("Flower %s received tick %s", self.id, tick.data)
		tick_id = tick.data
		status = {
				'tick_id': tick_id, 
				'flower_id': self.id,
				'current_reward': self.current_reward
			}
		if (tick_id % 100 == 0):
			rospy.logdebug("Flower status update: %s", status)

		update = FlowerUpdate(**status)
		self.publisher.publish(update)



def start_flower_node():
	pub = rospy.Publisher(FLOWER_UPDATE_TOPIC, FlowerUpdate, queue_size=10)
	rospy.init_node(NAME, anonymous=True)
	flower_id = rospy.get_param("~flower_id")
	rospy.logdebug("Starting flower node with id: %s", flower_id)
	reward = rospy.get_param("~reward")
	flower = Flower(flower_id, reward, pub)
	rospy.Subscriber("/update_flowers", Int64, flower.update)

	alive_pub = rospy.Publisher("flower_alive", FlowerAlive, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		status = {'flower_id': flower.id}
		alive_pub.publish(FlowerAlive(**status))
		rate.sleep()

if __name__ == "__main__":
	start_flower_node()


