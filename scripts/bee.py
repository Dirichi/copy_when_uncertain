#!/usr/bin/env python

import rospy
import random
from copy_when_uncertain.msg import BeeUpdate, BeeAlive, FlowerAlive, ActionOutcome, ActionOutcomeProcessed
from std_msgs.msg import Int64

BEE_UPDATE_TOPIC = "bee_updated"
OUTCOME_PROCESSED_TOPIC = "outcome_processed"
NAME = "bee"

EXPLORE = 0
EXPLOIT = 1
OBSERVE = 2

class Bee():
	def __init__(self, bee_id, gene, publisher, outcome_processed_publisher):
		self.id = bee_id
		self.gene = gene
		self.publisher = publisher
		self.outcome_processed_publisher = outcome_processed_publisher
		self.gene_cursor = 0
		self.cumulative_reward = 0
		self.num_ticks = 0
		self.expected_rewards_by_flower_id = {}

	def restart(self, gene):
		self.gene = gene
		self.num_ticks = 0
		self.cumulative_reward = 0
		self.gene_cursor = 0
		self.expected_rewards_by_flower_id = {flower: 0.0 for flower in self.expected_rewards_by_flower_id}
		

	def update(self, tick):
		tick_id = tick.data
		self.num_ticks += 1
		if self.gene_cursor >= len(self.gene):
			self.gene_cursor = 0

		action = self.gene[self.gene_cursor]
		status = {
				'bee_id': self.id,
				'tick_id': tick_id, 
				'action': action,
				'target_flower_id': self.get_target_flower(action),
				'num_ticks': self.num_ticks,
				'gene': self.gene,
				'total_reward': self.cumulative_reward
			}
		if (tick_id % 100 == 0):
			rospy.logdebug("Bee status update: %s", status)	
		update = BeeUpdate(**status)
		self.publisher.publish(update)

	def register_flower(self, flower_data):
		flower_id = flower_data.flower_id
		if not self.expected_rewards_by_flower_id.get(flower_id, None):
			self.expected_rewards_by_flower_id[flower_id] = 0.0

	def get_target_flower(self, action):
		rewards = self.expected_rewards_by_flower_id
		if len(rewards) == 0:
			return -1

		if action == EXPLOIT: 
			if len(set(rewards.values())) == 1:
				return random.sample(rewards.keys(), 1)[0]

			return max(rewards, key=rewards.get)

		return -1

	def process_outcome(self, outcome_data):
		if outcome_data.bee_id != self.id:
			return

		if outcome_data.reset:
			self.restart(outcome_data.gene)
			self.outcome_processed_publisher.publish(ActionOutcomeProcessed(self.id))
			return

		self.cumulative_reward += outcome_data.received_reward
		flower = outcome_data.observed_flower_id
		if flower != -1:
			self.expected_rewards_by_flower_id[flower] = outcome_data.observed_reward
		self.outcome_processed_publisher.publish(ActionOutcomeProcessed(self.id))


def start_bee_node():
	pub = rospy.Publisher(BEE_UPDATE_TOPIC, BeeUpdate, queue_size=10)
	outcome_processed_pub = rospy.Publisher(OUTCOME_PROCESSED_TOPIC, ActionOutcomeProcessed, queue_size=10)
	rospy.init_node(NAME, anonymous=True)
	gene_length = 10
	gene = [random.randint(0, 2) for i in range(gene_length)]
	bee_id = rospy.get_param("~bee_id")
	rospy.loginfo("Bee initialized with id: %s"%bee_id)
	bee = Bee(bee_id, gene, pub, outcome_processed_pub)
	rospy.Subscriber("/update_bees", Int64, bee.update)
	rospy.Subscriber("/outcomes", ActionOutcome, bee.process_outcome)
	rospy.Subscriber("/flowers/flower_alive", FlowerAlive, bee.register_flower)

	rate = rospy.Rate(1)
	alive_pub = rospy.Publisher("bee_alive", BeeAlive, queue_size=10)
	while not rospy.is_shutdown():
		status = {'bee_id': bee.id}
		alive_pub.publish(BeeAlive(**status))
		rate.sleep()

if __name__ == "__main__":
	start_bee_node()


