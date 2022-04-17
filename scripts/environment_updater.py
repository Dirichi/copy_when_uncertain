#!/usr/bin/env python

import rospy
import random
from copy_when_uncertain.msg import BeeUpdate, FlowerUpdate, BeeAlive, FlowerAlive, ActionOutcome, ActionOutcomeProcessed

from std_msgs.msg import Int64
import copy

NAME = "environment_updater"
UPDATE_FLOWERS_TOPIC = "update_flowers"
UPDATE_BEES_TOPIC = "update_bees"
TICK_COMPLETED_TOPIC = "tick_completed"
OUTCOME_TOPIC = "outcomes"

EXPLORE = 0
EXPLOIT = 1
OBSERVE = 2

class ActionOutcomeCalculator():
	def __init__(self, observation_noise_factor):
		self.observation_noise_factor = observation_noise_factor

	def calculate(self, bees_by_id, flowers_by_id):
		bees = [self.assign_flower_if_missing(bee, flowers_by_id) for bee in bees_by_id.values()]
		exploiters = [bee for bee in bees if bee.action == EXPLOIT]
		flower_profiles = self.build_flower_profiles(flowers_by_id, exploiters)
		return [self.build_outcome(bee, flower_profiles) for bee in bees]	

	def build_outcome(self, bee, flower_profiles_by_id):
		flower = self.select_target_flower(bee, flower_profiles_by_id)
		rewards = self.build_reward(bee, flower) 
		outcome = {
			'tick_id': bee.tick_id,
			'bee_id': bee.bee_id,
			'received_reward': rewards['received'],
			'observed_flower_id': flower['id'],
			'observed_reward': rewards['expected'],
		}
		return  ActionOutcome(**outcome)

	def build_reward(self, bee, flower):
		total = float(flower['reward'])
		n_bees = float(flower['n_bees'])
		received = (total / n_bees) if bee.action == EXPLOIT else 0
		expected = received if bee.action == EXPLOIT else total / (n_bees + 1)
		if bee.action == EXPLORE:
			expected = self.add_observation_noise(expected)

		return {'expected': expected,'received': received}

	def select_target_flower(self, bee, flower_profiles_by_id):
		flower_profiles = flower_profiles_by_id.values()
		if bee.action == EXPLORE:
			return random.sample(flower_profiles, 1)[0]

		if bee.action == EXPLOIT:
			return flower_profiles_by_id[bee.target_flower_id]

		if bee.action == OBSERVE:
			exploited_flowers = [f for f in flower_profiles if f['n_bees'] > 0]
			if len(exploited_flowers) == 0:
				return {'n_bees': 0, 'id': -1, 'reward': 0}
			return random.sample(exploited_flowers, 1)[0]

	def build_flower_profiles(self, flower_updates, exploiting_bees):
		flower_updates_list = flower_updates.values()		
		count_by_flower = self.count_by_target_flower(exploiting_bees)
		
		profiles = {}
		for flower_update in flower_updates_list:
			n_bees = count_by_flower.get(flower_update.flower_id, 0)
			reward = flower_update.current_reward
			profile = {'n_bees': n_bees, 'id': flower_update.flower_id, 'reward': reward}
			profiles[flower_update.flower_id] = profile

		return profiles

	def count_by_target_flower(self, bee_updates):
		count_dict = {}
		for update in bee_updates:
			if not count_dict.get(update.target_flower_id, None):
				count_dict[update.target_flower_id] = 1
			else:
				count_dict[update.target_flower_id] += 1
		return count_dict

	def assign_flower_if_missing(self, bee_update, flower_updates):
		update = copy.deepcopy(bee_update)
		if update.action == EXPLORE and update.target_flower_id == -1:
			flower_id = random.sample(flower_updates.keys(), 1)[0]
			update.target_flower_id = flower_id
		return update

	def add_observation_noise(self, reward):
		delta_factor = 0.0001
		delta = random.random() * (2 * delta_factor) - delta_factor
		noise_range = self.observation_noise_factor * (reward + delta)
		noise = (random.random() * (2 * noise_range)) - noise_range
		return reward + noise
	
		
	
class EnvironmentUpdater():
	def __init__(self, bee_publisher, flower_publisher, tick_completed_publisher, outcome_publisher):
		self.bee_publisher = bee_publisher
		self.flower_publisher = flower_publisher
		self.tick_completed_publisher = tick_completed_publisher
		self.outcome_publisher = outcome_publisher
		self.flower_updates = {}
		self.flower_ids = set([])
		self.bee_updates = {}
		self.bee_ids = set([])
		self.tick_id = 0
		self.n_flowers = 3
		self.n_bees = 5
		self.outcome_calculator = ActionOutcomeCalculator(0.1)
		self.bee_ids_with_unprocessed_outcomes = set([])
	
	def tick(self, tick):
		self.tick_id = tick.data + 1
		rospy.loginfo("Running tick %s", self.tick_id)
		self.update_flowers()

	def update_flowers(self):
		rospy.loginfo("Sending update request to flowers")
		self.flower_publisher.publish(Int64(self.tick_id))

	def receive_flower_update(self, flower_data):
		rospy.loginfo("Received update from flower with id: %s", flower_data.flower_id)
		old_updates = copy.deepcopy(self.flower_updates)
		self.flower_updates[flower_data.flower_id] = flower_data
		if self.updates_just_completed(old_updates, self.flower_updates, self.n_flowers):
			rospy.loginfo("Flower updates just completed. Starting bee updates")
			self.update_bees()

	def update_bees(self):
		rospy.loginfo("Sending update request to bees")
		self.bee_publisher.publish(Int64(self.tick_id))	

	def receive_bee_update(self, bee_data):
		rospy.loginfo("Received update from bee with id: %s", bee_data.bee_id)
		old_updates = copy.deepcopy(self.bee_updates)
		self.bee_updates[bee_data.bee_id] = bee_data
		if self.updates_just_completed(old_updates, self.bee_updates, self.n_bees):
			rospy.loginfo("Bee actions for tick: %s completed", self.tick_id)
			self.publish_outcomes()
	
	def receive_bee_alive(self, bee_data):
		self.bee_ids.add(bee_data.bee_id)

	def receive_flower_alive(self, flower_data):
		self.flower_ids.add(flower_data.flower_id)

	def all_objects_ready(self):
		return len(self.bee_ids) == self.n_bees and len(self.flower_ids) == self.n_flowers
	
	def publish_outcomes(self):
		rospy.loginfo("Publishing outcomes for tick: %s", self.tick_id)
		outcomes = self.outcome_calculator.calculate(self.bee_updates, self.flower_updates)
		self.bee_ids_with_unprocessed_outcomes = self.bee_ids.copy()
		for outcome in outcomes:
			self.outcome_publisher.publish(outcome)

	def receive_outcome_processed(self, outcome_processed_data):
		bee_id = outcome_processed_data.bee_id
		rospy.loginfo("Outcomes processed for bee: %s at tick: %s", bee_id, self.tick_id)
		self.bee_ids_with_unprocessed_outcomes.remove(bee_id)
		rospy.loginfo("Unprocessed outcome ids: %s", self.bee_ids_with_unprocessed_outcomes)
		if len(self.bee_ids_with_unprocessed_outcomes) == 0:
			self.tick_completed_publisher.publish(Int64(self.tick_id))
			

	def updates_just_completed(self, old_v, new_v, n_objs):
		return (not self.updates_complete(old_v, n_objs)) and self.updates_complete(new_v, n_objs)

	def updates_complete(self, updates_hash, n_objs):
		updates = updates_hash.values()
		if len(updates) < n_objs:
			return False

		tick_ids = set([update.tick_id for update in updates])
		return len(tick_ids) == 1 and self.tick_id in tick_ids

def start_environment_node():
	bee_pub = rospy.Publisher(UPDATE_BEES_TOPIC, Int64, queue_size=10, latch=True)
	flower_pub = rospy.Publisher(UPDATE_FLOWERS_TOPIC, Int64, queue_size=10, latch=True)
	tick_pub = rospy.Publisher(TICK_COMPLETED_TOPIC, Int64, queue_size=10, latch=True)
	outcome_pub = rospy.Publisher(OUTCOME_TOPIC, ActionOutcome, queue_size=10, latch=True)
	env = EnvironmentUpdater(bee_pub, flower_pub, tick_pub, outcome_pub)
	rospy.init_node(NAME, anonymous=True)
	rospy.Subscriber("/bees/bee_updated", BeeUpdate, env.receive_bee_update)
	rospy.Subscriber("/flowers/flower_updated", FlowerUpdate, env.receive_flower_update)
	rospy.Subscriber("/bees/bee_alive", BeeAlive, env.receive_bee_alive)
	rospy.Subscriber("/flowers/flower_alive", FlowerAlive, env.receive_flower_alive)
	rospy.Subscriber("/bees/outcome_processed", ActionOutcomeProcessed, env.receive_outcome_processed)
	rospy.Subscriber(TICK_COMPLETED_TOPIC, Int64, env.tick)

	rate = rospy.Rate(1)
	while not env.all_objects_ready():
		rate.sleep()

	env.tick(Int64(0))
	rospy.spin()

if __name__ == "__main__":
	start_environment_node()

