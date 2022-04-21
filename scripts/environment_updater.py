#!/usr/bin/env python

import rospy
import roslaunch
import random
from copy_when_uncertain.msg import BeeUpdate, FlowerUpdate, BeeAlive, FlowerAlive, ActionOutcome, ActionOutcomeProcessed
from copy_when_uncertain.srv import Evolve, EvolveRequest, RecordGeneration, RecordGenerationRequest
from std_msgs.msg import Int64
import copy
import numpy as np
from scipy.stats import gamma

NAME = "environment_updater"
UPDATE_FLOWERS_TOPIC = "update_flowers"
UPDATE_BEES_TOPIC = "update_bees"
TICK_COMPLETED_TOPIC = "tick_completed"
OUTCOME_TOPIC = "outcomes"

EXPLORE = 0
EXPLOIT = 1
OBSERVE = 2

PACKAGE = "copy_when_uncertain"
BEE_EXE = "bee.py"
FLOWER_EXE = "flower.py"


class ActionOutcomeCalculator():
	def __init__(self, observation_noise_factor, prob_reset, max_bee_ticks):
		self.observation_noise_factor = observation_noise_factor
		self.prob_reset = prob_reset
		self.max_bee_ticks = max_bee_ticks

	def calculate(self, bees_by_id, flowers_by_id):
		bees = bees_by_id.values()
		exploiters = [bee for bee in bees if bee.action == EXPLOIT and bee.target_flower_id != -1]
		flower_profiles = self.build_flower_profiles(flowers_by_id, exploiters)
		return [self.build_outcome(bee, flower_profiles) for bee in bees]	

	def build_outcome(self, bee, flower_profiles_by_id):
		flower = self.select_target_flower(bee, flower_profiles_by_id)
		rewards = self.build_reward(bee, flower) 
		reset = (bee.num_ticks > self.max_bee_ticks) or random.random() < self.prob_reset
		outcome = {
			'tick_id': bee.tick_id,
			'bee_id': bee.bee_id,
			'received_reward': rewards['received'],
			'observed_flower_id': flower['id'],
			'observed_reward': rewards['expected'],
			'reset': reset,
			'gene': []
		}
		return  ActionOutcome(**outcome)

	def build_reward(self, bee, flower):
		total = float(flower['reward'])
		n_bees = float(flower['n_bees'])
		received = (total / n_bees) if bee.action == EXPLOIT and bee.target_flower_id != -1 else 0
		expected = received if bee.action == EXPLOIT else total / (n_bees + 1)
		if bee.action == EXPLOIT:
			#expected = self.add_observation_noise(expected)
			pass

		return {'expected': expected,'received': received}

	def select_target_flower(self, bee, flower_profiles_by_id):
		flower_profiles = flower_profiles_by_id.values()
		if bee.action == EXPLORE:
			return random.sample(flower_profiles, 1)[0]

		if bee.action == EXPLOIT and bee.target_flower_id != -1:
			return flower_profiles_by_id[bee.target_flower_id]

		if bee.action == EXPLOIT and bee.target_flower_id == -1:
			return {'n_bees': 0, 'id': -1, 'reward': 0}

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

	def add_observation_noise(self, reward):
		delta_factor = 0.0001
		delta = random.random() * (2 * delta_factor) - delta_factor
		noise_range = self.observation_noise_factor * (reward + delta)
		noise = (random.random() * (2 * noise_range)) - noise_range
		return reward + noise
	
		
	
class EnvironmentUpdater():
	def __init__(self, bee_pub, flower_pub, tick_completed_pub, outcome_pub, evolution_freq, max_ticks, rewards_shape, rewards_scale, gene_length):
		self.bee_publisher = bee_pub
		self.flower_publisher = flower_pub
		self.tick_completed_publisher = tick_completed_pub
		self.outcome_publisher = outcome_pub
		self.evolution_frequency = evolution_freq 
		self.flower_updates = {}
		self.flower_ids = set([])
		self.bee_updates = {}
		self.bee_ids = set([])
		self.tick_id = 0
		self.n_flowers = 20
		self.n_bees = 6
		self.outcome_calculator = ActionOutcomeCalculator(observation_noise_factor=0, prob_reset=0.05, max_bee_ticks=gene_length)
		self.gene_length = gene_length
		self.processed_outcome_ids = set([])
		self.tick_outcome_published = False
		self.update_bees_published = False
		self.tick_completed = False
		self.max_ticks = max_ticks
		self.child_processes = []
		self.rewards_shape = rewards_shape
		self.rewards_scale = rewards_scale
	
	def setup(self):
		rospy.logdebug("Setup started")
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()
		self.start_bee_nodes(launch)
		self.start_flower_nodes(launch)
		rate = rospy.Rate(1)
		while not self.all_objects_ready():
			rate.sleep()

		rospy.logdebug("Setup complete")

	def tick(self, tick):
		if (self.tick_id >= self.max_ticks):
			for process in self.child_processes:
				process.stop()
			return

		self.bee_updates = {}
		self.flower_updates = {}
		self.processed_outcome_ids = set([])
		self.tick_id = tick.data + 1
		self.tick_outcome_published = False
		self.update_bees_published = False
		self.tick_completed = False
		rospy.logdebug("Running tick %s", self.tick_id)
		self.update_flowers()

	def update_flowers(self):
		rospy.logdebug("Sending update request to flowers")
		self.flower_publisher.publish(Int64(self.tick_id))

	def receive_flower_update(self, flower_data):
		rospy.logdebug("Received update from flower with id: %s", flower_data.flower_id)
		self.flower_updates[flower_data.flower_id] = flower_data
		if len(self.flower_updates.keys()) == self.n_flowers and not self.update_bees_published:
			self.update_bees_published = True
			rospy.logdebug("Flower updates just completed. Starting bee updates")
			self.update_bees()

	def update_bees(self):
		rospy.logdebug("Sending update request to bees")
		self.bee_publisher.publish(Int64(self.tick_id))	

	def receive_bee_update(self, bee_data):
		rospy.logdebug("Received update from bee with id: %s", bee_data.bee_id)
		self.bee_updates[bee_data.bee_id] = bee_data
		if len(self.bee_updates.keys()) == self.n_bees and not self.tick_outcome_published:
			self.tick_outcome_published = True
			rospy.logdebug("Bee actions for tick: %s completed", self.tick_id)
			self.publish_outcomes()
	
	def receive_bee_alive(self, bee_data):
		self.bee_ids.add(bee_data.bee_id)

	def receive_flower_alive(self, flower_data):
		self.flower_ids.add(flower_data.flower_id)

	def all_objects_ready(self):
		return len(self.bee_ids) == self.n_bees and len(self.flower_ids) == self.n_flowers
	
	def publish_outcomes(self):
		outcomes = self.build_outcomes()
		self.processed_outcome_ids = set([])
		rospy.logdebug("Publishing outcomes for tick: %s", self.tick_id)
		for outcome in outcomes:
			self.outcome_publisher.publish(outcome)

	def build_outcomes(self):
		if (self.tick_id > 0 and (self.tick_id % self.evolution_frequency == 0)):
			self.record_generation()

		bees = copy.deepcopy(self.bee_updates)
		flowers = copy.deepcopy(self.flower_updates)
		outcomes = self.outcome_calculator.calculate(bees, flowers)
		dead_bees = [bee for bee in outcomes if bee.reset]
		n_dead_bees = len(dead_bees)
		if (n_dead_bees > 0):
			bee_inits = self.evolve_bees(n_dead_bees)
		
			for i in range(n_dead_bees):
				bee_init = bee_inits[i]
				outcome = dead_bees[i]
				outcome.gene = bee_init.gene
		return outcomes

	def receive_outcome_processed(self, outcome_processed_data):
		bee_id = outcome_processed_data.bee_id
		rospy.logdebug("Outcomes processed for bee: %s at tick: %s", bee_id, self.tick_id)
		self.processed_outcome_ids.add(bee_id)
		rospy.logdebug("Processed outcome ids: %s", self.processed_outcome_ids)
		if len(self.processed_outcome_ids) == self.n_bees and not self.tick_completed:
			self.tick_completed = True
			rospy.logdebug("All action outcomes for tick: %s processed", self.tick_id)
			rospy.logdebug("Tick: %s completed", self.tick_id)
			self.tick_completed_publisher.publish(Int64(self.tick_id))

	def evolve_bees(self, num_bees):
		rospy.logdebug("Beginning evolution process at tick_id: %s", self.tick_id)
		rospy.wait_for_service('evolve')
		try:
			run_evolve = rospy.ServiceProxy('evolve', Evolve)
			request = EvolveRequest(
				bee_updates=self.bee_updates.values(), 
				tick_id=self.tick_id,
				next_generation_size=num_bees)
			rospy.logdebug("Completed evolution process")
			return run_evolve(request).bee_inits
		except rospy.ServiceException as e:
			rospy.logerr("Evolution service call failed: %s", e)
		return []

	def record_generation(self):
		rospy.logdebug("Recording generation at tick_id: %s", self.tick_id)
		rospy.wait_for_service('record_generation')
		try:

			record = rospy.ServiceProxy('record_generation', RecordGeneration)
			request = RecordGenerationRequest(bee_updates=self.bee_updates.values(), tick_id=self.tick_id)
			record(request)
		except rospy.ServiceException as e:
			rospy.logerr("Record service call failed: %s", e)

	
	def start_bee_nodes(self, launch):
		for i in range(self.n_bees):
			bee_id = i + 1
			node = roslaunch.core.Node(
				PACKAGE, 
				BEE_EXE,
				namespace="/bees",
				name="bee_" + str(bee_id),
				args="_bee_id:=%s _gene_length:=%s" % (bee_id, self.gene_length))
			process = launch.launch(node)
			self.child_processes.append(process)

	def start_flower_nodes(self, launch):
		distribution = gamma.rvs(self.rewards_shape, scale=self.rewards_scale, size=self.n_flowers)
		for i in range(self.n_flowers):
			flower_id = i + 1
			reward = distribution[i]
			node = roslaunch.core.Node(
				PACKAGE, 
				FLOWER_EXE, 
				namespace="/flowers",
				name="flower_" + str(flower_id),
				args="_flower_id:=%s _reward:=%s" % (flower_id, reward))
			process = launch.launch(node)
			self.child_processes.append(process)

def start_environment_node():
	bee_pub = rospy.Publisher(UPDATE_BEES_TOPIC, Int64, queue_size=10, latch=True)
	flower_pub = rospy.Publisher(UPDATE_FLOWERS_TOPIC, Int64, queue_size=10, latch=True)
	tick_pub = rospy.Publisher(TICK_COMPLETED_TOPIC, Int64, queue_size=10, latch=True)
	outcome_pub = rospy.Publisher(OUTCOME_TOPIC, ActionOutcome, queue_size=10, latch=True)
	rospy.init_node(NAME, anonymous=True)
	rewards_shape = rospy.get_param("~rewards_shape")
	rewards_scale = rospy.get_param("~rewards_scale")
	env = EnvironmentUpdater(bee_pub, flower_pub, tick_pub, outcome_pub, evolution_freq=10, max_ticks=10000, rewards_shape=rewards_shape, rewards_scale=rewards_scale, gene_length=20)
	rospy.Subscriber("/bees/bee_updated", BeeUpdate, env.receive_bee_update)
	rospy.Subscriber("/flowers/flower_updated", FlowerUpdate, env.receive_flower_update)
	rospy.Subscriber("/bees/bee_alive", BeeAlive, env.receive_bee_alive)
	rospy.Subscriber("/flowers/flower_alive", FlowerAlive, env.receive_flower_alive)
	rospy.Subscriber("/bees/outcome_processed", ActionOutcomeProcessed, env.receive_outcome_processed)
	rospy.Subscriber(TICK_COMPLETED_TOPIC, Int64, env.tick)

	env.setup()
	env.tick(Int64(0))
	rospy.spin()

if __name__ == "__main__":
	start_environment_node()

