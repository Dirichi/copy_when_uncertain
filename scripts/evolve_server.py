#!/usr/bin/env python

import rospy
import random
from copy_when_uncertain.srv import Evolve, EvolveRequest, EvolveResponse
from copy_when_uncertain.msg import BeeInit
from std_msgs.msg import Int64
import copy

NAME = "evolve_server"

class EvolveServer():
	def __init__(self, tournament_size, prob_recombination, gene_cardinality):
		self.tournament_size = tournament_size
		self.prob_recombination = prob_recombination
		self.gene_cardinality = gene_cardinality

	def handle_request(self, request):
		rospy.logdebug("Handling evolve request at tick_id: %s", request.tick_id)
		bees = request.bee_updates
		self.log_progress(bees)
		children = self.evolve(bees, request.next_generation_size)
		new_generation = [BeeInit(gene=gene) for gene in children]
		rospy.logdebug("Completed evolution. New generation: %s", new_generation)
		return EvolveResponse(bee_inits=new_generation, tick_id=request.tick_id)

	def evolve(self, population, next_generation_size):
		parents = [self.tournament_selection(population) for _ in range(len(population))]
		pairs = self.pair_up_parents(parents)
		children_genes = [child for p1, p2 in pairs for child in self.cross_over(p1, p2)] 
		children_genes = random.sample(children_genes, next_generation_size)
		return [self.mutate(child) for child in children_genes]

	def fitness(self, individual):
		return float(individual.total_reward) / individual.num_ticks

	def tournament_selection(self, population):
		tournament_participants = random.sample(population, self.tournament_size)
		return max(tournament_participants, key=lambda p: self.fitness(p))


	def cross_over(self, parent_one, parent_two):
		p1_gene, p2_gene = list(parent_one.gene), list(parent_two.gene)
		child_one, child_two = copy.deepcopy(p1_gene), copy.deepcopy(p2_gene)
		if random.random() < self.prob_recombination:
			cross_over_point = random.randint(1, len(p1_gene) - 2)
			child_one = p1_gene[:cross_over_point] + p2_gene[cross_over_point:]
			child_two = p2_gene[:cross_over_point] + p1_gene[cross_over_point:]
		return [child_one, child_two]
	
	def pair_up_parents(self, parents):
		pairs = []
		for i in range(0, len(parents), 2):
			p1 = parents[i]
			p2_idx = i + 1
			if (p2_idx >= len(parents)):
				p2_idx = random.randint(0, len(parents) - 1)
			p2 = parents[p2_idx]
			pairs.append([p1, p2])
		return pairs

	def mutate(self, gene):
		gene_copy = gene[:]
		for i in range(len(gene_copy)):
			if random.random() < (1.0 / len(gene_copy)):
				gene_copy[i] = random.randint(0, self.gene_cardinality - 1)
		return gene_copy

	def log_progress(self, bees):
		fittest_bee = max(bees, key=lambda b: self.fitness(b))
		rospy.logdebug("Fittest bee: %s", fittest_bee.gene)
		rospy.logdebug("Fittest bee score: %s", self.fitness(fittest_bee))


def start_server():
	server = EvolveServer(tournament_size=3, prob_recombination=0.9, gene_cardinality=3)
	rospy.init_node(NAME)
	rospy.Service("evolve", Evolve, server.handle_request)
	rospy.logdebug("Ready to evolve bees")
	rospy.spin()

if __name__ == "__main__":
	start_server()


