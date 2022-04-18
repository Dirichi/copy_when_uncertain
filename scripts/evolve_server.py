#!/usr/bin/env python

import rospy
import random
from copy_when_uncertain.srv import Evolve, EvolveRequest, EvolveResponse
from copy_when_uncertain.msg import BeeInit
from std_msgs.msg import Int64
import copy

NAME = "evolve_server"

class EvolveServer():
	def __init__(self, tournament_ratio, prob_recombination, gene_cardinality, prob_mutation):
		self.tournament_ratio = tournament_ratio
		self.prob_recombination = prob_recombination
		self.gene_cardinality = gene_cardinality
		self.prob_mutation = prob_mutation

	def handle_request(self, request):
		rospy.loginfo("Handling evolve request at tick_id: %s", request.tick_id)
		bees = request.bee_updates
		self.log_progress(bees)
		children = self.evolve(bees)
		new_generation = []
		for i in range(len(bees)):
			new_bee = self.build_bee(bees[i].bee_id, children[i])
			new_generation.append(new_bee)
		
		rospy.loginfo("Completed evolution. New generation: %s", new_generation)
		return EvolveResponse(bee_inits=new_generation, tick_id=request.tick_id)

	def evolve(self, population):
		parents = [self.tournament_selection(population) for _ in range(len(population))]
		pairs = self.pair_up_parents(parents)
		children_genes = [child for p1, p2 in pairs for child in self.cross_over(p1, p2)] 
		if len(children_genes) > len(population):
			children_genes.pop(random.randint(0, len(children_genes) - 1))

		return [self.mutate(child) for child in children_genes]

	def fitness(self, individual):
		return float(individual.total_reward) / individual.num_ticks

	def tournament_selection(self, population):
		tournament_size = int(self.tournament_ratio * len(population))
		tournament_participants = random.sample(population, tournament_size)
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
			if random.random() < self.prob_mutation:
				gene_copy[i] = random.randint(0, self.gene_cardinality - 1)
		return gene_copy

	def build_bee(self, bee_id, gene):
		return BeeInit(**{'bee_id': bee_id, 'gene': gene})
		
	def log_progress(self, bees):
		fittest_bee = max(bees, key=lambda b: self.fitness(b))
		rospy.loginfo("Fittest bee: %s", fittest_bee.gene)
		rospy.loginfo("Fittest bee score: %s", self.fitness(fittest_bee))


def start_server():
	server = EvolveServer(tournament_ratio=0.4, prob_recombination=0.9, gene_cardinality=3, prob_mutation=0.1)
	rospy.init_node(NAME)
	rospy.Service("evolve", Evolve, server.handle_request)
	rospy.loginfo("Ready to evolve bees")
	rospy.spin()

if __name__ == "__main__":
	start_server()


