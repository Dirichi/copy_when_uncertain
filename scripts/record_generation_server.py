#!/usr/bin/env python

import rospy
import random
from copy_when_uncertain.srv import RecordGeneration, RecordGenerationRequest, RecordGenerationResponse
from std_msgs.msg import Int64
import copy
import re
import csv
from datetime import datetime

NAME = "record_generation_server"

EXPLORE = 0
EXPLOIT = 1
OBSERVE = 2

class RecordGenerationServer():
	def __init__(self, filename):
		self.filename = filename

	def handle_request(self, request):
		rospy.logdebug("Handling record request at tick_id: %s", request.tick_id)
		with open(self.filename, 'a') as f:
			rospy.logdebug("Writing to file: %s", self.filename)
			row = self.build_row(request)
			rospy.loginfo(row)
			headers = row.keys()
			headers.sort()
			writer = csv.DictWriter(f, headers)
			writer.writerow(row)

		rospy.logdebug("Completed recording at tick_id: %s", request.tick_id)
		return RecordGenerationResponse(tick_id=request.tick_id)

	def fitness(self, bee):
		return float(bee.total_reward) / bee.num_ticks
 
	def build_row(self, request):
		fitness_scores = [self.fitness(bee) for bee in request.bee_updates]
		avg_fitness = float(sum(fitness_scores)) / len(fitness_scores)
		fittest = max(request.bee_updates, key=lambda b: self.fitness(b))
		all_genes = [g for bee in request.bee_updates for g in bee.gene]
		fittest_gene = '-'.join([str(g) for g in fittest.gene])
		return {
			'tick_id': request.tick_id,
			'max_fitness': max(fitness_scores),
			'avg_fitness': avg_fitness,
			'fittest_gene': fittest_gene,
			'avg_explore_ratio': self.ratio(all_genes, EXPLORE),
			'avg_exploit_ratio': self.ratio(all_genes, EXPLOIT),
			'avg_observe_ratio': self.ratio(all_genes, OBSERVE),
			'fittest_explore_ratio': self.ratio(fittest.gene, EXPLORE),
			'fittest_exploit_ratio': self.ratio(fittest.gene, EXPLOIT),
			'fittest_observe_ratio': self.ratio(fittest.gene, OBSERVE)
		}	

	def ratio(self, arr, val):
		return float(len([v for v in arr if v == val])) / len(arr) 

def start_server():
	rospy.init_node(NAME)
	filename_suffix = re.sub(r'[\W_]+', '', str(datetime.now())) + '.csv'
	filename = rospy.get_param("~filename_prefix") + filename_suffix
	server = RecordGenerationServer(filename)
	rospy.Service("record_generation", RecordGeneration, server.handle_request)
	rospy.loginfo("Ready to record generations of bees. Saving files to %s", filename)
	rospy.spin()

if __name__ == "__main__":
	start_server()


