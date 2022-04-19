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
		rospy.loginfo("Handling record request at tick_id: %s", request.tick_id)
		with open(self.filename, 'a') as f:
			rospy.logdebug("Writing to file: %s", self.filename)
			row = self.build_row(request)
			rospy.loginfo(row)
			writer = csv.DictWriter(f, row.keys())
			writer.writerow(row)

		rospy.logdebug("Completed recording at tick_id: %s", request.tick_id)
		return RecordGenerationResponse(tick_id=request.tick_id)

	def fitness(self, bee):
		return float(bee.total_reward) / bee.num_ticks
 
	def build_row(self, request):
		fittest = max(request.bee_updates, key=lambda b: self.fitness(b))
		all_genes = [g for bee in request.bee_updates for g in bee.gene]
		fittest_gene = '-'.join([str(g) for g in fittest.gene])
		return {
			'tick_id': request.tick_id,
			'max_fitness': self.fitness(fittest),
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
	filename = rospy.get_param("~filename_prefix") + "_"  + filename_suffix
	server = RecordGenerationServer(filename)
	rospy.Service("record_generation", RecordGeneration, server.handle_request)
	rospy.logdebug("Ready to record generations of bees")
	rospy.spin()

if __name__ == "__main__":
	start_server()


