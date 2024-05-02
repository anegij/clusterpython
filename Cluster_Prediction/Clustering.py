# April 2024 -Anegi James
# Clustering code/module

import warnings

import numpy as np
import numpy.linalg as LA
import itertools


from abc import ABC, abstractmethod

class Clusterings(ABC):
    def __init__(self, obstacle_environment,tols,Cluster_environment):
        self.obstacle_environment = obstacle_environment
        self.ctol = tols[0]
        self.dtol = tols[1]
        self.c_env = Cluster_environment
        self.clusters = None
        self.index = set()

    def do_clustering(self):
        a = []
        b = []
        g = []
        d = []
        C = self.c_env
        Tf = 10.0   # Time to goal?
        #Tf = calculate_avg_switch_time()
        for obs in self.obstacle_environment._obstacle_list:
            a.append(obs.position)
            b.append(obs.linear_velocity)
            g.append(obs.goal)
            #d.append(obs.destination)
        # Could maybe have tuples of each obs/ somehow select pairs and have cost and distance for each factor
        #Tf_avg = assume constant time for this iteration  
        cost = self.calculate_cost(a,b,g)
        # identify low cost pairs
        # Find ED betwen low cost pairs
        # Some function to check if pairs are below tolerance 
        # Group agent pairs find cluster properties
        # Repeat process for cluster vs other single agents/ other unclustered agents
        # Update clustering class obj, and add to cluster_environment
        #
    







    def calculate_cost(self,set1,set2,goals):
        # input - positions and velocities along with goals for all obstacles in order
        # output- find pairs, evaluate cost between all pairs, identify low cost pairs
        a = list(itertools.combinations(set1,2))
        print(a[1])
        b = 1
 

            




    
        
            


