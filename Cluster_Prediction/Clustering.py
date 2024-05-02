# April 2024 -Anegi James
# Clustering code/module

import warnings

import numpy as np
import numpy.linalg as LA


from abc import ABC, abstractmethod

class Clusterings(ABC):
    def __init__(self, obstacle_environment,tols):
        self.obstacle_environment = obstacle_environment
        self.ctol = tols[0]
        self.dtol = tols[1]
        self.clusters = None
    def do_clustering(self):
        a = []
        b = []
        g = []
        d = []
        Tf = 10.0   # Place holder value, sim has no set time
        for obs in self.obstacle_environment._obstacle_list:
            a.append(obs.position)
            b.append(obs.linear_velocity)
            g.append(obs.goal)
            #d.append(obs.destination)
        # Could maybe have tuples of each obs/ somehow select pairs and have cost and distance for each factor
        #Tf_avg = assume constant time for this iteration  
        cost = self.calculate_cost(a,b,g,d)




    #def calculate_cost(set1,set2):
        




    
        
            


