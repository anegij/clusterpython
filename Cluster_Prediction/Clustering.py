# April 2024 -Anegi James
# Clustering code/module

import warnings

import numpy as np
import numpy.linalg as LA
import itertools


from abc import ABC, abstractmethod
from Cluster_Prediction import Metrics
from Cluster_Prediction import UTILS as util
from Cluster_Prediction.Cluster import Cluster

class Clusterings(ABC):
    def __init__(self, obstacle_environment,Cluster_environment):
        self.obstacle_environment = obstacle_environment
        tols = [1.0,1.0,1.0,0.65]
        self.ctol = tols[0]
        self.dtol = tols[1]
        self.vtol = tols[2]
        self.otol = tols[3]
        self.c_env = Cluster_environment
        self.clusters = None
        self.index = []
        self.method = "DistanceOnly"
        self.dim = 4
        self.numobs = obstacle_environment.n_obstacles




    def do_clustering(self):
        method = self.method
        a = []
        b = []
        g = []
        d = np.ndarray(shape=(3,5),dtype = float)
        C = self.c_env
        Tf = 10.0   # Time to goal?
        #Tf = calculate_avg_switch_time()
        count = 0
        for obs in self.obstacle_environment._obstacle_list: 
            d[count,0:2] = obs.position
            d[count,2:4] = obs.linear_velocity
            d[count,4] = count
            #a.append(np.concatenate((obs.position, obs.linear_velocity),axis=0))
            g.append(obs.goal)
            count = count +1 
            #d.append(obs.destination)
        
        # Could maybe have tuples of each obs/ somehow select pairs and have cost and distance for each factor
        #Tf_avg = assume constant time for this iteration  
        self.calculate_metrics(d,method)
        # identify low cost pairs
        # Find ED betwen low cost pairs
        # Some function to check if pairs are below tolerance 
        # Group agent pairs find cluster properties
        # Repeat process for cluster vs other single agents/ other unclustered agents
        # Update clustering class obj, and add to cluster_environment


    '''def calculate_cost(self,set1,goals,Tf):
        # input - positions and velocities along with goals for all obstacles in order
        # output- find pairs, evaluate cost between all pairs, identify low cost pairs
        
        a = list(itertools.combinations(set1,2))
        set1
        for pair in a:
            counter = 0
            Cost = Metrics.cost_metric(pair,Tf,goals)'''

    
 
    def calculate_metrics(self,b,g):
        # b is list of all agents
        # a is list of all pairs of agents
        a = list(itertools.combinations(b,2))
        Dist = np.empty(len(a),dtype=float)
        Vel = np.empty(len(a),dtype=float)
        Orient = np.empty(len(a),dtype=float)

        method = self.method
        if method == "DistanceOnly":
            counter = 0
            for pair in a:
                Dist[counter] = Metrics.distance_metric(pair[0],pair[1])
                Vel[counter] = Metrics.vel_distance(pair[0],pair[1])
                Orient[counter] = Metrics.orientation_distance(pair[0],pair[1])
                counter = counter + 1
            # Find min distance pair under dtol
            d_list = Dist < self.dtol
            v_list =  Vel < self.vtol
            o_list =  Orient < self.otol
            agent_list = d_list & v_list & o_list

            self.make_two_agent_cluster(agent_list,b,a)
            # Maybe update b to remove agents and add clusters
            # Reuse function to make cluster-cluster groupings 

        # Other methods
        elif method == 'CostMetrics':
            for pair in a:
                counter = 0
                Dist[counter] = Metrics.cost_metric(pair,Tf,g)
                counter =+ 1

    def make_two_agent_cluster(self,tol_list,agents,pair_list):
        # agents - array with all sgents in [x y vx vy] form
        # pair_list - list of all combinations of agents
        # Check if multiple agent pair satisfy metrics for clustering
        new_agents = np.copy(agents)
        cluster_no = -1
        # loop through all pairs
        for tols,pair in zip(tol_list, pair_list):
            if new_agents.size == 0:
                return agents
            if tols == 1:
                [p1,p2,idp] = util.parse_agent_dat(pair[0])
                [q1,q2,idq] = util.parse_agent_dat(pair[1])
                [an1,an2] = util.pair_inagentlist(new_agents,pair[0],pair[1])

                if (an1 == 1) & (an2 == 1):
                    new_agents = np.delete(new_agents,[idp,idq],0)
                    # add pair to new cluster
                    cluster_no = cluster_no + 1
                    C_new = Cluster(pair[0],pair[1],cluster_no) # make cluster
                    print(type(self.c_env))
                    self.c_env.append(C_new) # Add to clusterings
                elif (an1 == 0) & (an2 == 0):

                    continue
                else:
                # assume one is true
                    if an1 == 1:
                        # add to existing cluster maybe
                        util.check_agent_cluster_group(pair[0])
                    if an2 == 1:
                        # check and add to existing cluster

                        util.check_agent_cluster_group(pair[1])

        if new_agents.size != 0:
            for agent in new_agents:
                [p1,p2,idp] = util.parse_agent_dat(agent)
                cluster_no = cluster_no + 1
                C_new = Cluster(agent,agent,cluster_no) # make cluster
                self.c_env.append(C_new)
        

    #def make_multi_agent_clusters():


    
            




    
        
            


