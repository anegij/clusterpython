# April 2024 -Anegi James
# Clustering code/module

import warnings

import numpy as np
import numpy.linalg as LA
import itertools


from abc import ABC, abstractmethod
from Cluster_Prediction import Metrics

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
        self.index = set()
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


    def calculate_cost(self,set1,goals,Tf):
        # input - positions and velocities along with goals for all obstacles in order
        # output- find pairs, evaluate cost between all pairs, identify low cost pairs
        
        a = list(itertools.combinations(set1,2))
        set1
        for pair in a:
            counter = 0
            Cost = Metrics.cost_metric(pair,Tf,goals)

    
 
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

            # if any true- > cluster tht pair -> define cluster params-> remove from b, replace with cluster mean
        # Check other tolerances
            self.make_two_agent_cluster(agent_list,b,a)
            # Maybe update b to remove agents and add clusters
            # Reuse function to make cluster-cluster groupings 
            a = 1




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
        arr = np.ndarray(shape = (3,1), dtype=float)
        arr = [i for i in range(0,3)]
        np.reshape(arr)
        print(np.shape(new_agents))
        print(np.shape(arr))
        a =  np.concatenate((new_agents,arr),axis = 1)
        print(a)
        for tols,pair in zip(tol_list, pair_list):
            pair = np.array(pair)
            if new_agents.size == 0:
                return agents
            if tols == 1:
                # if intersection gives both pair mem then cluster
                # if intersection gives one, then check to add to cluster
                anum = np.intersect1d(new_agents,pair)
                #[i for i in new_agents if i not in pair]

                if len(anum) == 8:
                    Cluster = pair
                    print(len(pair))
                    n_na = len(new_agents)
                    #new_agents = [np.setdiff1d(new_agents,i) for i in pair for j in new_agents if (i==j).all()]
                    for i in range(0,len(pair)):
                        for j in range(0,len(new_agents)):
                            if (pair[i,:]==new_agents[j,:]).all():
                                mask = np.ones_like(new_agents,dtype=bool)
                                mask[j,:] = False
                                result = new_agents[mask]
                                result = np.reshape(result,(-1,self.dim))
                                                        
                elif len(anum) == 4:
                    # find other agent
                    np.setdiff1d(pair, new_agents, assume_unique=False)

                    check_agent_cluster_group()
                    # if intersection gives one, then check to add to cluster
                elif len(anum) == 0:
                    # skip to next pair
                    continue




    #def make_multi_agent_clusters():


    
            




    
        
            


