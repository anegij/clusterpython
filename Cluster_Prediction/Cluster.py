# April 2024 -Anegi James
# Clustering code/module

import warnings

import numpy as np
import numpy.linalg as LA
import itertools


from abc import ABC, abstractmethod
from Cluster_Prediction import Metrics
from Cluster_Prediction import UTILS as util
from Cluster_Prediction.MM_UKF import MM_UKF

class Cluster(ABC):
    def __init__(self,mem1,mem2,cluster_idx,radii):
        [p1,p2,idp] = util.parse_agent_dat(mem1)
        [q1,q2,idq] = util.parse_agent_dat(mem2)
        self.cov = 0.001*np.eye(4,4)
        self.x_mean_prop = None
        self.x_sigs = None
        self.radii = radii
        self.modals = [0.5,0.5]
        if all(mem1 == mem2):
            self.mean_pos = p1
            self.mean_vel = p2
            self.cluster_no = cluster_idx
            self.cluster_members = mem1
            self.mem_idx = idp
            return

        self.mean_pos = 0.5*(p1+q1)
        self.mean_vel = 0.5*(p2+q2)
        self.cluster_no = cluster_idx
        self.cluster_members = [np.concatenate((p1,p2)), np.concatenate((q1,q2))]
        self.mem_idx = [idp,idq]
    

    def find_center(self,mem1,mem2,cluster_idx,cluster_no):
        # More than a pair of agents
        [p1,p2,idp] = util.parse_agent_dat(mem1)
        [q1,q2,idq] = util.parse_agent_dat(mem2)
        self.mean_pos = 1/cluster_no*(p1+p2)
        self.mean_vel = 1/cluster_no*(q1+q2)

    def do_filtering(self,delta_time,Cluster_environment):
        # Have cluster no = i and its members, as well as entire environ with 
        # other clusters under cluster list
        modes = [0,1]
        filter = MM_UKF(delta_time,self) # create filter class, holds variables
        filter.MMUKF(self,Cluster_environment,modes)

    def gen_observation(self,filter):
        z = np.matmul(filter.H,self.x_mean_prop) + np.random.normal(np.zeros(4),filter.R)
        # OR TODO make linear path with constant vel assumption + add noise
        return 
