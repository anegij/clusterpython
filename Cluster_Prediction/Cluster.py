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
from Cluster_Prediction.obs_Controller import obs_Controller
from Cluster_Prediction import UTILS as util
from scipy.integrate import solve_ivp

class Cluster(ABC):
    def __init__(self,mem1,mem2,cluster_idx,radii,g,cov):
        [p1,p2,idp] = util.parse_agent_dat(mem1)
        [q1,q2,idq] = util.parse_agent_dat(mem2)
        self.x_mean_prop = None
        self.x_sigs = None
        self.radii = radii
        self.modals = [0.5,0.5]
        self.goal = g[idp]
        if all(mem1 == mem2):
            self.mean_pos = p1
            self.mean_vel = p2
            self.cluster_no = cluster_idx
            self.cluster_members = mem1
            self.mem_idx = idp
            self.cov = cov[idp]
            return
            #TODO figure out for n>2
        self.cov = 0.5*(cov[idp]+cov[idq])
        self.mean_pos = 0.5*(p1+q1)
        self.mean_vel = 0.5*(p2+q2)
        self.cluster_no = cluster_idx
        self.cluster_members = [np.concatenate((p1,p2)), np.concatenate((q1,q2))]
        self.mem_idx = [idp,idq]

    def __len__(self):
        if len(self.cluster_members) == 5: 
            return 1
          
        return len(self.mem_idx)
    
    def linear_dyn_forward(self):
        self.x_mean_prop = [self.mean_pos,self.mean_vel]
    

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
        # forward control towards goal
        Controller = obs_Controller()
        if len(self) == 1:
            x = np.reshape([self.mean_pos,self.mean_vel],filter.dim)
            goal = self.goal
            ug = np.reshape(Controller.PD_control(x,goal),2)
            lam = util.param_goal_tuning(x,goal)
            def xdot(t,y): return np.matmul(Controller.A,y) + np.matmul(Controller.B,ug)*lam
            sol = solve_ivp(xdot,[0,filter.dt],x)
            xk1 = sol.y[:,-1]
        else:
            x = self.cluster_members
            xk1 = np.zeros((len(self), filter.dim))
            for n,ind in enumerate(x):
                x = np.reshape(ind,filter.dim)
                goal = self.goal
                ug = np.reshape(Controller.PD_control(x,goal),2)
                lam = util.param_goal_tuning(x,goal)
                def xdot(t,y): return np.matmul(Controller.A,y) + np.matmul(Controller.B,ug)*lam
                sol = solve_ivp(xdot,[0,filter.dt],x)
                xk1[n] = sol.y[:,-1]
            return xk1

        z = np.reshape(np.matmul(filter.H,xk1),4) #+ np.random.multivariate_normal(np.zeros(4),filter.R)
        # OR TODO make linear path with constant vel assumption + add noise
        return z
