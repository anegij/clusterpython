# April 2024 -Anegi James
# Controller code/module

import warnings

import numpy as np
import numpy.linalg as LA
import itertools


from abc import ABC, abstractmethod
from Cluster_Prediction import Metrics
from Cluster_Prediction import UTILS as util
from scipy.integrate import solve_ivp

class obs_Controller():
    def __init__(self) -> None:
        self.K = [0.5,0.5]
        self.A = [[0,0,1,0],[0,0,0,1],
                  [0,0,0,0],[0,0,0,0]]
        self.B = [[0,0,1,0],[0,0,0,1]]
        self.Aatt = 20
        self.Batt = 0.08
        self.Arep = 20
        self.Brep = 0.08
        self.udim = 2
        self.u_max = [2,2]
        self.radii = 0.5

    
    def main_func(self,t,y,lam,goal,usf):
        ug = self.PD_control(y,goal)
        dydt = np.zeros(4)
        dydt[0] = y[2]
        dydt[1] = y[3]
        dydt[2] = lam[0]*ug[0] + lam[1]*usf[0]
        dydt[3] = lam[0]*ug[1] + lam[1]*usf[1]
        return dydt

    def PD_control(self,y,goal):
        u = np.zeros(2)
        Kp = self.K[0]
        Kv = self.K[1]
        u[0] = -Kp*y[0] + -Kv*y[2] + goal[0]
        u[1] = -Kp*y[1] + -Kv*y[3] + goal[1]
        return u
    
    def Optimal_control():
        return
    
    def SF_control(self,ego,goal,other_obstacles):
        Frep = np.zeros((self.udim))
        Fatt = np.zeros((self.udim))
        rad1 = self.radii
        rad2 = self.radii
        goal_dist = goal - ego[0:2]
        goal_dir = goal_dist / LA.norm(goal_dist)
        #Goal attraction
        Fatt = -self.Aatt*np.exp((rad1 - LA.norm(goal_dist) )/self.Batt) * goal_dir
        # TODO - Change to only include close obstacles
        for obs in other_obstacles:
            dist = LA.norm(obs[0:2] - ego[0:2])
            n_ij = (obs[0:2] - ego[0:2])/dist
            # repulsive force
            Frep = Frep + self.Arep*np.exp((rad1+rad2-dist)/self.Brep)*n_ij
        
        #TODO add attractive force for within cluster/close by agents - None
        Fsf = Fatt + Frep
        # floor value to max input
        usf = np.sign(Fsf) * np.minimum(np.abs(Fsf),self.u_max)
        return usf
