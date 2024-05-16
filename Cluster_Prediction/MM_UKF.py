#James 2024
#MM-UKF implmentation
import numpy as np
import numpy.linalg as LA
from abc import ABC, abstractmethod
from Cluster_Prediction import UTILS as util

class MM_UKF(ABC):
    def __init__(self,delta_time,xi):
        self.dim = 4
        self.H = np.ones((4,4)) #z=Hx
        self.n_mem = len(xi.cluster_members)   # Members in each cluster
        self.n_sig = 2*self.dim
        self.modes = [[1.0, 0.2],[1.0,1.2]]
        self.dt = delta_time
        self.Q = 0.001*np.eye(4,4)


    def MMUKF(self,xi,Clusters,modes,ego = None):
        # xi - cls ->Cluster with a particular pair
        # cluster object with cluster list -> cls
        #General Multimodal for either cluster with members/singleton cluster
        for mode in self.modes:
            self.propagate_clusters(xi,Clusters,mode,ego = None)
        #porpogate
        # Combine modes
        MM_UKF.combine_modes(xi,Clusters,ego)
        update_cluster()


    def propagate_clusters(self,xi,Clusters,mode,ego=None):
        # Make sigma points 
        index = 0
        [X_sigma,Wm,Wc] = util.make_sigma_points(xi,self.n_sig,self.dim,xi.cov)
        new_XS = np.empty_like(X_sigma)
        other_obstacles = []
        for cls in Clusters.cluster_list:
            if any((cls.mean_pos != xi.mean_pos)):
                other_obstacles.append([cls.mean_pos,cls.mean_vel])
        other_obstacles = np.reshape(other_obstacles,(len(other_obstacles),self.dim))      
        for point in X_sigma:
            new_XS[index] = util.dyn_model_SF(point,self.dt,mode,other_obstacles)
        
        xm = Wm*new_XS
        Pm = np.zeros((self.dim,self.dim)) + self.Q
        for ii in range(0,self.n_sig):
            Pm = Pm + Wc[ii]*np.transpose((new_XS[ii] - xm))*(new_XS[ii] - xm)

        # Update cluster? Make new cluster instance?
        xi.cov = Pm
        xi.x_mean_prop = xm
        xi.x_sigs = new_XS
        xi.cluster_members = new_XS[-2:-1,:]
        return xm, Pm, new_XS
            

    def update_cluster():
        a = 1

    