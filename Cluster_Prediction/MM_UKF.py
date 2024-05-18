#James 2024
#MM-UKF implmentation
import numpy as np
import numpy.linalg as LA
from abc import ABC, abstractmethod
from Cluster_Prediction import UTILS as util
from Cluster_Prediction.obs_Controller import obs_Controller

class MM_UKF(ABC):
    def __init__(self,delta_time,xi):
        self.dim = 4
        self.H = np.eye(self.dim) #z=Hx
        self.R = np.diag([0.1,0.1,0.01,0.01])
        self.n_mem = len(xi.cluster_members)   # Members in each cluster
        self.n_sig = 2*self.dim
        self.modes = [[1.0, 0.2],[1.0,1.2]]
        self.dt = delta_time
        self.Q = 0.001*np.eye(4,4)


    def MMUKF(self,xi,Clusters,modes,ego = None):
        # xi - cls ->Cluster with a particular pair
        # cluster object with cluster list -> cls
        #General Multimodal for either cluster with members/singleton cluster
        Holder = []
        for mode in self.modes:
            [xm, Pm, new_XS, Wm, Wc] = self.propagate_clusters(xi,Clusters,mode,ego = None)
            Holder.append([xm,Pm,new_XS])
        #porpogate
        # Combine modes
        z = self.combine_modes(Holder,xi,Clusters,modes)
        self.update_cluster(z,xi,Wm,Wc)


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
        # propagate    
        for point in X_sigma:
            new_XS[index] = util.dyn_model_SF(point,self.dt,mode,other_obstacles)
            index = index+1
        
        # Combine sigma
        xm = np.dot(np.transpose(Wm),new_XS)
        Pm = np.zeros((self.dim,self.dim)) + self.Q
        for ii in range(0,self.n_sig):
            Pm = Pm + Wc[ii]*np.matmul(np.transpose((new_XS[ii] - xm)),(new_XS[ii] - xm))

        # Update cluster? Make new cluster instance?
   
        xi.cov = Pm
        xi.x_mean_prop = xm
        xi.x_sigs = new_XS
        xi.cluster_members = new_XS[-2:-1,:]
        return xm, Pm, new_XS, Wm, Wc
            
    def combine_modes(self,Holder,xi,Clusters,modes):
        H = self.H
        c = 0
        R = self.R
        z = xi.gen_observation(self)
        for mode in range(0,len(modes)):
            ee = Holder[mode][0]
            Pk = Holder[mode][1]
            t1 = z - np.matmul(self.H,ee)
            t2 = np.matmul(H,np.matmul(Pk,np.transpose(H))) + self.R
            expo = np.exp(-0.5*np.matmul(np.transpose(t1),np.matmul(LA.inv(t2),t1)))
            pz = 1/(np.sqrt(2*np.pi))**self.dim * LA.det(t2)*expo
            c = c + pz*xi.modals[mode]

        for mode in range(0,len(modes)):
            ee = Holder[mode][0]
            Pk = Holder[mode][1]
            t1 = z - np.matmul(self.H,ee)
            t2 = np.matmul(H,np.matmul(Pk,np.transpose(H))) + self.R
            expo = np.exp(-0.5*np.matmul(np.transpose(t1),np.matmul(LA.inv(t2),t1)))
            pz = 1/(np.sqrt(2*np.pi))**self.dim * LA.det(t2)*expo
            xi.modals[mode] = 1/c*pz*xi.modals[mode]

        Cm = 0
        Pm = 0
        nmem = len(xi.mem_idx)
        for mode in range(0,len(modes)):
            ee = Holder[mode][0]
            Pk = Holder[mode][1]
            Cm = Cm + xi.modals[mode]*Holder[mode][0]
            Pm = Pm + xi.modals[mode]*Holder[mode][1]
            if nmem != 1:
                Xsig = Xsig + xi.modals[mode]*Holder[mode][2][-len(xi.mem_idx):-1]  
            

        xi.cov = Pk
        xi.x_mean_prop = Cm
        if nmem != 1: 
            Holder[mode][2][-len(xi.mem_idx):-1] = Xsig
            xi.x_sigs[-nmem:-1] = Xsig
            
        return z

   def update_cluster(self,z,xi,Wm,Wc):
    ns = self.dim # no of states
    Pzz = np.zeros(ns,ns) + self.R
    Pxz = np.zeros(ns,ns)
    Zk = np.zeros(xi.n_sig+1,ns)
    for ii in range(0,xi.n_sig): # Check if n_sig or nosig+1
        Xm = xi.x_sigs[ii]
        Zk[ii,:] = np.matmul(self.H,Xm) + np.random.normal(np.zeros(ns,1),self.R)
    zm = np.matmul(Zk,Wm)
    xm = xi.x_mean_prop
    for ii in range(0,xi.n_sig):
        Xm = xi.x_sigs[ii]
        Pzz = Pzz + matmul(Wc[ii],np.matmul((Zk[ii] - zm),np.transpose(Zk[ii] - zm))
        Pxz = Pxz + matmul(Wc[ii],np.matmul((Xm[ii] - xm),np.transpose(Zk[ii] - zm))
    z_mean = np.mean(zm)
    Kk = np.matmul(Pxz,np.inv(Pzz))
    xk = xm + matmul(Kk,(z_mean-z);
    Pk = P_m - matmul(Kk,np.matmul(Pzz,np.tranpose(Kk)));
      
    return xk,Pk

    
