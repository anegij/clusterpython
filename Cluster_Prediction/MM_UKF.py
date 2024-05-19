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
        [X_sigma,Wm,Wc] = util.make_sigma_points(xi,self.n_sig,self.dim,xi.cov)
        for mode in self.modes:
            [xm, Pm, new_XS,Wm,Wc] = self.propagate_clusters(xi,Clusters,mode,X_sigma,Wm,Wc,ego = None)
            Holder.append([xm,Pm,new_XS])
        #porpogate
        # Combine modes
        [z,z_ind] = self.combine_modes(Holder,xi,Clusters,modes)
        [xk,Pk] = self.update_cluster(z,xi,Wm,Wc,z_ind)


    def propagate_clusters(self,xi,Clusters,mode,X_sigma,Wm,Wc,ego=None):
        # Make sigma points 
        index = 0
        new_XS = np.empty_like(X_sigma)
        other_obstacles = []
        for cls in Clusters.cluster_list:
            if any((cls.mean_pos != xi.mean_pos)):
                other_obstacles.append([cls.mean_pos,cls.mean_vel])
        other_obstacles = np.reshape(other_obstacles,(len(other_obstacles),self.dim))  
        print(X_sigma)
        # propagate    
        for point in X_sigma:
            new_XS[index] = util.dyn_model_SF(point,self.dt,mode,other_obstacles)
            index = index+1
        print(new_XS)
        # Combine sigma
        xm = np.dot(np.transpose(Wm),new_XS)
        Pm = np.zeros((self.dim,self.dim)) + self.Q

        for ii in range(0,self.n_sig):
            one = np.reshape((new_XS[ii] - xm),(4,1))
            two = np.reshape((new_XS[ii] - xm),(1,4))
            Pm = Pm + Wc[ii]*np.matmul(one,two)
        # Update cluster? Make new cluster instance?
        xi.x_mean_prop = xm
   
        return xm, Pm, new_XS, Wm, Wc
            
    def combine_modes(self,Holder,xi,Clusters,modes):
        H = self.H
        c = 0
        R = self.R
        z = xi.gen_observation(self)
        if len(xi) !=1:
            z_ind = z
            z = np.mean(z,0)
        else: z_ind = 0
            
        for mode in range(0,len(modes)):
            ee = np.transpose(Holder[mode][0])
            Pk = Holder[mode][1]
            t1 = z - np.reshape(np.matmul(self.H,ee),4)
            t2 = np.matmul(H,np.matmul(Pk,np.transpose(H))) + self.R
            expo = np.exp(-0.5*np.matmul(np.reshape(t1,(1,4)),np.matmul(LA.inv(t2),np.reshape(t1,(4,1)))))
            pz = 1/((np.sqrt(2*np.pi))**self.dim * LA.det(t2))*expo
            c = c + pz*xi.modals[mode]

        for mode in range(0,len(modes)):
            ee = np.transpose(Holder[mode][0])
            Pk = Holder[mode][1]
            t1 = z - np.reshape(np.matmul(self.H,ee),4)
            t2 = np.matmul(H,np.matmul(Pk,np.transpose(H))) + self.R
            expo = np.exp(-0.5*np.matmul(np.reshape(t1,(1,4)),np.matmul(LA.inv(t2),np.reshape(t1,(4,1)))))
            pz = 1/((np.sqrt(2*np.pi))**self.dim * LA.det(t2))*expo
            xi.modals[mode] = 1/c*pz*xi.modals[mode]

        Cm = 0
        Pm = np.zeros((self.dim,self.dim))
        nmem = len(xi)
        Xsig = np.zeros((nmem,4))
        for mode in range(0,len(modes)):
            ee = Holder[mode][0]
            Pk = Holder[mode][1]
            Cm = Cm + xi.modals[mode]*Holder[mode][0]
            Pm = Pm + xi.modals[mode]*Holder[mode][1]
            if nmem != 1:
                print(Xsig)
                Xsig = Xsig + xi.modals[mode]*Holder[mode][2][-nmem-1:-1]  
                print(Xsig)
        xi.cov = Pm
        xi.x_mean_prop = Cm
        if nmem != 1: 
            Holder[mode][2][-nmem-1:-1] = Xsig
            xi.x_sigs[-nmem-1:-1] = Xsig
            xi.cluster_members = Xsig

        return z,z_ind

    def update_cluster(self,z,xi,Wm,Wc,z_ind):
        ns = self.dim # no of states
        Pzz = np.zeros((ns,ns)) + self.R
        Pxz = np.zeros((ns,ns))
        num_Xsigs = len(xi.x_sigs)
        Zk = np.zeros((num_Xsigs,ns))
        for ii in range(0,num_Xsigs): # Check if n_sig or nosig+1
            Xm = xi.x_sigs[ii]
            Zk[ii,:] = np.reshape(np.matmul(self.H,Xm),ns) #+ np.random.multivariate_normal(np.zeros(ns),self.R)
        zm = np.reshape(np.dot(np.transpose(Wm),Zk),4)
        xm = np.reshape(xi.x_mean_prop,4)
        for ii in range(0,num_Xsigs):
            Xm = xi.x_sigs[ii]
            one = np.reshape(Zk[ii] - zm,(4,1))
            two = np.reshape((Zk[ii] - zm),(1,4))
            three = np.reshape(Xm - xm,(4,1))
            Pzz = Pzz + Wc[ii]*np.matmul(one,two)
            Pxz = Pxz + Wc[ii]*np.matmul(three,two)
        Kk = np.matmul(Pxz,LA.inv(Pzz))

        xk = xm + np.matmul(Kk,(z-zm))
        # Update sigma points
        if len(xi)!= 1:
            xi.x_sigs = xi.x_sigs[-len(xi)-1:-1,:] + np.reshape(np.matmul(Kk,np.transpose(z_ind - [zm,zm])),(len(xi),self.dim))
        # TODO check for any errors det(Pk) is ~0
        Pk = xi.cov - np.matmul(Kk,np.matmul(Pzz,np.transpose(Kk)))

        xi.mean_pos = xk[0:2]
        xi.mean_vel = xk[2:4]
        xi.cov = Pk
        
        return xk,Pk


    