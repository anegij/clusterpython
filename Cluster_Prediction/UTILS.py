import numpy as np
import numpy.linalg as LN
from Cluster_Prediction import Metrics
from Cluster_Prediction import UTILS as util
from Cluster_Prediction.obs_Controller import obs_Controller
from scipy.integrate import solve_ivp

def parse_agent_dat(a):
    pa = a[0:2]
    va = a[2:4]
    ida = int(a[4])
    return pa,va,ida    

def pair_inagentlist(agent_list, pair1,pair2):
    p1 = pair1[0]
    p2 = pair2[0]
    anum1 = 0
    anum2 = 0
    for agent in agent_list: 
        pos_ag = agent[0]
        anum1 = anum1 + (pos_ag==p1)
        anum2 = anum2 + (pos_ag==p2)
        # return true/false
    return anum1,anum2

def check_agent_cluster_group(mem1):
    a=1

def make_sigma_points(xi,n_sigma,dim,P):
    # Standard choices for sigma point params
    index = 0
    alpha = 1
    beta = 2
    kap = 1
    #X(0) - xi, X(1:ns)+ X(ns+1:2ns
    xm = np.concatenate((xi.mean_pos, xi.mean_vel))
    L = LN.cholesky(P)
    X_sigma = (np.empty((n_sigma+1,dim)))   # needs len no, want 0 + 1..n terms
    X_sigma[index] = xm # n=0
    if (len(xi.cluster_members) > 1):
        for member in xi.cluster_members:
            X_sigma = np.append(X_sigma,member)
        n_sigma = n_sigma + 1 + len(xi.cluster_members)
        X_sigma = np.reshape(X_sigma,(n_sigma,dim))
    else:
        n_sigma = n_sigma +1    # actual no of points = 9 0..2*n
    lam = alpha^2*(n_sigma+kap) - n_sigma

    for ii in range(1,dim):
        xnr = xm + np.sqrt(n_sigma+lam)*L[:,ii-1]
        xnl = xm - np.sqrt(n_sigma+lam)*L[:,ii-1]

        X_sigma[ii] = xnr
        X_sigma[ii+dim] = xnl

    # find weights 
    n = np.round(n_sigma/2)   # weights for X(1:2n)
    Wm = np.zeros((n_sigma,1))
    Wc = np.zeros((n_sigma,1))
    Wm[0] = lam/(n+lam)
    Wc[0] = lam/(n+lam) + (1-alpha**2 +beta)
    Wm[1:n_sigma] = 1/(2*(n+lam))
    Wc[1:n_sigma] = 0.5/(n+lam)

    return X_sigma,Wm,Wc


def dyn_model_SF(ego,delta_time,goal,other_obstacles):
    # input is vector not class object
    Contrl = obs_Controller()
    lam1 = param_goal_tuning(ego,goal)
    lam2 = param_agent_tuning(ego,other_obstacles)
    lam = [lam1,lam2]

    usf = Contrl.SF_control(ego,goal,other_obstacles)
    
    y0 = ego

    sol = solve_ivp(Contrl.main_func,[0,delta_time],y0,args = [lam,goal,usf])

    return sol.y[:,-1]

def param_goal_tuning(ego,goal):
    lam = LN.norm(ego[0:2] - goal[0:2])
    return lam

def param_agent_tuning(ego,other_obstacles):
    count  =0
    pos = np.zeros((len(other_obstacles),2))
    for obs in other_obstacles:
        pos[count] = obs[0:2]
        count = count+1
    mean_pos = np.average(pos,0)
    lam = LN.norm(ego[0:2] - mean_pos)
    return lam