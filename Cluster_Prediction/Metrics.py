# April 2024 -Anegi James
# code/module

import warnings

import numpy as np
import numpy.linalg as LA


from abc import ABC, abstractmethod

def distance_metric(x,y):
    x = x[0:2]
    y = y[0:2]
    return np.linalg.norm(x-y,2)

def vel_distance(x,y):
    vx = x[2:4]
    vy = y[2:4]
    return np.linalg.norm(vx-vy,2)

def orientation_distance(x,y):
    v1 = x[2:4]
    v2 = y[2:4]
    tan1 = np.empty(1,dtype=float)
    tan1 = np.arctan2(v1[1],v1[0]) - np.arctan2(v2[1],v2[0])
    tan1 = np.absolute(tan1)
    return tan1


def cost_metric(x,Tf,lam):
    lam = [0.5,0.5]
    dist = distance_metric(x[0],x[1])
    

def terminal_vel_cost(xi,vi,vT,xT,Tf):
    C1 = 1/Tf^3*(12*xi - 12*xT + 6*Tf*vi + 6*Tf*vT)
    C2 = -1/Tf*(b*Tf^2/2 + vi - vT)
    J1 = 1/2*(Tf*LA.norm(C1)^2 + Tf^2*LA.transpose(a)*b + 1/3*Tf^3*LA.norm(C2)^2)
    return J1

def terminal_goal_cost(xi,vi,xg,xj,vj):
    Tf = LA.norm(xg - xi,2)/LA.norm(-vi)
    C1 = 6/Tf^2*(xg - xi - Tf*vi)


