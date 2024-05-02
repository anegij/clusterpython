# April 2024 -Anegi James
# code/module

import warnings

import numpy as np
import numpy.linalg as LA


from abc import ABC, abstractmethod

def distance_metric(x,y):
    return np.norm(x,y)

def cost_metric(x,y,Tf,lam,other_agents):
    dist = distance_metric(x,y)
