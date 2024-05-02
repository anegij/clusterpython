# Realized I am making a new instance of clustering everytime so this will
# hold cluster objects at each time
# Include timestep, cluster member positions, cluster centers, indices representing cluster memerbship 
# for each agent 

from abc import ABC, abstractmethod

import numpy as np
import warnings

class Cluster_storage(ABC):
    def __init__(self,cltr_list = None):
        self.cluster_list = []
        

        if cltr_list is not None:
            # If we have clusters add them in
            for cls in self.cluster_list:
                self.append(cls)

        if isinstance(cltr_list, (list, Cluster_storage)):
            self.cluster_list = cltr_list


    def __getitem__(self, key):
        return self._obstacle_list[key]

    def __setitem__(self, key, value):
        self._obstacle_list[key] = value

    def append(self, value):  
        self._obstacle_list.append(value)