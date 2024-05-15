#James 2024
#MM-UKF implmentation
import numpy as np
import numpy.linalg as LA

def MM_UKF(xi,Clusters,modes,ego = None):
    # xi - cls ->Cluster with a particular pair
    # cluster object with cluster list -> cls
    #General Multimodal for either cluster with members/singleton cluster
    for mode in modes:
        MM_UKF.propagate_cluster(xi,Clusters,ego = None)
    #porpogate
    # Combine modes
    MM_UKF.combine_modes(xi,Clusters,ego)
    update_cluster()


def propagate_clusters(xi,Clusters,modes,ego=None):
    for cls in Clusters.cluster_list:
    for 
    a = 1

def update_cluster():
    a = 1
