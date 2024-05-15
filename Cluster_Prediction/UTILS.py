import numpy as np


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

    