#!/usr/bin/env python
import rospy
from semear_ptr.srv import *

from scipy.sparse.csgraph import dijkstra, csgraph_from_dense
import numpy as np

def handle_find_path(req):
    
    start = req.start
    end = req.end
    impossible_paths= []
    for i,j in zip(req.impossible_paths[0::2], req.impossible_paths[1::2] ):
        impossible_paths.append([i, j])
    
    G2_data = np.array(
    [ 
        [np.inf , 1     , 1], 
        [1      , np.inf, 1], 
        [1      , 1     , np.inf]
    ])

    for (i,j) in impossible_paths:
        G2_data[i, j] = np.inf
        G2_data[j, i] = np.inf

    G2_sparse = csgraph_from_dense(G2_data, null_value=np.inf)

    dist, pred = dijkstra(G2_data, indices=[start], return_predecessors=True )

    ended = False
    path = [end]

    while not ended:
        path.append(pred[0, end])
        if path[-1] == start:
            ended = True
        elif path[-1] == -9999 :
            ended = True 

        else:
            end = path[-1]

    path.pop()
    path.reverse()
    print(path)
    return FindPathResponse(path)

def find_path_server():
    rospy.init_node('find_path_server')
    s = rospy.Service('find_path', FindPath, handle_find_path)
    print("Ready to Find Path.")
    rospy.spin()


if __name__ == "__main__":
    find_path_server()