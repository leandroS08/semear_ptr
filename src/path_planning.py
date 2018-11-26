#!/usr/bin/env python
import rospy
from semear_ptr.srv import FindPath, FindPathRequest, FindPathResponse

from scipy.sparse.csgraph import dijkstra, csgraph_from_dense
import numpy as np

""" Function definition:
    start: the number of the vertice to start from
    end: the number of the vertice to go to
    impossible_paths: 1D matrix that store the paths that should not be taken in to account
            The matrix should be in the form [ X1, Y1, X2, Y2, ..., Xn, Yn], where Xi and Yi are the
            two vertices that should be disconnected
"""
def handle_find_path(req):
    
    start = req.start
    end = req.end
    impossible_paths= []
    
    # Converts the matrix [ X1, Y1, X2, Y2, ..., Xn, Yn] in [ [X1, Y1], [X2, Y2], ... ,[Xn Yn ]]
    for i,j in zip(req.impossible_paths[0::2], req.impossible_paths[1::2] ):
        impossible_paths.append([i, j])
    
    # Matrix of cost
    G2_data = np.ones(shape=(12,12))

    for i in range(12):
        for j in range(12):
            is_upper =  j == i + 3
            is_bottom = j == i - 3
            is_left =   j == i + 1 and not (i%3 == 0)
            is_right =  j == i - 1 and not (i%3 == 2)
            
            if (not is_upper) and (not is_bottom) and (not is_left) and (not is_right) and j>=0 and i>=0 and j<=11 and i<=11:
                
                G2_data[i,j] = np.inf
                G2_data[j,i] = np.inf

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