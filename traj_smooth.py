# -----------
# User Instructions
#
# Define a function smooth that takes a path as its input
# (with optional parameters for weight_data, weight_smooth,
# and tolerance) and returns a smooth path. The first and 
# last points should remain unchanged.
#
# Smoothing should be implemented by iteratively updating
# each entry in newpath until some desired level of accuracy
# is reached. The update should be done according to the
# gradient descent equations given in the instructor's note
# below (the equations given in the video are not quite 
# correct).
# -----------

from copy import deepcopy

# thank you to EnTerr for posting this on our discussion forum
def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print ('['+ ', '.join('%.3f'%x for x in old) + \
               '] -> ['+ ', '.join('%.3f'%x for x in new) +']')

# Don't modify path inside your function.
path = [[0, 0],
        [0, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [3, 2],
        [4, 2],
        [4, 3],
        [4, 4]]

# path = [[1,1],[1,2],[1,3],[1,4],[1,5]]

def norm(prevpath,newpath):
    dis = 0
    for i in range(len(path)):
        for j in range(len(path[0])):
            dis += abs(newpath[i][j]-prevpath[i][j])**2
    return dis

def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):

    # Make a deep copy of path into newpath
    newpath = deepcopy(path)

    #######################
    ### ENTER CODE HERE ###
    #######################
    prevpath = [[0 for _ in range(len(path[0]))] for _ in range(len(path))]
    distance = norm(prevpath, newpath)
    while distance > tolerance:
        prevpath = deepcopy(newpath)
        for i in range(1,len(path)-1):
            for j in range(len(path[0])):
                newpath[i][j] += weight_data*(path[i][j]-newpath[i][j]) + weight_smooth*(newpath[i+1][j]+newpath[i-1][j]-2*newpath[i][j])
        distance = norm(prevpath, newpath)

    return newpath # Leave this line for the grader!

printpaths(path,smooth(path,0.5,0.1))