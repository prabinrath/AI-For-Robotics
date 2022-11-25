# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

class PriorityQueue():
    def __init__(self):
        self.queue = {}
    
    def push(self, item, priority):
        if item in self.queue:
            if self.queue[item][0] > priority[0]:
                self.queue[item] = priority
        else:
            self.queue[item] = priority
    
    def pop(self):
        best = min(self.queue, key=self.queue.get)
        value = self.queue[best]
        del self.queue[best]
        return best, value
    
    def is_empty(self):
        return len(self.queue)==0
        

def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    path = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    ind = 0
    pq = PriorityQueue()
    pq.push(tuple(init),(0,[]))
    closed = []
    while not pq.is_empty():
        node,value = pq.pop()
        if list(node) == goal:
            for track in value[1]:
                path[track[0][0]][track[0][1]] = track[1]
            path[goal[0]][goal[1]] = '*'
        if node not in closed:
            closed.append(node)
            for i in range(len(delta)):
                ds = delta[i]
                child = [node[0]+ds[0],node[1]+ds[1]]
                if child[0]>=0 and child[0]<len(grid) and child[1]>=0 and child[1]<len(grid[0]):
                    if grid[child[0]][child[1]] == 0:
                        pq.push(tuple(child),(value[0]+cost,value[1]+[(node,delta_name[i])]))
    
    return path

res = search(grid,init,goal,cost)
for r in res:
    print(r)