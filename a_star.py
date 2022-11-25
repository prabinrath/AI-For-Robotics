# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

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
            if self.queue[item] > priority:
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
        

def search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    expand = [[-1 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    ind = 0
    pq = PriorityQueue()
    pq.push(tuple(init),0+heuristic[init[0]][init[1]])
    closed = []
    while not pq.is_empty():
        node,value = pq.pop()
        value -= heuristic[node[0]][node[1]]
        if list(node) == goal:
            expand[goal[0]][goal[1]] = ind
            return expand
        if node not in closed:
            closed.append(node)
            expand[node[0]][node[1]] = ind
            ind = ind + 1
            for ds in delta:
                child = [node[0]+ds[0],node[1]+ds[1]]
                if child[0]>=0 and child[0]<len(grid) and child[1]>=0 and child[1]<len(grid[0]):
                    if grid[child[0]][child[1]] == 0:
                        pq.push(tuple(child),value+cost+heuristic[child[0]][child[1]])
    
    return 'fail'

res = search(grid,init,goal,cost,heuristic)
for r in res:
    print(r)