# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D(grid,init,goal,cost):
    value_grid = [[[999 for _ in range(4)] for _ in range(len(grid[0]))] for _ in range(len(grid))]
    policy_3d = [[[' ' for _ in range(4)] for _ in range(len(grid[0]))] for _ in range(len(grid))]
    value_grid[goal[0]][goal[1]] = [0 for _ in range(4)]
    policy_3d[goal[0]][goal[1]] = ['*'for _ in range(4)]
    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for ori in range(4):
                    if grid[x][y] == 0:
                        for act in action:
                            new_ori = (ori+act)%4
                            new_x = x + forward[new_ori][0]
                            new_y = y + forward[new_ori][1]
                            if new_x>=0 and new_x<len(grid) and new_y>=0 and new_y<len(grid[0]) and grid[new_x][new_y]==0:
                                new_value = value_grid[new_x][new_y][new_ori] + cost[act+1]
                                if new_value < value_grid[x][y][ori]:
                                    value_grid[x][y][ori] = new_value
                                    policy_3d[x][y][ori] = action_name[act+1]
                                    change = True
                                    
    policy_2d = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    policy_2d[goal[0]][goal[1]] = '*'
    x = init[0]
    y = init[1]
    ori = init[2]
    action_map = {action_name[i]:action[i] for i in range(len(action))}
    while policy_3d[x][y][ori] != '*':
        policy_2d[x][y] = policy_3d[x][y][ori]
        act = action_map[policy_2d[x][y]]
        ori = (ori+act)%4
        x = x + forward[ori][0]
        y = y + forward[ori][1] 
    return policy_2d

res = optimum_policy2D(grid,init,goal,cost)
for r in res:
    print(r)