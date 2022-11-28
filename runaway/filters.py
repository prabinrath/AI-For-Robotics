# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos_naive(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    if not OTHER:
        OTHER = (measurement,0)
    
    last_measurement, last_theta = OTHER
    dy = measurement[1]-last_measurement[1]
    dx = measurement[0]-last_measurement[0]
    theta = atan2(dy,dx)
    dtheta = theta - last_theta
    vec = sqrt(dx**2+dy**2)
    xy_estimate = (measurement[0]+vec*cos(theta+dtheta),measurement[1]+vec*sin(theta+dtheta))
    OTHER = (measurement, theta)

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER 

def estimate_next_pos_kf_simple(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    dt = 1
    F =  matrix([[1,0,0,dt,0,0],[0,1,0,0,dt,0],[0,0,1,0,0,dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])    
    H =  matrix([[1,0,0,0,0,0],[0,1,0,0,0,0]]) # measurement function: reflect the fact that we observe x and y but not others
    R =  matrix([[0.1,0],[0,0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    I =  matrix([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,0,0,0,1]]) # 6d identity matrix
    u = matrix([[0.], [0.], [0.], [0.], [0.], [0.]]) # external motion

    if not OTHER:
        x = matrix([[measurement[0]], [measurement[1]], [0.], [0.], [0.], [0.]])
        P =  matrix([[0.1,0,0,0,0,0],[0,0.1,0,0,0,0],[0,0,1000,0,0,0],[0,0,0,1000,0,0],[0,0,0,0,1000,0],[0,0,0,0,0,1000]]) # initial uncertainty: for positions x and y, 1000 for others
        OTHER = (x,P)

    x, P = OTHER  
    
    # measurement update
    Z = matrix([measurement])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    x.show()
    xy_estimate = (x.value[0][0], x.value[1][0])
    OTHER = (x, P)
    return xy_estimate, OTHER

def estimate_next_pos_kf_polar(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    
    dt = 1
    F =  matrix([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])    
    H =  matrix([[1,0,0,0],[0,1,0,0]]) # measurement function: reflect the fact that we observe x and y but not others
    R =  matrix([[0.05,0],[0,0.05]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    I =  matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) # 6d identity matrix
    u = matrix([[0.], [0.], [0.], [0.]]) # external motion

    if not OTHER:   
        x = matrix([[0.], [0.], [0.], [0.]]) 
        P =  matrix([[1000,0,0,0],[0,1000,0,0],[0,0,1000,0],[0,0,0,1000]])
        xy_estimate = (measurement[0], measurement[1])
        OTHER = (x,P,measurement)

    x, P, last_measurement = OTHER
    dy = measurement[1]-last_measurement[1]
    dx = measurement[0]-last_measurement[0]
    dd = sqrt(dx**2+dy**2)
    th = atan2(dy,dx) # this approach does not work due to non linearity introduced at this step

    # measurement update
    Z = matrix([[dd,th]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P    

    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()       

    x.show()
    vec = x.value[0][0]
    theta = (x.value[1][0] + x.value[3][0])%(2*pi)
    xy_estimate = (measurement[0]+vec*cos(theta),measurement[1]+vec*sin(theta))
    OTHER = (x,P,measurement)

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER

def estimate_next_pos_ekf(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    
    # Assuming dt = 1
    H =  matrix([[1,0,0,0,0],[0,1,0,0,0]]) # measurement function: reflect the fact that we observe x and y but not others
    R =  matrix([[0.1,0],[0,0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    I =  matrix([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1]]) # 6d identity matrix
    u = matrix([[0], [0], [0], [0], [0]]) # external motion

    if not OTHER:   
        x = matrix([[0], [0], [0], [0], [0]])    
        P =  matrix([[0.1,0,0,0,0],[0,0.1,0,0,0],[0,0,1000,0,0],[0,0,0,1000,0], [0,0,0,0,1000]])
        OTHER = (x,P)

    x, P = OTHER   

    # measurement update
    Z = matrix([measurement])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    # prediction
    xc, yc, th, dst, rot = (x.value[0][0], x.value[1][0], x.value[2][0], x.value[3][0], x.value[4][0])
    x = matrix([[xc + dst * cos(th + rot)],
                [yc + dst * sin(th + rot)],
                [th + rot],
                [dst],
                [rot]])
    J = matrix([[1,0,-dst*sin(th+rot), cos(th+rot), -dst*sin(th+rot)],
                [0,1,dst*cos(th+rot), sin(th+rot), dst*cos(th+rot)],
                [0,0,1,0,1],
                [0,0,0,1,0],
                [0,0,0,0,1]])
    P = J * P * J.transpose()  

    x.show()
    xy_estimate = (x.value[0][0],x.value[1][0])
    OTHER = (x,P)

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
# def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
#     localized = False
#     distance_tolerance = 0.01 * target_bot.distance
#     ctr = 0
#     # if you haven't localized the target bot, make a guess about the next
#     # position, then we move the bot and compare your guess to the true
#     # next position. When you are close enough, we stop checking.
#     while not localized and ctr <= 1000:
#         ctr += 1
#         measurement = target_bot.sense()
#         position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
#         target_bot.move_in_circle()
#         true_position = (target_bot.x, target_bot.y)
#         print 'M:', measurement, ' | G:', true_position, ' | P:', position_guess
#         error = distance_between(position_guess, true_position)
#         if error <= distance_tolerance:
#             print "You got it right! It took you ", ctr, " steps to localize."
#             localized = True
#         if ctr == 1000:
#             print "Sorry, it took you too many steps to localize the target."
#     return localized

def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        # print 'M:', measurement, ' | G:', true_position, ' | P:', position_guess
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos_ekf, test_target)
