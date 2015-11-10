import time
import sys
import random
import brickpi
import robotmoves as rm
from math import *

c = 0;
def getRandomX():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomY():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomTheta(): 
    return random.randint(0, 360)


numberOfParticles = 100
stdev = 0.01

# line1 = (0, 0, 40, 0) # (x0, y0, x1, y1)
# line2 = (20, 20, 500, 200)  # (x0, y0, x1, y1)

# print "drawLine:" + str(line1)
# print "drawLine:" + str(line2)

x, y, theta = 0.0, 0.0, 0.0


interface = brickpi.Interface()
interface.initialize()
motors = [0,1]
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
motorParams1 = interface.MotorAngleControllerParameters()
motorParams2 = interface.MotorAngleControllerParameters()
motorParams1, motorParams2 = rm.initialize_motors(motorParams1, motorParams2)

interface.setMotorAngleControllerParameters(motors[0],motorParams1)
interface.setMotorAngleControllerParameters(motors[1],motorParams2)

total_dist = 0.0
# rm.move_forward(40)
total_dist += 200
sigma = total_dist * stdev
x += 200 * cos(theta)
y += 200 * sin(theta)

particles = [(x+random.gauss(0,sigma) , y+random.gauss(0,sigma), theta) for i in range(numberOfParticles)]
print "drawParticles:" + str(particles)
time.sleep(10)
