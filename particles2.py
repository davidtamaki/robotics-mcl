import time
import sys
import random
import robotmoves as rm
import brickpi
from math import *
import numpy as np

# constants
numberOfParticles = 100
stdev_e = 0.05
stdev_f = 0.35
stdev_g = 0.25
scale = 10  # cm -> pixel
offset_x = 150
offset_y = 20
width = 1000  # screen size
height = 650

x, y, theta = 0.0, 0.0, 0.0  # initial location
particles = [(x, y, theta, 1.0/numberOfParticles)] * numberOfParticles

# set up motors
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

# functions

def getRandomX():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomY():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomTheta(): 
    return random.randint(0, 360)

def uncertainMove(D):
    global particles
    rm.move_forward(interface,motors,D)  # robot move
    sigma_e = np.sqrt(D) * stdev_e
    sigma_f = np.sqrt(D) * stdev_f
    new_particles = []
    for p in particles:  # update particles
        e = random.gauss(0,sigma_e)
        f = random.gauss(0,sigma_f)
        theta = radians(p[2])
        new_particles.append((p[0]+(D+e)*cos(theta), p[1]+(D+e)*sin(theta), p[2]+f, p[3]))
    particles = new_particles

def uncertainTurn(alpha):  # in degrees
    global particles
    alpha = alpha % 360
    if alpha<=180:
        rm.turn_left(interface,motors,alpha)
    else:
        rm.turn_right(interface,motors,360-alpha)
    sigma_g = np.sqrt(alpha) * stdev_g
    new_particles = []
    for p in particles:
        g = random.gauss(0,sigma_g)
        new_particles.append((p[0], p[1], p[2]+alpha+g,p[3]))
    particles = new_particles

def xToPixel(x):  # convert x in world coordinate to screen position
    return x*scale+offset_x

def yToPixel(y):  # convert y in world coordinate to screen position
    return height-y*scale-offset_y

def particlesToPixels(particles):
    pixels = []
    for p in particles:
        pixels.append((xToPixel(p[0]), yToPixel(p[1])))
    return pixels

def navigateToWaypoint(x,y):
    global particles
    pos = np.average(np.asarray(particles), axis=0)
    line = (xToPixel(pos[0]), yToPixel(pos[1]), xToPixel(x), yToPixel(y))
    print "drawLine:" + str(line)
    dy, dx = y-pos[1], x-pos[0]
    theta = (degrees(atan2(dy, dx)) - pos[2]) % 360 #  angle to turn
    uncertainTurn(theta)
    uncertainMove(sqrt(dx*dx+dy*dy))
    pixels.append(particlesToPixels(particles))
    print "drawParticles:" + str(pixels)

line1 = (xToPixel(0), yToPixel(0), xToPixel(40), yToPixel(0))
line2 = (xToPixel(40), yToPixel(0), xToPixel(40), yToPixel(40))
line3 = (xToPixel(40), yToPixel(40), xToPixel(0), yToPixel(40))
line4 = (xToPixel(0), yToPixel(40), xToPixel(0), yToPixel(0))

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

pixels = particlesToPixels(particles)
print "drawParticles:" + str(pixels)
time.sleep(1)

# test naviate to waypoint
"""
navigateToWaypoint(40,20)
time.sleep(2)
navigateToWaypoint(0,-10)
time.sleep(2)
navigateToWaypoint(0,0)
time.sleep(2)

time.sleep(10)
"""
# draw square
for j in range(4):
    for i in range(4):
        D = 10.0
        uncertainMove(D)
        pixels.append(particlesToPixels(particles))
        print "drawParticles:" + str(pixels)    
    uncertainTurn(90)
    pixels.append(particlesToPixels(particles))
    print "drawParticles:" + str(pixels)
