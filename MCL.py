#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import brickpi
import numpy as np
import robotmoves as rm
from math import *

K = 0.05
sigma_s = 3
beta_limit = 35
beta_threshold = 0.5
numberOfParticles = 200
stdev_e = 0.25
stdev_f = 0.25
stdev_g = 0.1
sreadings = 10
length = 20
sonar_range = 190
sonar_adj = -2.0


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
port = 1 # need to change
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

# Functions
def uncertainMove(D):
    global particles
    rm.move_forward(interface,motors,D)  # robot move
    sigma_e = np.sqrt(D) * stdev_e
    sigma_f = np.sqrt(D) * stdev_f
    new_particles = []
    for p in particles.data:  # update particles
        e = random.gauss(0,sigma_e)
        f = random.gauss(0,sigma_f)
        theta = radians(p[2])
        new_particles.append((p[0]+(D+e)*cos(theta), p[1]+(D+e)*sin(theta), p[2]+f, p[3]))
    particles.data = new_particles
    sonar_reading = sonar_measurement()
    particles.update(sonar_reading)
    particles.draw()

def uncertainTurn(alpha):  # in degrees
    global particles
    alpha = alpha % 360
    if alpha<=180:
        rm.turn_left(interface,motors,alpha)
    else:
        rm.turn_right(interface,motors,360-alpha)
    sigma_g = np.sqrt(alpha) * stdev_g
    new_particles = []
    for p in particles.data:
        g = random.gauss(0,sigma_g)
        new_particles.append((p[0], p[1], p[2]+alpha+g,p[3]))
    particles.data = new_particles
    sonar_reading = sonar_measurement()
    particles.update(sonar_reading)
    particles.draw()
    
    
def navigateToWaypoint(x,y):
    global particles
    pos = np.average(np.asarray(particles.data), axis=0)
    print pos
    dy, dx = y-pos[1], x-pos[0]
    theta = (degrees(atan2(dy, dx)) - pos[2]) % 360 #  angle to turn
    uncertainTurn(theta)
    pos = np.average(np.asarray(particles.data), axis=0)
    dy, dx = y-pos[1], x-pos[0]
    D = sqrt(dx*dx+dy*dy)
    if D<length:
        line = (pos[0], pos[1], x, y)
        canvas.drawLine(line)
        uncertainMove(D)
    else:
        D1 = length
        line = (pos[0], pos[1], pos[0]+D1*cos(radians(pos[2])), pos[1]+D1*sin(radians(pos[2])))
        canvas.drawLine(line)
        uncertainMove(D1)
        navigateToWaypoint(x,y)

def sonar_measurement():
    reading = []
    for i in range(sreadings):
        usReading = interface.getSensorValue(port)
        if usReading:
            reading.append(usReading[0])
        time.sleep(0.05)
    print reading
    return sorted(reading)[len(reading)/2] + sonar_adj

def bsearch(m, x, a, b):
    if a>=b:
        return a
    k = a + (b-a)/2
    if x<=m[k]:
        return bsearch(m,x,a,k)
    else:
        return bsearch(m,x,k+1,b)

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        # display.extend([(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data])
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

# Simple Particles set
class Particles:
    def __init__(self,x,y,theta,n):
        self.n = n;
        self.data = [(x, y, theta, 1.0/self.n)] * self.n;

    def update(self,z):
        if z>sonar_range:
            print "skipping particle update"
            print z
            return
        calc = [calculate_likelihood(p[0],p[1],p[2],z) for p in self.data];
        likelihood = np.array([x[0] for x in calc])
        beta = [x[1] for x in calc]
        oblique_angle = float(len([b for b in beta if b>beta_limit]))/self.n
        if oblique_angle>beta_threshold:
            print "skipping particle update"
            print beta, oblique_angle
            return
        # normalisation
        weight = np.array([x[3] for x in self.data]) * likelihood
        sum_weight = weight.sum()
        weight /= sum_weight
        # resampling
        cweight = np.cumsum(weight)
        new_data = []
        for i in range(self.n):
            r = random.random()
            k = bsearch(cweight,r,0,self.n-1)
            # print cweight, r, k
            new_data.append(self.data[k])
            
        self.data = new_data
        #print self.data
        """
        for x in new_data:
            r = random.random()
            k = bsearch(cweight,r,0,self.n-1)
            print cweight, r, k
            new_data.append(self.data[k])
            #x = self.data[k]
            
        self.data = new_data

        new_data = []
        for i in range(self.n):
            r = random.random()
            k = bsearch(cweight,r,0,self.n-1)
            print cweight, r, k
            new_data.append(self.data[k])

            for j in range(self.n):
                if r<=cweight[j]:
                    # print j
                    new_data.append(self.data[j])
                    break
        """
        #print likelihood
        #print beta, oblique_angle
    
    def draw(self):
        canvas.drawParticles(self.data);
        
# calculate likelihood of a particle
def calculate_likelihood(x,y,theta,z):
    close = 10000.0
    beta = 0.0
    rt = radians(theta)
    first_wall = []
    
    for wall in mymap.walls:
        #print wall
        Ax, Ay, Bx, By = wall[0], wall[1], wall[2], wall[3]
        denom = (By-Ay)*cos(rt) - (Bx-Ax)*sin(rt)
        if denom==0:
            continue;
        m = ((By-Ay)*(Ax-x)-(Bx-Ax)*(Ay-y)) / denom
        #print m
        if m<0 or m>close:
            continue
        x_i = x + m*cos(rt)
        y_i = y + m*sin(rt)
        if (Ay == By) and (x_i >= min(Ax,Bx)) and (x_i <= max(Ax,Bx)):
            # print m
            close = m
            first_wall = wall
        elif (Ax == Bx) and (y_i >= min(Ay,By)) and (y_i <= max(Ay,By)):
            close = m
            first_wall = wall    
    # print close, first_wall
    if first_wall == []:
        return 0, 0
    Ax, Ay, Bx, By = first_wall[0], first_wall[1], first_wall[2], first_wall[3]
    x_i = x + close*cos(rt)
    y_i = y + close*sin(rt)
    # print x_i, y_i
    beta = degrees(acos((cos(rt)*(Ay-By) + sin(rt)*(Bx-Ax))/sqrt((Ay-By)**2+(Bx-Ax)**2)))
    return exp(-(z-close)**2/2.0/sigma_s**2) + K, beta


canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();

particles = Particles(84,30,0,numberOfParticles)
t = 0

path = [(180,30),(180,54),(126,54),(126,168),(126,126),(30,54),(84,54),(84,30)]

for point in path:
    print point
    navigateToWaypoint(point[0],point[1])
    time.sleep(1)

"""
for i in range(5):
    uncertainMove(10)
    sonar_reading = interface.getSensorValue(port)
    particles.update(sonar_reading[0])
    particles.draw()
    t += 2
    time.sleep(2)
"""