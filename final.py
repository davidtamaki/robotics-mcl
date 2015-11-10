#!/usr/bin/env python

import random
import brickpi
import time
import robotmoves as rm
import numpy as np
from math import *

direction = 1  # 1 for anti-clockwise turn, -1 for clockwise turn
rspeed = pi / 3.5
K = 0.05
sigma_s = 3
numberOfParticles = 200
stdev_e = 0.25
stdev_f = 0.25
stdev_g = 0.1
length = 50
sonar_range = 190
beta_limit = 35
beta_threshold = 0.5
wf_speed = 12
half_t = 8.1
whole_t = 15.3

interface = brickpi.Interface()
rm.initialize_everything(interface)

class Particles:
    def __init__(self,x,y,theta,n):
        self.n = n;
        self.data = [(x, y, theta, 1.0/self.n)] * self.n;

    def update(self,z):
        if z>sonar_range:
            return
        calc = [calculate_likelihood(p[0],p[1],p[2],z) for p in self.data];
        likelihood = np.array([x[0] for x in calc])
        beta = [x[1] for x in calc]
        oblique_angle = float(len([b for b in beta if b>beta_limit]))/self.n
        if oblique_angle>beta_threshold:
            return
        # normalisation
        weight = np.array([x[3] for x in self.data]) * likelihood
        sum_weight = weight.sum()
        weight /= sum_weight
        # resampling
        cweight = np.cumsum(weight)
        new_data = [[]] * self.n
        for i in range(self.n):
            r = random.random()
            k = bsearch(cweight,r,0,self.n-1)
            # print cweight, r, k
            new_data[i] = self.data[k]
            
        self.data = new_data
    
    def draw(self):
        canvas.drawParticles(self.data);

def bsearch(m, x, a, b):
    if a>=b:
        return a
    k = a + (b-a)/2
    if x<=m[k]:
        return bsearch(m,x,a,k)
    else:
        return bsearch(m,x,k+1,b)

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

def uncertainMove(D):
    global particles
    rm.move_forward(interface,D)  # robot move
    sigma_e = np.sqrt(D) * stdev_e
    sigma_f = np.sqrt(D) * stdev_f
    new_particles = []
    for p in particles.data:  # update particles
        e = random.gauss(0,sigma_e)
        f = random.gauss(0,sigma_f)
        theta = radians(p[2])
        new_particles.append((p[0]+(D+e)*cos(theta), p[1]+(D+e)*sin(theta), p[2]+f, p[3]))
    particles.data = new_particles
    sonar_reading = rm.sonar_measurement(interface)
    particles.update(sonar_reading)
    particles.draw()

def uncertainTurn(alpha):  # in degrees
    global particles
    alpha = alpha % 360
    if alpha<=180:
        rm.turn_left(interface,alpha)
    else:
        rm.turn_right(interface,360-alpha)
    sigma_g = np.sqrt(alpha) * stdev_g
    new_particles = []
    for p in particles.data:
        g = random.gauss(0,sigma_g)
        new_particles.append((p[0], p[1], p[2]+alpha+g,p[3]))
    particles.data = new_particles
    sonar_reading = rm.sonar_measurement(interface)
    particles.update(sonar_reading)
    particles.draw()

def pseudo_uncertainMove(D):
    global particles
    sigma_e = np.sqrt(D) * stdev_e
    sigma_f = np.sqrt(D) * stdev_f
    new_particles = []
    for p in particles.data:  # update particles
        e = random.gauss(0,sigma_e)
        f = random.gauss(0,sigma_f)
        theta = radians(p[2])
        new_particles.append((p[0]+(D+e)*cos(theta), p[1]+(D+e)*sin(theta), p[2]+f, p[3]))
    particles.data = new_particles
    sonar_reading = rm.sonar_measurement(interface)
    particles.update(sonar_reading)
    particles.draw()

def pseudo_uncertainTurn(alpha):  # in degrees
    global particles
    alpha = alpha % 360
    sigma_g = np.sqrt(alpha) * stdev_g
    new_particles = []
    for p in particles.data:
        g = random.gauss(0,sigma_g)
        new_particles.append((p[0], p[1], p[2]+alpha+g,p[3]))
    particles.data = new_particles
    sonar_reading = rm.sonar_measurement(interface)
    particles.update(sonar_reading)
    particles.draw()
    
def navigateToWaypoint((x,y)):
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
        D1 = D/2.0
        line = (pos[0], pos[1], pos[0]+D1*cos(radians(pos[2])), pos[1]+D1*sin(radians(pos[2])))
        canvas.drawLine(line)
        uncertainMove(D1)
        navigateToWaypoint((x,y))

def detect_angle():
    global direction
    pause = pi/180/rspeed
    rm.sonar_rotate_v(interface,rspeed*direction)
    measurements = np.zeros(360)
    for i in range(360):
        measurements[i] = rm.sonar_measurement(interface)
        time.sleep(pause)
    rm.sonar_rotate_v_stop(interface)
    if direction == -1:
        measurements = np.array(measurements[::-1])
    direction = -direction
    print measurements
    pos = np.where(measurements>30)[0]
    a = len(np.where(pos<20)[0])
    b = len(np.where(pos>340)[0])
    if a>1 and b>1:
        for i in range(len(pos)):
            if pos[i] < 80:
                pos[i] += 360
                
    return np.mod((270 - int(np.average(pos)))+360,360)
    
def characterize_location(ls):
    global direction
    pause = pi/180/rspeed
    rm.sonar_rotate_v(interface,rspeed*direction)
    for i in range(len(ls.sig)):
        ls.sig[i] = rm.sonar_measurement(interface)
        time.sleep(pause)
    rm.sonar_rotate_v_stop(interface)
    if direction == -1:
        ls.sig = list(reversed(ls.sig))
    direction = -direction

def compare_signatures(ls1, ls2):  # ls1 is observed
    min_dist, theta = -1, 0
    for i in range(len(ls1)):
        ls_new = ls1[-i:] + ls1[:-i]
        dist = 0
        for x,y in zip(ls_new, ls2):
            dist += (x-y)**2
        if min_dist==-1 or dist<min_dist:
            print i, dist
            min_dist = dist
            theta = i
    return min_dist, theta

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    ls = rm.LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."

def recognize_location():
    ls_obs = rm.LocationSignature();
    characterize_location(ls_obs);
    result=[]
    for idx in range(signatures.size):
        print "STATUS:  Comparing signature " + str(idx) + " with the observed signature."
        ls_read = signatures.read(idx);
        dist, theta    = compare_signatures(ls_obs.sig, ls_read.sig)
        print 'Distance = ' + str(dist) + ' Theta = ' +str(theta)
        result.append((idx,theta,dist))
    return sorted(result, key=lambda x:x[2])[0][:-1]

def move_points_to_B():
    global particles
    dx = PB[0] - PA[0]
    for i in range(particles.n):
        particles.data[i] = (particles.data[i][0]+dx, particles.data[i][1], particles.data[i][2], particles.data[i][3])
        
def move_points_to_C():
    global particles
    dx = PC[0] - PA[0]
    for i in range(particles.n):
        particles.data[i] = (particles.data[i][0]+dx, particles.data[i][1], particles.data[i][2], particles.data[i][3])

    
canvas = rm.Canvas();
mymap = rm.Map();
mymap.add_wall((0,0,0,84));        # a
mymap.add_wall((0,84,42,84));      # b
mymap.add_wall((42,84,42,42));     # c
mymap.add_wall((42,42,252,42));    # d
mymap.add_wall((252,42,252,84));   # e
mymap.add_wall((252,84,294,84));   # f
mymap.add_wall((294,84,294,42));   # g
mymap.add_wall((294,42,504,42));   # h
mymap.add_wall((504,42,504,84));   # i
mymap.add_wall((504,84,546,84));   # j
mymap.add_wall((546,84,546,0));    # k
mymap.add_wall((546,0,0,0));       # o
mymap.draw(canvas);

PA = (21,63)
PB = (273,63)
PC = (525,63)
PD = (21,21)
PE = (273,21)
PF = (525,21)
points = [PA,PB,PC]
path = []




theta =  detect_angle()
particles = Particles(PA[0],PA[1],theta,numberOfParticles)
navigateToWaypoint(PD)
rm.sonar_rotate(interface,90)
left_reading = rm.sonar_measurement(interface)
rm.sonar_rotate(interface,-180)
right_reading = rm.sonar_measurement(interface)
rm.sonar_rotate(interface,90)

print left_reading, right_reading

if left_reading < 35:
    print 'Position C'
    move_points_to_C()
elif right_reading < 35:
    print 'Position A'
    uncertainTurn(90)
    rm.sonar_rotate(interface,-90)
    rm.wall_following(interface,wf_speed,21,half_t,-1)
    rm.sonar_rotate(interface,90)
    pseudo_uncertainMove(PE[0]-PD[0])
    navigateToWaypoint(PB)
    navigateToWaypoint(PE)
    uncertainTurn(90)
    rm.sonar_rotate(interface,-90)
    rm.wall_following(interface,wf_speed,21,half_t,-1)
    rm.sonar_rotate(interface,90)
    pseudo_uncertainMove(PF[0]-PE[0])
    navigateToWaypoint(PC)
    navigateToWaypoint(PF)
    uncertainTurn(-90)
    rm.sonar_rotate(interface,90)
    rm.wall_following(interface,wf_speed,21,whole_t,1)
    rm.sonar_rotate(interface,-90)
    pseudo_uncertainMove(PF[0]-PD[0])
    navigateToWaypoint(PA)
    
else:
    print 'Position B'
    move_points_to_B()


"""
signatures = rm.SignatureContainer(3);
#signatures.delete_loc_files()

location, theta = recognize_location()
print location, theta
particles = Particles(points[location][0],points[location][1],theta,numberOfParticles)

if location==0:
    path = [(21,21),(273,21),(273,63),(273,21),(525,21),(525,63),(525,21),(21,21),(21,63)]
elif location==1:
    path = [(273,21),(21,21),(21,63),(21,21),(525,21),(525,63),(525,21),(273,21),(273,63)]
else:
    navigateToWaypoint(PF)
    uncertainTurn(-90)
    rm.sonar_rotate(interface,90)
    rm.wall_following(interface,wf_speed,21,half_t,1)
    rm.sonar_rotate(interface,-90)
    pseudo_uncertainMove(PF[0]-PE[0])
    navigateToWaypoint(PB)
    navigateToWaypoint(PE)
    uncertainTurn(-90)
    rm.sonar_rotate(interface,90)
    rm.wall_following(interface,wf_speed,21,half_t,1)
    rm.sonar_rotate(interface,-90)
    pseudo_uncertainMove(PE[0]-PD[0])
    navigateToWaypoint(PA)
    navigateToWaypoint(PD)
    uncertainTurn(90)
    rm.sonar_rotate(interface,-90)
    rm.wall_following(interface,wf_speed,21,whole_t,-1)
    rm.sonar_rotate(interface,+90)
    pseudo_uncertainMove(PF[0]-PD[0])
    navigateToWaypoint(PC)
    #path = [(525,21),(273,21),(273,63),(273,21),(21,21),(21,63),(21,21),(525,21),(525,63)]


location, theta = recognize_location()
print location, theta
particles = Particles(points[location][0],points[location][1],theta,numberOfParticles)

if location==0:
    path = [(21,21),(273,21),(273,63),(273,21),(525,21),(525,63),(525,21),(21,21),(21,63)]
elif location==1:
    path = [(273,21),(21,21),(21,63),(21,21),(525,21),(525,63),(525,21),(273,21),(273,63)]
else:
    navigateToWaypoint(PF)
    uncertainTurn(-90)
    rm.sonar_rotate(interface,90)
    rm.wall_following(interface,wf_speed,21,half_t,1)
    rm.sonar_rotate(interface,-90)
    pseudo_uncertainMove(PF[0]-PE[0])
    navigateToWaypoint(PB)
    navigateToWaypoint(PE)
    uncertainTurn(-90)
    rm.sonar_rotate(interface,90)
    rm.wall_following(interface,wf_speed,21,half_t,1)
    rm.sonar_rotate(interface,-90)
    pseudo_uncertainMove(PE[0]-PD[0])
    navigateToWaypoint(PA)
    navigateToWaypoint(PD)
    uncertainTurn(90)
    rm.sonar_rotate(interface,-90)
    rm.wall_following(interface,wf_speed,21,half_t*2,-1)
    rm.sonar_rotate(interface,+90)
    pseudo_uncertainMove(PF[0]-PD[0])
    navigateToWaypoint(PC)
    #path = [(525,21),(273,21),(273,63),(273,21),(21,21),(21,63),(21,21),(525,21),(525,63)]

learn_location()
time.sleep(15)
learn_location()
time.sleep(15)
learn_location()


rm.sonar_rotate(interface,90)
rm.wall_following(interface,wf_speed,21,half_t,1)
rm.sonar_rotate(interface,-90)
rm.turn_right(interface,180)
rm.sonar_rotate(interface,-90)
rm.wall_following(interface,wf_speed,21,half_t,-1)
rm.sonar_rotate(interface,90)


"""