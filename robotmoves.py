import brickpi
import time
from numpy import pi
import os

speed = 2.7454 # cm per angle
turn = 20.02 # degree per angle
sonar_adj = 0
sonarPort = 1
sreadings = 5
motors = [0,1] # right and left
smotor = [3]

def wall_following(interface,v,d,t,direction): #direction==1 means moving with sonar facing left
    interval = 0.02
    kp = 0.5
    cap = 3
    facing = 0  # 1 facing away from wall, -1 facing wall, 0 parallel to wall
    dist = []
    n = 15
    
    steps = int(t / interval)
    motorParams1 = interface.MotorAngleControllerParameters()
    motorParams2 = interface.MotorAngleControllerParameters()
    motorParams1, motorParams2 = initialize_motors_v(motorParams1, motorParams2)
    interface.setMotorAngleControllerParameters(motors[0],motorParams1)
    interface.setMotorAngleControllerParameters(motors[1],motorParams2)
    interface.setMotorRotationSpeedReferences(motors,[v,v])
    dist.append(sonar_measurement(interface) - d)
    
    for i in range(steps):
        diff = sonar_measurement(interface) - d
        if abs(diff)>100:
            time.sleep(interval)
            continue
        if abs(diff-dist[-1])>1:
            diff = dist[-1]
        dist.append(diff)
        if len(dist)<=n:
            time.sleep(interval)
            continue
        dist.pop(0)
        facing = diff - dist[0]
        if facing*diff <0:
            dv = (abs(facing)-1)*(facing/abs(facing))*kp*2*direction
        elif diff==0:
            if facing==0:
                dv = 0
            else:
                dv = kp*facing*direction
        else:
            dv = (diff+2*facing)*kp*direction
        dv = max(min(dv,cap),-cap)
        print dist, ' facing=', facing, ' dv=', dv
        interface.setMotorRotationSpeedReferences(motors,[v+dv,v-dv])
        time.sleep(interval)

    interface.setMotorPwm(motors[0],0)
    interface.setMotorPwm(motors[1],0)
    motorParams1, motorParams2 = initialize_motors(motorParams1, motorParams2)
    interface.setMotorAngleControllerParameters(motors[0],motorParams1)
    interface.setMotorAngleControllerParameters(motors[1],motorParams2)

class Canvas:
    def __init__(self,map_size=475):
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
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self,canvas):
        for wall in self.walls:
            canvas.drawLine(wall);

# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = 360):
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))
    
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;
    
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
    
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
        f = open(filename, 'w')
        for i in range(len(signature.sig)):
            s = str(int(signature.sig[i])) + "\n"
            f.write(s)
        f.close()
    
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."        
        return ls

# --------------------- initilization functions ---------------
def initialize_everything(interface):
    interface.initialize()
    
    interface.motorEnable(motors[0])
    interface.motorEnable(motors[1])
    interface.motorEnable(smotor[0])
    interface.sensorEnable(sonarPort, brickpi.SensorType.SENSOR_ULTRASONIC)

    motorParams1 = interface.MotorAngleControllerParameters()
    motorParams2 = interface.MotorAngleControllerParameters()
    sonarMotorParams = interface.MotorAngleControllerParameters()
    motorParams1, motorParams2 = initialize_motors(motorParams1, motorParams2)
    sonarMotorParams = initialize_sonar_motor(sonarMotorParams)

    interface.setMotorAngleControllerParameters(motors[0],motorParams1)
    interface.setMotorAngleControllerParameters(motors[1],motorParams2)
    interface.setMotorAngleControllerParameters(smotor[0],sonarMotorParams)

def initialize_motors(motorParams1, motorParams2): # initialize motor params
    motorParams1.maxRotationAcceleration = 6.0
    motorParams1.maxRotationSpeed = 12.0
    motorParams1.feedForwardGain = 255/20.0
    motorParams1.minPWM = 20.0
    motorParams1.pidParameters.minOutput = -255
    motorParams1.pidParameters.maxOutput = 255
    motorParams1.pidParameters.k_p = 500.0
    motorParams1.pidParameters.k_i = 320.0
    motorParams1.pidParameters.k_d = 20.0
    
    motorParams2.maxRotationAcceleration = 6.0
    motorParams2.maxRotationSpeed = 12.0
    motorParams2.feedForwardGain = 255/20.0
    motorParams2.minPWM = 20.0
    motorParams2.pidParameters.minOutput = -255
    motorParams2.pidParameters.maxOutput = 255
    motorParams2.pidParameters.k_p = 500.0
    motorParams2.pidParameters.k_i = 320.0
    motorParams2.pidParameters.k_d = 20.0
    
    return motorParams1, motorParams2

def initialize_motors_v(motorParams1,motorParams2): # initialize motor params
    motorParams1.maxRotationAcceleration = 6.0
    motorParams1.maxRotationSpeed = 12.0
    motorParams1.feedForwardGain = 0/20.0
    motorParams1.minPWM = 20.0
    motorParams1.pidParameters.minOutput = -255
    motorParams1.pidParameters.maxOutput = 255
    motorParams1.pidParameters.k_p = 480.0
    motorParams1.pidParameters.k_i = 20.0
    motorParams1.pidParameters.k_d = 20.0

    motorParams2.maxRotationAcceleration = 6.0
    motorParams2.maxRotationSpeed = 12.0
    motorParams2.feedForwardGain = 0/20.0
    motorParams2.minPWM = 20.0
    motorParams2.pidParameters.minOutput = -255
    motorParams2.pidParameters.maxOutput = 255
    motorParams2.pidParameters.k_p = 480.0
    motorParams2.pidParameters.k_i = 20.0
    motorParams2.pidParameters.k_d = 20.0
    
    return motorParams1, motorParams2

def initialize_sonar_motor(sonarMotorParams): # initialize motor params
    sonarMotorParams.maxRotationAcceleration = 6.0
    sonarMotorParams.maxRotationSpeed = 12.0
    sonarMotorParams.feedForwardGain = 0.0
    sonarMotorParams.minPWM = 50.0
    sonarMotorParams.pidParameters.minOutput = -255
    sonarMotorParams.pidParameters.maxOutput = 255
    sonarMotorParams.pidParameters.k_p = 200.0
    sonarMotorParams.pidParameters.k_i = 400.0
    sonarMotorParams.pidParameters.k_d = 20.0
    
    return sonarMotorParams

def initialize_sonar_motor_v(sonarMotorParams): # initialize motor params
    sonarMotorParams.maxRotationAcceleration = 6.0
    sonarMotorParams.maxRotationSpeed = 12.0
    sonarMotorParams.feedForwardGain = 0.0
    sonarMotorParams.minPWM = 10.0
    sonarMotorParams.pidParameters.minOutput = -255
    sonarMotorParams.pidParameters.maxOutput = 255
    sonarMotorParams.pidParameters.k_p = 400.0
    sonarMotorParams.pidParameters.k_i = 10.0
    sonarMotorParams.pidParameters.k_d = 10.0
    
    return sonarMotorParams

# --------------------- movement functions ---------------
def move_forward(interface,x): # move forward x centimeters
    angle = x/speed
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Traveled forward "+str(x)+" cm"

def move_backward(interface,x): # move backward x centimeters
    angle = x/speed
    interface.increaseMotorAngleReferences(motors,[-angle,-angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Traveled backward "+str(x)+" cm"

def Left90deg(interface):
    angle = 90 / turn
    interface.increaseMotorAngleReferences(motors,[angle,-angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Left turn 90 degree"

def Right90deg(interface):
    angle = 90 / turn
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Right turn 90 degree"

def turn_left(interface,x): # left turn x degrees
    angle = x / turn
    interface.increaseMotorAngleReferences(motors,[angle,-angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Left turn "+str(x)+" degrees"

def turn_right(interface,x): # left turn x degrees
    angle = x / turn
    interface.increaseMotorAngleReferences(motors,[-angle,angle])
    while not interface.motorAngleReferencesReached(motors) :
        time.sleep(0.1)
        
    print "Right turn "+str(x)+" degrees"

def sonar_measurement(interface):
    reading = []
    for i in range(sreadings):
        usReading = interface.getSensorValue(sonarPort)
        if usReading:
            reading.append(usReading[0])
    #print 'sonar: '+str(reading)
    if len(reading)==0:
        return -10000
    return sorted(reading)[len(reading)/2] + sonar_adj

def sonar_rotate(interface,x):  # turn sonar motor x degrees anticlockwise
    interface.increaseMotorAngleReferences(smotor,[x*pi/180.0])
    while not interface.motorAngleReferencesReached(smotor) :
        time.sleep(0.02)
        
def sonar_rotate_v(interface,v):
    sonarMotorParams = interface.MotorAngleControllerParameters()
    sonarMotorParams = initialize_sonar_motor_v(sonarMotorParams)
    interface.setMotorAngleControllerParameters(smotor[0],sonarMotorParams)
    interface.setMotorRotationSpeedReferences(smotor,[v])
    
def sonar_rotate_v_stop(interface):
    interface.setMotorPwm(smotor[0],0)
    sonarMotorParams = interface.MotorAngleControllerParameters()
    sonarMotorParams = initialize_sonar_motor(sonarMotorParams)
    interface.setMotorAngleControllerParameters(smotor[0],sonarMotorParams)
    