import brickpi
import time
import robotmoves as rm

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

for i in range(6):
    angle = 10.0/2.7454
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    while not interface.motorAngleReferencesReached(motors):
        usReading = interface.getSensorValue(port)    
        if usReading:
            r = int(usReading[0])
        else:
            print "Failed US reading"
            continue
        print r
        time.sleep(0.1)

interface.terminate()