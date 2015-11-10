import brickpi
import time

interface = brickpi.Interface()
interface.initialize()

port = 1 # need to change
interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

reading = []

while True:
    usReading = interface.getSensorValue(port)    
    if usReading:
        r = usReading[0]
    else:
        print "Failed US reading"
        continue
    reading.append(r)        
    print r
    time.sleep(0.05)

interface.terminate()