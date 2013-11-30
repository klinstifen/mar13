#!/usr/bin/python
import serial
import RPi.GPIO as GPIO
import time
import math
from RPIO import PWM
import logging
from math import *
from hmc5883l import hmc5883l 
import sys
import os

# -----------------------------------------
# ----- begin declare variables 
# -----------------------------------------

# log filename
logfile = 'log.csv'

# waypoint filename
wpfile = "waypoints.txt"

# GPS serial port
serialport = serial.Serial("/dev/gps0", 115200)

# xbee serial port
#xbee = serial.Serial("/dev/gps0", 9600)

# compass adjustment
cAdjust = +2

# GPIO pins
goButton = 17
readyLED = 18
steering = 24
#throttle = 23

# GPS accuracy * 2
GPSaccuracy = 10

# -----------------------------------------
# ----- end declare variables 
# -----------------------------------------

GPIO.setmode(GPIO.BCM)
GPIO.setup(readyLED,GPIO.OUT)
GPIO.setup(goButton,GPIO.IN)

# setup compass
#mydec = -13,25
compass = hmc5883l(gauss = 4.7, declination = (-7,13))

# read in waypoints
wps = []
wplist = open(wpfile,r)
for line in wplist:
    wps.append([line])
wplist.close()

# open logfile
f = open(logfile,'a')

# init steering / throttle
servo = PWM.Servo(pulse_incr_us=1)
servo.set_servo(steering,1500)
#servo.set_servo(throttle,1500)

def blinkLED(n):
    # blink LED on/off n number of times
    # LED is on/off for 0.5/0.2 seconds
    i = 0
    while i <= n:
        GPIO.output(readyLED,1)
        time.sleep(0.5)
        GPIO.output(readyLED,0)
        time.sleep(0.2)
        i += 1
    
def getDegrees(dms,nw):
    # convert GPS in dddmm.mmmm format to dd.dddd
    if (int(dms[0:1]) != 0):
        dms = str(0) + dms
    D = int(dms[0:3])
    M = float(dms[3:])
    #S = float(dms[5:])
    DD = D + float(M)/60 #+ float(S)/3600
    if (nw == "S" or nw == "W"): DD *= -1
    return float(DD)

def getLocation():
    # read serial port and parse out GPS lat/long/compass/heading info
    # return a list of found values
    GPS = [0, 1, 2, 3, 4]
    GPSFound = 0

    while not GPSFound:
        NMEAline = serialport.readline()
        NMEAdata = NMEAline.split(',')
        if (NMEAdata[0] == "$GPRMC"):
            # make sure we have GPS lock
            if NMEAdata[2] == "V": continue
            GPS[0] = round(getDegrees(NMEAdata[3],NMEAdata[4]),6) # lat
            GPS[1] = NMEAdata[4] # n/s
            GPS[2] = round(getDegrees(NMEAdata[5],NMEAdata[6]),6) # long
            GPS[3] = NMEAdata[6] # e/w
            GPS[4] = NMEAdata[8] # heading
            GPSFound = 1
    return GPS

def getBearing(lat1, long1, lat2, long2):
    long1, lat1, long2, lat2 = map(radians, [long1, lat1, long2, lat2])
    dLon = long2 - long1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2) \
        - sin(lat1) * cos(lat2) * cos(dLon)
    b = round(degrees(atan2(y, x)))
    b = b + 360 if b < 0 else b
    return b
    
def getDistance(lat1, long1, lat2, long2):
    lat1, long1, lat2, long2 = map(radians, [lat1, long1, lat2, long2])
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    d = 3961 * c
    d = round(d * 5280,6) # convert distance to feet

    print ('Distance: ' + str(d))
    return d

def changeDirection(course):
    # change rc steering to match course angle
    steerAng = (round(course / 3.5) * 50) + 1500
    servo.set_servo(steering,steerAng)

def main():
    GPIO.output(readyLED,1)
    while True:
        if (GPIO.input(goButton)):
            # get ready
            blinkLED(3)
            # go
            #servo.set_servo(throttle,1600)
            
            # time of run
            tor = str(time.strftime("%d%m%Y")) + str(time.strftime("%H%M%S"))
            
            # set loop count
            n = 0
            
            for wp in wps:
                wpLat = float(wp[0])
                wpLong = float(wp[1])
                distance = GPSaccuracy
                while distance >= GPSaccuracy:      
                    start = int(round(time.time() * 1000))
                    GPS = getLocation()
                    myLat = GPS[0]
                    myLong = GPS[2]
                    bearing = getBearing(myLat,myLong,wpLat,wpLong)
                                        
                    heading = compass.heading() + cAdjust
                    
                    while (course = bearing - heading) > 0:
                        if (course >= 180):
                            course -= 360
                        if (course <= -180):
                            course +=360
                        # correct for max turn capability
                        if (course > 35):
                            course = 35
                        if (course < -35):
                            course = -35

                        changeDirection(course)
                        heading = compass.heading() + cAdjust
                    
                    # -----------------------
                    # ---- output to log 
                    # -----------------------
                    end = int(round(time.time() * 1000))
                    lduration = (end - start)

                    # -----------------------
                    # --- output to xbee 
                    # -----------------------
                    output = str(n) + ' || ' + str(myLat) + ' || ' + str(myLong) + ' || ' + \
                    str(wp) + ' || ' + str(bearing) + ' || ' + str(distance) + ' || ' + \
                    str(heading) + ' || ' + str(course) + ' || ' + str(lduration) + '\r'
                
                    #xbee.write(output)                   
                    
                    # ---- header
                    # tor,loop,lat,long,waypoint,bearing,distance,heading,course,loop duration
                    output = str(tor), str(n) + ',' + str(myLat) + ',' + str(myLong) + ',' + \
                    str(wp) + ',' + str(bearing) + ',' + str(distance) + ',' + \
                    str(heading) + ',' + str(course) + ',' + str(lduration) + '\n'
               
                    f.write(output)
                    n += 1
                    distance = getDistance(myLat,myLong,wpLat,wpLong)
                    
            f.close()
            # stop 
            #servo.set_servo(throttle,1500)
            
                             
                
if __name__=="__main__":
	main()
