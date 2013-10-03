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

# set logging
fname = 'mar13_' + str(int(time.time())) + '.csv'
f = open(fname,'w')

# connect to GPS
serialport = serial.Serial("/dev/gps0", 115200)

# setup compass
compass = hmc5883l(gauss = 4.7, declination = (-13,25))

# init button and ready LED
#button = 17
#startLED = 18
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(startLED,GPIO.OUT)
#GPIO.setup(button,GPIO.IN)

# init steering
steering = 24
servo = PWM.Servo(pulse_incr_us=1)
servo.set_servo(steering,1500)

# define waypoints list and set current
wps = [[41.024353,-73.762033],[41.024005,-73.761872],[41.023954,-73.762109],[41.024314,-73.762242]]
wpn = 0 

def getWaypoint(n):
    wp = wps[n]
    return (float(wp[0]),float(wp[1]))

def blinkLED(n):
    # blink LED on/off n number of times
    # LED is on/off for 0.5/0.2 seconds
    i = 0
    while i < n:
        GPIO.output(startLED,1)
        time.sleep(0.5)
        GPIO.output(startLED,0)
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
    return DD

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
    the_stdout, the_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = open(os.devnull,"w")
    servo.set_servo(steering,steerAng)
    sys.stdout, sys.stderr = the_stdout, the_stderr

def main():
    #while True:
        #if (GPIO.input(button)):
            #blinkLED(3)
            global wpn
            wpLat, wpLong = getWaypoint(wpn)
            
            ltime = 0
            
            while True:
                #print ('--Begin Loop--')
                btime = int(round(time.time() * 1000))
                
                GPS = getLocation()
                myLat = float(GPS[0])
                myLong = float(GPS[2])
                distance = getDistance(myLat,myLong,wpLat,wpLong)
                # check if we are close to waypoint (GPS accuracy is 8.2')	
                if (distance <= 8.5):
                    wpn = 0 if wpn + 1 > wps.len() else wpn + 1
                    wpLat, wpLong = getWaypoint(wpn)
                    distance = getDistance(myLat,myLong,wpLat,wpLong)
                	
                bearing = getBearing(myLat,myLong,wpLat,wpLong)
                #heading = float(GPS[4])
                heading = compass.heading() - 10
                
                course = bearing - heading
                #course = 0 - heading
                if (course >= 180):
                    course -= 360;
                if (course <= -180):
                    course +=360
                # correct for max turn capability
                if (course > 35):
                    course = 35
                if (course < -35):
                    course = -35

                changeDirection(course)

                # output to screen
                sout = 'Lat: ' + str(myLat) + " || "
                sout = sout + 'Long: ' + str(myLong) + " || "
                sout = sout + 'Waypoint: ' + str(wpn) + " || "
                sout = sout + 'Bearing: ' + str(bearing) + " || "
                sout = sout + 'Distance: ' + str(distance) + " || "
                sout = sout + 'Heading: ' + str(heading) + " || "
                sout = sout + 'Course: ' + str(course)
                sys.stdout.write("\r" + sout)
                sys.stdout.flush()

                #print ('Steering: ' + str(steerAng))
                # output to log
                etime = int(round(time.time() * 1000))
                ltime += (etime - btime)
                output = str(ltime) + ','
                output = output + str(myLat) + ','
                output = output + str(myLong) + ','
                output = output + str(wpn) + ','
                output = output + str(bearing) + ','
                output = output + str(distance) + ','
                output = output + str(heading) + ','
                output = output + str(course) + ','
                #output = output + str(steerAng)
                output = output + '\n'
                f.write(output)
                #time.sleep(0.25)
                #n += 1
                #print ('--End Loop--\n\n')
                
if __name__=="__main__":
	main()
