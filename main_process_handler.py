#! /usr/bin/python
"""
Created 10/1/15 by Greg Griffes
Measure distance using the Adafruit IR distance sensor connected to an
Adafruit I2C 16bit ADC.
"""

###################################################################
# Libraries
###################################################################

import smbus, sys, os, pigpio, time, numpy, socket
from datetime import datetime
import RPi.GPIO as GPIO     # GPIO is the handle to control pins

# Add the directory containing your module to the Python path (wants absolute paths)
scriptpath = "../utils"
sys.path.append(os.path.abspath(scriptpath))
from processhandler import *

#########################
# Local libraries
#########################
from distance import *

# Command line argument constants
DEBUG = 0               # set this to 1 to see debug messages on monitor
ROAM = 0                # if true, robot will look for a heat signature
RAND = 0                # Causes random head movement when idle
MONITOR = 1             # assume a monitor is attached
CALIBRATION = 0         # don't perform calibration cycle

def server_start(event_q):
	port = 33333
	# Create socket and bind to address
	UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	UDPSock.bind(('', port))
	try:
		print 'starting server on port', port
		while True:
			data, addr = UDPSock.recvfrom(1024)
			event_q.put(data)
	except(KeyboardInterrupt):
		UDPSock.close()
		print 'Server stopped.'
		sys.exit(0)

###############################
#
# Start of main line program
#
###############################

DEBUG = True

##    # Initialize pygame
##        init_pygame()
# Initialize i2c bus address
I2C_BUS = smbus.SMBus(1)
time.sleep(0.1)                # Wait

GPIO.setwarnings(False) # turn off warnings about DMA channel in use
GPIO.setmode(GPIO.BCM)

# make some space
print ''
if DEBUG:
    print('Initializing Pigpio...')

# intialize pigpio library and socket connection for daemon (pigpiod)
PIGPIO_HANDLE = pigpio.pi()              # use defaults
PIGPIO_VERSION = PIGPIO_HANDLE.get_pigpio_version()

# initalize the distance class    
valve_detector = distance()


#############################
# Main while loop
#############################
functions = [server_start]
p = ProcessHandler(functions)
p.start()

try:
    while True:
        p.watchDog()
        
        if not p.event_q.empty():
            print p.event_q.get()
            
        time.sleep(0.01)
        
except(KeyboardInterrupt):
    p.close()
    sys.exit(0)

##    while True:                 # The main loop
##
##        print 'loop'
##        p.watchdog()
##
##        if not p.event_q.empty():
##            print p.event_q.qsize()
##            event = p.event_q.get()
##            dateStr, timeStr = util.timeStamp()
##            packet = {}
##            packet = {timeStr: event['event']}
##            path = event['name'] + "/" + dateStr
##
##            if event['name'] == "valve":
##                for key in packet:
##                    packet = packet[key]['valve']
##                    print packet
##                result = pool.apply_async(valve_detector.start())
##
##        time.sleep(0.25)

#############################
# End main while loop
#############################
