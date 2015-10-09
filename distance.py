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
from webcolors import name_to_rgb
import pygame
from pygame.locals import Rect, QUIT, KEYDOWN, K_q, K_ESCAPE
import random
#import urllib, pycurl, os           # needed for text to speech

import numpy as np
from numpy import convolve
 
import RPi.GPIO as GPIO     # GPIO is the handle to control pins
import time                 # for time delays
import random               # for random delays
import webcolors            # for converting a color name to RGB

#########################
# Local libraries
#########################
from system_info import get_ram, get_up_stats
from raspbot_functions import getCPUtemperature, fahrenheit_to_rgb
from pid import PID
from rgb_led import rgb_led # for controlling eye color

# Add the directory containing your module to the Python path (wants absolute paths)
scriptpath = "../Adafruit-Raspberry-Pi-Python-Code/Adafruit_ADS1x15"
sys.path.append(os.path.abspath(scriptpath))
from Adafruit_ADS1x15 import ADS1x15

###################################################################
# Globals / Constants
###################################################################
# all the hit array constants
# HAMA = Hit Array Moving Average
HAMA_SIZE = 10                   # the number of measurements to average
                                # this affects how slowly a person is detected
                                # the bigger the number, the slower it works
                                # but it also takes out noise blips and variability
                                # in the sensor
MA0 = [0]*HAMA_SIZE

# Command line argument constants
DEBUG = 0               # set this to 1 to see debug messages on monitor
ROAM = 0                # if true, robot will look for a heat signature
RAND = 0                # Causes random head movement when idle
MONITOR = 1             # assume a monitor is attached
CALIBRATION = 0         # don't perform calibration cycle

def movingaverage (values, window):
    weights = np.repeat(1.0, window)/window
    sma = np.convolve(values, weights, 'valid')
    return sma
 
def hstack_push(array, element):
    """
    This function pushes an element in to the proper
    position in the hit array moving average stack array
    """
    new_array = [0]*HAMA_SIZE
    for ei in range(HAMA_SIZE-1, 0, -1):
        new_array[ei] = array[ei-1]
#        print 'new_array['+str(ei)+'] = array['+str(ei-1)+']'
    new_array[0] = element                  # enter new data
#    print 'new_array[0] = '+str(element)
    return new_array    

class UDPClient(object):
    def __init__(self, port):
        self.addr = ('<broadcast>', port)  
        self.UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDPSock.bind(('', 0))
        self.UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def sendMessage(self, message):
        try:
            if len(message):
                message = str(message)
                print "Sending Message:", message
                self.UDPSock.sendto(message, self.addr)
        except:
            self.UDPSock.close()
            sys.exit(0)

class distance(object):

    def __init__(self):

        # Initialize the Analog to Digital converter for reading the distance sensor
        ADS1015 = 0x00  # 12-bit ADC
        ADS1115 = 0x01	# 16-bit ADC

        # Select the gain
        # gain = 6144  # +/- 6.144V
        self.gain = 4096  # +/- 4.096V
        # gain = 2048  # +/- 2.048V
        # gain = 1024  # +/- 1.024V
        # gain = 512   # +/- 0.512V
        # gain = 256   # +/- 0.256V

        # Select the sample rate
        # sps = 8    # 8 samples per second
        # sps = 16   # 16 samples per second
        # sps = 32   # 32 samples per second
        # sps = 64   # 64 samples per second
        # sps = 128  # 128 samples per second
        self.sps = 250  # 250 samples per second
        # sps = 475  # 475 samples per second
        # sps = 860  # 860 samples per second

        # Initialise the ADC using the default mode (use default I2C address)
        # Set this to ADS1015 or ADS1115 depending on the ADC you are using!
        self.DISTANCE_HANDLE = ADS1x15(ic=ADS1115)

##    def __del__(self):
##        GPIO.cleanup()          # reset GPIOs

    def read(self):
        # read the distance sensor
        distance_in_mv = self.DISTANCE_HANDLE.readADCSingleEnded(0, self.gain, self.sps)
        return (27/(distance_in_mv/1000))

    def start(self):
    #############################
    # Main while loop
    #############################
        event_change = False
        return_string = "valve=pool"

        UDP_client = UDPClient(33333)

        while True:                 # The main loop
    # read the distance sensor
            distance_in_cm = self.read()

##            RET_MA0 = hstack_push(MA0, distance_in_cm)
##            print MA0
##            cm_moving_average = int(round(movingaverage(RET_MA0,HAMA_SIZE)))

#            print('distance=%0.2f'%distance_in_cm)

            if (distance_in_cm < 15.0):
                if(return_string == "valve=pool"):
                    event_change = True
                return_string = "valve=spa"
            else:
                if(return_string == "valve=spa"):
                    event_change = True
                return_string = "valve=pool"

            if (event_change):
                print(return_string)
                UDP_client.sendMessage(return_string)
                event_change = False
                
            time.sleep(1)
        

###############################
#
# Start of main line program
#
###############################
if __name__ == '__main__':  #test code

    DEBUG = True

    try:
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

    #############################
    # Main while loop
    #############################
        while True:                 # The main loop

            valve_detector = distance()

            valve_detector.start()
            
            time.sleep(0.25)

    #############################
    # End main while loop
    #############################

    except KeyboardInterrupt:
        print 'Keyboard Interrupt Exception!'
        CRASH_MSG = '\r\nKeyboard interrupt; quitting'
        crash_and_burn(CRASH_MSG)

    except IOError:
        now_string = str(datetime.now())
        print 'I/O Error Exception! Quitting at '+now_string
        GPIO.cleanup()

