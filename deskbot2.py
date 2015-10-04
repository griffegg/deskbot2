#! /usr/bin/python
"""
Created 10/1/15 by Greg Griffes
Second generation desktop robot main program

 A Python command line tool for a robot based on the Raspberry Pi
 By Greg Griffes http://yottametric.com
 GNU GPL V3

 A better way to run the script automatically?
 http://www.stuffaboutcode.com/2012/06/raspberry-pi-run-program-at-start-up.html

 This file is automatically run at boot time using the following method
 edit the /etc/rc.local file using sudo nano /etc/rc.local
 add these two lines at the end before "exit 0"
 sudo pigpiod # starts the pigpio daemon
 sudo python /home/pi/projects_ggg/deskbot2/deskbot2.py -nomonitor -roam
 >> /home/pi/projects_ggg/raspbot/deskbot2_python.log 2>&1 &
 The above command will route all regular and error messages to a log file
 The log file is appended each time, so it can get large and should be
 deleted every once in a while.

 There is another log file created by this program when running
 idependently is created in the raspbot directory about every
 five minutes the log file is closed and reopened. This is probably
 redundant but it contains more debug info than the python log.

 !!!!!!!!!!!!!!!!!
 remember to run this as root "sudo ./deskbot2 -debug -roam" so that
 DMA can be used for the servo and the GPIO pins can be used
 !!!!!!!!!!!!!!!!!

 To get sound permanently sent to the analog audio headphone jack:
    Run sudo raspi-config, advanced options, audio 
"""

###################################################################
# Libraries
###################################################################

import smbus, sys, os, pigpio, time, numpy
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

scriptpath = "../omrond6t"
sys.path.append(os.path.abspath(scriptpath))
from omrond6t import *

scriptpath = "../zilog_ZDMII"
sys.path.append(os.path.abspath(scriptpath))
from zilog_ZDMII import *


###################################################################
# Globals / Constants
###################################################################
DEBUG = 1
MONITOR = 1

# GPIO Constants
SERVO_GPIO_PIN = 4
# all the LED constants

# GPIO assignments for the hit LEDs (three colors, red, yellow, green)
#   red = burn hazard (hit_array[x] > 4
#   yellow = possible person (hit_array[x] == 1
#   green = person probable (hit_array[x] >=2 <= 4)
#   dark = no hit detected (hit_array[x] = 0)
#
#   LEDs are connected to ground on one pin and the other three pins
#       (RGY) take +3v through a 1k resistor. The LEDs are LUMEX
#       SSL-LX5097 or DigiKey 67-2184-ND
#
LED0_RED = 11   # AKA: BCM GPIO 17
LED0_YEL = 12   # AKA: BCM GPIO 18
LED0_GRN = 13   # AKA: BCM GPIO 27

LED1_RED = 15   # AKA: BCM GPIO 22
LED1_YEL = 16   # AKA: BCM GPIO 23
LED1_GRN = 18   # AKA: BCM GPIO 24

LED2_RED = 22   # AKA: BCM GPIO 25
LED2_YEL = 29   # AKA: BCM GPIO 5
LED2_GRN = 31   # AKA: BCM GPIO 6

LED3_RED = 32   # AKA: BCM GPIO 12
LED3_YEL = 35   # AKA: BCM GPIO 19
LED3_GRN = 36   # AKA: BCM GPIO 16

LED_ON = True
LED_OFF = False
LAST_KNOWN_LED_POS = 0  # counter keeps track of which LED to light
LED_POS_MAX = 4
LIT_LED = LED0_RED
LED_GPIO_PIN = 7    # GPIO number that the LED is connected to
                    # (BCM GPIO_04 (Pi Hat) is the same as BOARD pin 7)
                    # See "Raspberry Pi B+ J8 Header" diagram
LED_STATE = True    # this is the LED in the speaker head mouth

# all the servo constants
PERIOD = 20000.0
LOW_TO_HIGH_IS_COUNTERCLOCKWISE = 0
LOW_TO_HIGH_IS_CLOCKWISE = 1
CTR_SERVO_POSITION = 1500
MINIMUM_SERVO_GRANULARITY = 10  # microseconds
SERVO_CUR_DIR_CW = 1            # Direction to move the servo next
SERVO_CUR_DIR_CCW = 2
ROAMING_GRANULARTY = 20         # the distance moved during roaming
MOVE_DIST_CLOSE = 100     
MOVE_DIST_SHORT = 160      
MOVE_DIST_MEDIUM = 220       
MOVE_DIST_FAR = 300       
SERVO_ENABLED = 1   # set this to 1 if the servo motor is wired up
SERVO_GPIO_PIN = 11 # GPIO number (GPIO 11 aka. SCLK)
ROAM_MAX = 600         # Max number of times to roam between person
                        # detections (roughly 0.5 seconds between roams
ROAM_COUNT = 0 # keep track of head roams so that we can turn it off
# initialize the servo to face directly forward
SERVO_POSITION = CTR_SERVO_POSITION
# set initial direction
SERVO_DIRECTION = SERVO_CUR_DIR_CW

# Some servos move CW and others move CCW using the
# same number. The colors of the wires on the
# servo seem to indicate different servos:
# brown, red, orange seems to be HIGH_TO_LOW is clockwise
# (2400 is full CCW and 600 is full CW)
# black, red, yellos seems to be LOW_TO_HIGH is clockwise
# (2400 is full CW and 600 is full CCW)
HITEC_HS55 = LOW_TO_HIGH_IS_CLOCKWISE   # Yellow, Red, Black wires
FITECH_MICRO_SERVO_FS90 = LOW_TO_HIGH_IS_COUNTERCLOCKWISE   # org, red, brn
HITEC_HS5055MG = LOW_TO_HIGH_IS_CLOCKWISE   # yellow, red, black

SERVO_TYPE = HITEC_HS5055MG

if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    MIN_SERVO_POSITION = 2300
    MAX_SERVO_POSITION = 700
else:
    MIN_SERVO_POSITION = 700
    MAX_SERVO_POSITION = 2300

SERVO_LIMIT_CW = MIN_SERVO_POSITION
SERVO_LIMIT_CCW = MAX_SERVO_POSITION

# all the hit array constants
# HAMA = Hit Array Moving Average
HAMA_SIZE = 3                   # the number of measurements to average
                                # this affects how slowly a person is detected
                                # the bigger the number, the slower it works
                                # but it also takes out noise blips and variability
                                # in the sensor
HIT_ARRAY_MA0 = [0]*HAMA_SIZE
HIT_ARRAY_MA1 = [0]*HAMA_SIZE
HIT_ARRAY_MA2 = [0]*HAMA_SIZE
HIT_ARRAY_MA3 = [0]*HAMA_SIZE
HMA_I0 = 0
HMA_I1 = 0
HMA_I2 = 0
HMA_I3 = 0
HIT_COUNT_LIMIT = 4             # if less than this, probably not a person

# Command line argument constants
DEBUG = 0           # set this to 1 to see debug messages on monitor
ROAM = 0                # if true, robot will look for a heat signature
RAND = 0                # Causes random head movement when idle
MONITOR = 1             # assume a monitor is attached
CALIBRATION = 0         # don't perform calibration cycle

# Omron constants
RASPI_I2C_CHANNEL = 1       # the /dev/i2c device
OMRON_1 = 0x0a              # 7 bit I2C address of Omron Sensor D6T-44L
OMRON_BUFFER_LENGTH = 19    # Omron data buffer size
OMRON_DATA_LIST = 16        # Omron data array - sixteen 16 bit words
MEASUREMENT_WAIT_PERIOD = 0.25   # time between Omron measurements
OMRON_ERROR_COUNT = 0
OMRON_READ_COUNT = 0

# Audio constants
MAX_VOLUME = 1.0            # maximum speaker volume

# Temperature constants
DEGREE_UNIT = 'F'           # F = Farenheit, C=Celcius
MIN_TEMP = 0            # minimum expected temperature in Fahrenheit
MAX_TEMP = 200          # maximum expected temperature in Fahrenheit
TEMPERATURE_ARRAY = [0.0]*OMRON_DATA_LIST # holds the recently measured temperature
HUMAN_TEMP_MIN = 82     # Human temp min empirically measured at 3 feet away
HUMAN_TEMP_MAX = 98     # Human temp max if they don't have the flu
SENSITIVITY = 3          # degrees > than room temp to detect person
SAMPLED_AVERAGE_TEMP = HUMAN_TEMP_MIN - SENSITIVITY

BURN_HAZARD_TEMP = 101  # temperature at which a warning is given
BURN_HAZARD_CNT = 0     # number of times burn hazard detected
BURN_HAZARD_HIT = 10    # Number used in Hit array to indicate hazard

# Screen constants
SCREEN_DIMENSIONS = [400, 600]  # setup IR window [0]= width [1]= height
# QUADRANT of the display (x, y, width, height)
QUADRANT = [Rect]*OMRON_DATA_LIST
CENTER = [(0, 0)]*OMRON_DATA_LIST      # center of each QUADRANT
PX = [0]*4
PY = [0]*4

# person detecting constants
PREVIOUS_HIT_COUNT = 0
HIT_COUNT = 0
HIT_ARRAY_TEMP = [0]*OMRON_DATA_LIST
HIT_ARRAY = [0]*4
NO_PERSON_COUNT = 0
P_DETECT = False
P_DETECT_COUNT = 0

# log file constants
LOG_MAX = 1200          # number of times through the main while loop
LOGFILE_NAME = "/home/pi/Projects/deskbot2/deskbot2.log"

# audio constants
HELLO_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/20150201_zoe-hello1.mp3"
AFTER_HELLO_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-sorry.mp3"
GOODBYE_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/20150201_chloe-goodbye1.mp3"
BADGE_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/badge_file.mp3"
BURN_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-warning.mp3"
STRETCH_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/stretch.mp3"                      
CPU_105_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-105a.mp3"
CPU_110_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-110a.mp3"
CPU_115_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-115a.mp3"
CPU_120_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-120a.mp3"
CPU_125_FILE_NAME = \
    "/home/pi/Projects/deskbot2/snd/girl-125a.mp3"
SAID_HELLO = 0
SAID_GOODBYE = 1
EXERSIZE_TIMEOUT = 1200   # seconds between exersize reminders
EXERSIZE_TIMEOUT_BLINKS = 10    # number of times to blink LEDs
                                # each blink takes 2 seconds

# state machine constants
STATE_NOTHING = 0
STATE_POSSIBLE = 1
STATE_LIKELY = 2
STATE_PROBABLE = 3
STATE_DETECTED = 4
STATE_BURN = 5
INITIAL_STATE = 100
PERSON_STATE = STATE_NOTHING
PREV_PERSON_STATE = INITIAL_STATE
STATE_POSSIBLE_COUNT = 0
STATE_LIKELY_COUNT = 0
STATE_PROBABLE_COUNT = 0
STATE_DETECTED_COUNT = 0
STATE_COUNT_LIMIT = 5
NTPC_LIMIT = 3
NOTHING_TO_POSSIBLE_COUNT = 0
PTLC_LIMIT = 3
POSSIBLE_TO_LIKELY_COUNT = 0
LTPC_LIMIT = 3
POSSIBLE_TO_PROBABLE_COUNT = 0
PTPC_LIMIT = 3
LIKELY_TO_PROBABLE_COUNT = 0
PTDC_LIMIT = 3
PROBABLE_TO_DETECTED_COUNT = 0

# Miscellaneous constants
CONNECTED = 0           # true if connected to the internet
CPU_105_ON = False      # the CPU can reach 105 easily
MAIN_LOOP_COUNT = 0
BADGE_GPIO_PIN = 38   # AKA: BCM GPIO 20
BADGE = 0

###################################################################
# Functions
###################################################################
def debug_print(message):
    """
    Debug messages are printed to display and log file using this
    """
    now_string = str(datetime.now())
    if DEBUG and MONITOR:
        print now_string+': '+message
    LOGFILE_HANDLE.write('\r\n'+now_string+': '+message)

def temp_to_rgb(temp):
  if temp < 80:
    return (0, 0, 192)
  elif temp >= 80 and temp < 90:
    return (255, 128, 0)
  elif temp > 90:
    return (255, 0, 0)

def crash_and_burn(msg):
    """
    Something bad happend; quit the program
    """
# doing a print here makes sure that the stdout gets a message
    print(msg)
    debug_print(msg)
    SERVO_PWM_HANDLE.stop(SERVO_GPIO_PIN)
##    GPIO.output(LED_GPIO_PIN, LED_OFF)
##    GPIO.output(LED0_RED, LED_ON)
##    GPIO.output(LED0_YEL, LED_OFF)
##    GPIO.output(LED0_GRN, LED_OFF)
##    GPIO.output(LED1_RED, LED_ON)
##    GPIO.output(LED1_YEL, LED_OFF)
##    GPIO.output(LED1_GRN, LED_OFF)
##    GPIO.output(LED2_RED, LED_ON)
##    GPIO.output(LED2_YEL, LED_OFF)
##    GPIO.output(LED2_GRN, LED_OFF)
##    GPIO.output(LED3_RED, LED_ON)
##    GPIO.output(LED3_YEL, LED_OFF)
##    GPIO.output(LED3_GRN, LED_OFF)
    cleanup_and_exit(msg)

def cleanup_and_exit(msg):
    pygame.quit()
    GPIO.cleanup()
    LOGFILE_HANDLE.write(msg+' @ '+str(datetime.now()))
    LOGFILE_HANDLE.close
    sys.exit()

def calc_dc(pulse_in_us):
    return ((float(pulse_in_us) / PERIOD)*100.0)

def set_servo_to_position(new_position):
    """
    Moves the servo to a new position
    """

# make sure we don't go out of bounds
    if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
        if new_position == 0:
            new_position = CTR_SERVO_POSITION
        elif new_position < MAX_SERVO_POSITION:
            new_position = MAX_SERVO_POSITION
        elif new_position > MIN_SERVO_POSITION:
            new_position = MIN_SERVO_POSITION
    else:
        if new_position == 0:
            new_position = CTR_SERVO_POSITION
        elif new_position < MIN_SERVO_POSITION:
            new_position = MIN_SERVO_POSITION
        elif new_position > MAX_SERVO_POSITION:
            new_position = MAX_SERVO_POSITION

# if there is a remainder, make 10us increments
    if (new_position%MINIMUM_SERVO_GRANULARITY < 5):
        final_position = \
        (new_position//MINIMUM_SERVO_GRANULARITY) \
        *MINIMUM_SERVO_GRANULARITY
    else:
        final_position = \
        ((new_position//MINIMUM_SERVO_GRANULARITY)+1) \
        *MINIMUM_SERVO_GRANULARITY

    debug_print('set_servo_to_position: '+str(final_position))
    
    duty_cycle = calc_dc(final_position)
    if DEBUG:
        print('Servo pulse (us): '+str(final_position)+' duty cycle: '+str(duty_cycle))
    SERVO_PWM_HANDLE.ChangeDutyCycle(duty_cycle)
    time.sleep(0.04)

    return final_position
    
def init_pygame():
    pygame.mixer.pre_init(44100,-16,1, 1024) # set for mono
    pygame.init()
    pygame.mixer.init()

def play_sound(volume, message):
    """
    Play an mp3 file
    """
    pygame.mixer.music.set_volume(volume)         
    pygame.mixer.music.load(message)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue

###############################
#
# Start of main line program
#
###############################

# Handle command line arguments
if "-debug" in sys.argv:
    DEBUG = 1         # set this to 1 to see debug messages on monitor
    print('DEBUG switch is on')

if "-cal" in sys.argv:
    CALIBRATION = 1

if "-noservo" in sys.argv:
    SERVO_ENABLED = 0         # assume using servo is default

if "-nomonitor" in sys.argv:
    MONITOR = 0       # assume using servo is default

if "-roam" in sys.argv:
    ROAM = 1          # set this to 1 to roam looking for a person

if "-rand" in sys.argv:
    RAND = 1          # set this to 1 to randomize looking for a person

if "-help" in sys.argv:
    print 'IMPORTANT: run as superuser (sudo) to allow DMA access'
    print '-debug:   print debug info to console'
    print '-cal      run with the calibration delay'
    print '-nomonitor run without producing the pygame temp display'
    print '-noservo: do not use the servo motor'
    print '-roam:    when no person turn head slowly 180 degrees'
    print '-rand:    when roaming randomize the head movement'
    sys.exit()

# Initialize pygame
init_pygame()

try:
# Initialize i2c bus address
    I2C_BUS = smbus.SMBus(1)
    time.sleep(0.1)                # Wait

    GPIO.setwarnings(False) # turn off warnings about DMA channel in use
    GPIO.setmode(GPIO.BCM)
##    GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
##    GPIO.setup(LED_GPIO_PIN, GPIO.OUT)
##    GPIO.setup(BADGE_GPIO_PIN, GPIO.IN, GPIO.PUD_OFF)

# make some space
    print ''
    if DEBUG:
        print('Initializing Pigpio...')

# intialize pigpio library and socket connection for daemon (pigpiod)
    PIGPIO_HANDLE = pigpio.pi()              # use defaults
    PIGPIO_VERSION = PIGPIO_HANDLE.get_pigpio_version()

# Initialize the selected Omron sensor

##    GPIO.output(LED0_GRN, LED_ON)
##    GPIO.output(LED1_GRN, LED_ON)
##    GPIO.output(LED2_GRN, LED_ON)
##    GPIO.output(LED3_GRN, LED_ON)
    if DEBUG:
        print('Initializing Omron sensor...')

    omron = OmronD6T(arraySize=8)
    BYTES_READ, TEMPERATURE_ARRAY = omron.read()
    if DEBUG:
        print('Bytes read: '+str(BYTES_READ)+' temperature: '+str(TEMPERATURE_ARRAY))
    
##
##
##    (OMRON1_HANDLE, OMRON1_RESULT) = \
##        omron_init(RASPI_I2C_CHANNEL, OMRON_1, PIGPIO_HANDLE, I2C_BUS)

    if BYTES_READ != OMRON_BUFFER_LENGTH:
        print('OMRON SENSOR ERROR: Bytes read: '+str(BYTES_READ)+' expected: '+str(OMRON_BUFFER_LENGTH))
##        GPIO.output(LED0_GRN, LED_OFF)
##        GPIO.output(LED1_GRN, LED_OFF)
##        GPIO.output(LED2_GRN, LED_OFF)
##        GPIO.output(LED3_GRN, LED_OFF)
##        GPIO.output(LED0_RED, LED_ON)
##        GPIO.output(LED1_RED, LED_ON)
##        GPIO.output(LED2_RED, LED_ON)
##        GPIO.output(LED3_RED, LED_ON)
        crash_and_burn("Omron Error")

# Open log file

    if DEBUG:
        print('Opening log file: '+LOGFILE_NAME)
    LOGFILE_HANDLE = open(LOGFILE_NAME, 'wb')
    LOGFILE_OPEN_STRING = '\r\nStartup log file opened at ' \
                          +str(datetime.now())
    if DEBUG:
        print LOGFILE_OPEN_STRING
    LOGFILE_ARGS_STRING = '\r\nDEBUG: '+str(DEBUG)+' SERVO: ' \
                          +str(SERVO_ENABLED)+' MONITOR: ' \
                          +str(MONITOR)+ \
                          ' ROAM: '+str(ROAM)+' RAND: '+str(RAND)
# doing a print here makes sure that the stdout gets a message
    if DEBUG:
        print LOGFILE_ARGS_STRING
    LOGFILE_HANDLE.write(LOGFILE_OPEN_STRING)
    LOGFILE_HANDLE.write(LOGFILE_ARGS_STRING)

    CPU_TEMP = getCPUtemperature()
    LOGFILE_TEMP_STRING = '\r\nInitial CPU Temperature = '+str(CPU_TEMP)
    if DEBUG:
        print LOGFILE_TEMP_STRING
    LOGFILE_HANDLE.write(LOGFILE_TEMP_STRING)
        
    LOGFILE_HANDLE.write('\r\nPiGPIO version = '+str(PIGPIO_VERSION))
    debug_print('PiGPIO version = '+str(PIGPIO_VERSION))
    debug_print('Omron sensor bytes = '+str(BYTES_READ))
    debug_print('Max CW: '+str(SERVO_LIMIT_CW)+' Max CCW: '+str(SERVO_LIMIT_CCW))

# Initialize servo position
    GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
    GPIO.setwarnings(False)
    SERVO_PWM_HANDLE = GPIO.PWM(SERVO_GPIO_PIN, 50)  # channel = SERVO_GPIO_PIN, frequency = 50Hz
# duty cycle is Pulse Width divided by Period
# Period at 50Hz is 0.02 or 20 milliseconds, or 20000 microseconds
    PERIOD = float(20000.0)
# to center the servo, a 1500 microsecond pulse is used
# therefore, duty cycle = 1500 / 20000 = 0.075 = 7.5 %
    SERVO_PWM_HANDLE.start(calc_dc(CTR_SERVO_POSITION))

# Initialize the Analog to Digital converter for reading the distance sensor
    ADS1015 = 0x00  # 12-bit ADC
    ADS1115 = 0x01	# 16-bit ADC

    # Select the gain
    # gain = 6144  # +/- 6.144V
    gain = 4096  # +/- 4.096V
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
    sps = 250  # 250 samples per second
    # sps = 475  # 475 samples per second
    # sps = 860  # 860 samples per second

    # Initialise the ADC using the default mode (use default I2C address)
    # Set this to ADS1015 or ADS1115 depending on the ADC you are using!
    DISTANCE_HANDLE = ADS1x15(ic=ADS1115)

################################
# initialize the PID controller
################################

# PID controller is the feedback loop controller for person following
    PID_CONTROLLER = PID(1.0, 0.1, 0.0)

    if CONNECTED:
        speakSpeechFromText("Now might be a good time to stand up and stretch", "stretch.mp3")
        debug_print("Connected to internet")
        LOGFILE_HANDLE.write('\r\nConnected to the Internet')
        play_sound(MAX_VOLUME, "stretch.mp3")
    else:
        debug_print("Not connected to internet")
        LOGFILE_HANDLE.write('\r\nNOT connected to the Internet')   
        
# setup the IR color window
    if MONITOR:
        SCREEN_DIMS = [1440, 900]
        xSize = 8
        ySize = 1
        arraySize = xSize * ySize
        screen = pygame.display.set_mode(SCREEN_DIMS)
        pygame.display.set_caption('Sensor Data')
        pygame.mouse.set_visible(False)
        font = pygame.font.Font(None, 36)
        font2 = pygame.font.Font(None, 72)

        X = []
        Y = []
        temp_hit = 0
        square = []
        center = []
        rect = [Rect] * arraySize

        cellWidth = SCREEN_DIMS[0] / xSize
        cellHeight = SCREEN_DIMS[1] / ySize
        cellWidthCenter = cellWidth / 2
        if cellHeight > cellWidth:
          cellHeight = cellWidth
        cellHeightCenter = cellHeight / 2

        for x in range(xSize):
            X.append(x * cellWidth)

        for y in range(ySize):
            Y.append((y * cellHeight) + (SCREEN_DIMS[1] - cellHeight))

        for x in range(xSize):
          for y in range(ySize):
            square.append((X[x], Y[y], cellWidth, cellHeight))
            center.append((X[x] + cellWidthCenter, Y[y] + cellHeightCenter))

        hit_start_time = time.time()
        hit_time = 11
        person_detect = False

        text = font.render('Thermal Sensor', 1, (255,255,255))
        text_pos = text.get_rect()
        text_pos.center = (SCREEN_DIMS[0]/2,SCREEN_DIMS[1] - cellHeight - 18)
        screen.blit(text, text_pos)



# Establish the initial background temperature for sensitivity
# 
        BYTES_READ, TEMPERATURE_ARRAY = omron.read()
##        (BYTES_READ, TEMPERATURE_ARRAY, ROOM_TEMP) = \
##            omron_read(OMRON1_HANDLE, DEGREE_UNIT, \
##            OMRON_BUFFER_LENGTH, PIGPIO_HANDLE)
        OMRON_READ_COUNT += 1
     
        if BYTES_READ != OMRON_BUFFER_LENGTH: # sensor problem
            OMRON_ERROR_COUNT += 1
            debug_print( \
                'ERROR: Omron thermal sensor failure! Bytes read: '\
                +str(BYTES_READ))
            crash_and_burn("Omron Error 2")
            
##        SAMPLED_AVERAGE_TEMP = numpy.mean(TEMPERATURE_ARRAY)
##        debug_print('SAMPLED_AVERAGE_TEMP = '+str(SAMPLED_AVERAGE_TEMP))
##        HUMAN_TEMP_MIN = SAMPLED_AVERAGE_TEMP + SENSITIVITY

#############################
# Main while loop
#############################
    play_sound(MAX_VOLUME, HELLO_FILE_NAME)
    ram_tuple = [0,0]
    while True:                 # The main loop
        MAIN_LOOP_COUNT += 1
        CPU_TEMP = getCPUtemperature()
        ram_tuple = get_ram()
        debug_print('\r\n^^^^^^^^^^^^^^^^^^^^\r\n    MAIN_WHILE_LOOP: '\
                    +str(MAIN_LOOP_COUNT)+' Pcount: ' \
                    +str(P_DETECT_COUNT)+ \
                    ' Servo: '+str(SERVO_POSITION)+' CPU: '+ \
                    str(CPU_TEMP)+' Uptime = '+str(get_up_stats()[0])+ \
                    ' Free RAM: '+str(ram_tuple[1])+'\r\n^^^^^^^^^^^^^^^^^^^^')
# Check for overtemp
##        if (CPU_TEMP >= 105.0):
##            if CPU_105_ON:
##                play_sound(MAX_VOLUME, CPU_105_FILE_NAME)
##                debug_print('Played 105 audio')
##        elif (CPU_TEMP >= 110.0):
##            play_sound(MAX_VOLUME, CPU_110_FILE_NAME)
##            debug_print('Played 110 audio')
##        elif (CPU_TEMP >= 115.0):
##            play_sound(MAX_VOLUME, CPU_115_FILE_NAME)
##            debug_print('Played 115 audio')
##        elif (CPU_TEMP >= 120.0):
##            play_sound(MAX_VOLUME, CPU_120_FILE_NAME)
##            debug_print('Played 120 audio')
##        elif (CPU_TEMP >= 125.0):
##            play_sound(MAX_VOLUME, CPU_125_FILE_NAME)
##            debug_print('Played 125 audio')

# periododically, write the log file to disk
        if MAIN_LOOP_COUNT >= LOG_MAX:

            SAMPLED_AVERAGE_TEMP = numpy.mean(TEMPERATURE_ARRAY)

            debug_print('\r\nLoop count max reached (' \
                +str(MAIN_LOOP_COUNT)+' at '+str(datetime.now()))
            MAIN_LOOP_COUNT = 0      # reset the counter
            debug_print('\r\nClosing log file at '+str(datetime.now()))
            LOGFILE_HANDLE.close       # for forensic analysis

            LOGFILE_HANDLE = open(LOGFILE_NAME, 'wb')
            debug_print('\r\nLog file re-opened at ' \
                        +str(datetime.now()))
            debug_print(LOGFILE_OPEN_STRING)
            debug_print(LOGFILE_ARGS_STRING)
            debug_print(LOGFILE_TEMP_STRING)
            debug_print('room temp: '+str(ROOM_TEMP))
            debug_print('SAMPLED_AVERAGE_TEMP = '+str(SAMPLED_AVERAGE_TEMP))
            debug_print('human temp threshold = ' \
                       +str(HUMAN_TEMP_MIN))
# Display the Omron internal temperature
            debug_print('Servo Type: '+str(SERVO_TYPE))

# reinitialize the mixer; for some reason the audio drops out
# after extended periods of operating time. See if this fixes
#            pygame.mixer.init()
# it didn't fix the garbling

            NO_PERSON_COUNT = 0
            P_DETECT_COUNT  = 0
            ROAM_COUNT = 0

        if (LED_STATE == False):
            LED_STATE = True
#                debug_print('Turning LED on')
##            GPIO.output(LED_GPIO_PIN, LED_STATE)
        else:
            LED_STATE = False
#                debug_print('Turning LED off')
##            GPIO.output(LED_GPIO_PIN, LED_STATE)
            
        time.sleep(MEASUREMENT_WAIT_PERIOD)

        for event in pygame.event.get():
            if event.type == QUIT:
                set_servo_to_position(CTR_SERVO_POSITION)
                CRASH_MSG = '\r\npygame event QUIT'
                crash_and_burn(CRASH_MSG)
            if event.type == KEYDOWN:
                if event.key == K_q or event.key == K_ESCAPE:
                    set_servo_to_position(CTR_SERVO_POSITION)
                    CRASH_MSG = \
                    '\r\npygame event: keyboard q or esc pressed'
                    crash_and_burn(CRASH_MSG)

# read the raw temperature data
# 
        BYTES_READ, TEMPERATURE_ARRAY = omron.read()
##        (BYTES_READ, TEMPERATURE_ARRAY, ROOM_TEMP) = \
##            omron_read(OMRON1_HANDLE, DEGREE_UNIT, \
##            OMRON_BUFFER_LENGTH, PIGPIO_HANDLE)
        OMRON_READ_COUNT += 1
     
# Display each element's temperature in F
#            debug_print('New temperature measurement')
#            print_temps(TEMPERATURE_ARRAY)

        if BYTES_READ != OMRON_BUFFER_LENGTH: # sensor problem
            OMRON_ERROR_COUNT += 1
            debug_print( \
                'ERROR: Omron thermal sensor failure! Bytes read: '\
                +str(BYTES_READ))
            crash_and_burn("Omron Error 3")

#        if (ROOM_TEMP >= HUMAN_TEMP_MIN):
#        HUMAN_TEMP_MIN = ROOM_TEMP + SENSITIVITY

# read the distance sensor
        voltage = DISTANCE_HANDLE.readADCSingleEnded(0, gain, sps) / 1000

# testing crash_and_burn
#        crash_and_burn("testing")
        
###########################
# Analyze sensor data
###########################
#
# Sensor data is hard to evaluate. Sometimes there is a weak signal
#     that appears to light up single array cells. In addition, the
#     cells are not a perfect 4x4 array. So it seems that each sensor
#     has a detection area lobe reaching out from the sensor.
#     As a result of these "lobes", there are dead spots inbetween
#     sensors. Also, the lobes are not perfectly symetrical; measured
#     10% offset from an adjacent lobe at 10" away from the sensor.
#     Hot spot of one lobe was off by 1" compared to an adjacent lobe.
#
# In addition, the further away an object is the lower its temperature 
#     Therefore, what temperature threshold is considered a person?
#     The room temp sensor is used as a baseline threshold. Anything
#     below the room temp is considered "background radiation" because
#     if there is no person or heat source, the sensors measure lower
#     than room temp (e.g. room temp = 70F, sensors are around 66F).
#     As a person appears, sensors start measuring above room temp. So,
#     who knows if room temp is a good threshold or not? I add
#     a fudge factor to room temp which requires a person to get closer.
#     Therefore, room temp plus fudge factor results in what I call a
#     "hit".
#
# Now, other complicating factors. A person's clothing will shield
#     temperature, so, the sensors mainly "see" face and hands.
#     A coffee cup, light bulb, candle, or other odd heat source light
#     up one of the sensors and if close enough, can trigger a burn
#     hazard. Therefore, another threshold, over the person temperature
#     which is used to say that this is not a person, it must be a fire.
#     Burn threshold is about 100F.
#
# As a result of this behavior, it is hard to say when a person is there
#     much less, where the person is (to the right or to the left)?
#     Using the raw threshold to say hello or goodbye results in false
#     positives and true negatives.
#

##        PREVIOUS_HIT_COUNT = HIT_COUNT
##        HIT_COUNT = 0
##        HIT_ARRAY = [0, 0, 0, 0]
##        HIT_ARRAY_TEMP = \
##                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
##        # go through each array element to find person "hits"
##        # max hit count is 4 unless there is a burn hazard
##
##        HUMAN_TEMP_MIN = SAMPLED_AVERAGE_TEMP + SENSITIVITY
##        debug_print('room temp: '+str(ROOM_TEMP))
##        debug_print('SAMPLED_AVERAGE_TEMP = '+str(SAMPLED_AVERAGE_TEMP))
##        debug_print('human temp threshold = ' \
##                   +str(HUMAN_TEMP_MIN))
##
##        for element in range(0, OMRON_DATA_LIST):
##            if (TEMPERATURE_ARRAY[element] > \
##                BURN_HAZARD_TEMP):
##                HIT_ARRAY_TEMP[element] = BURN_HAZARD_HIT
##                HIT_COUNT += 1
##                
##            elif (TEMPERATURE_ARRAY[element] > \
##                  HUMAN_TEMP_MIN):
##                HIT_ARRAY_TEMP[element] += 1
##                HIT_COUNT += 1
##
##            else:
##                HIT_ARRAY_TEMP[element] = 0
##                
##        # far left column
##        HIT_ARRAY[0] = HIT_ARRAY_TEMP[12]+HIT_ARRAY_TEMP[13]+ \
##                     HIT_ARRAY_TEMP[14]+HIT_ARRAY_TEMP[15]
##        HIT_ARRAY[1] = HIT_ARRAY_TEMP[8]+HIT_ARRAY_TEMP[9]+ \
##                     HIT_ARRAY_TEMP[10]+HIT_ARRAY_TEMP[11] 
##        HIT_ARRAY[2] = HIT_ARRAY_TEMP[4]+HIT_ARRAY_TEMP[5]+ \
##                     HIT_ARRAY_TEMP[6]+HIT_ARRAY_TEMP[7] 
##        # far right column
##        HIT_ARRAY[3] = HIT_ARRAY_TEMP[0]+HIT_ARRAY_TEMP[1]+ \
##                     HIT_ARRAY_TEMP[2]+HIT_ARRAY_TEMP[3]
##
### use a moving average so that variations in sensor data are deadened
##
### save new hit array to the moving average arrays
##        HIT_ARRAY_MA0 = hstack_push(HIT_ARRAY_MA0, HIT_ARRAY[0])
###        print HIT_ARRAY_MA0
##        HIT_ARRAY_MA1 = hstack_push(HIT_ARRAY_MA1, HIT_ARRAY[1])
###        print HIT_ARRAY_MA1
##        HIT_ARRAY_MA2 = hstack_push(HIT_ARRAY_MA2, HIT_ARRAY[2])
###        print HIT_ARRAY_MA2
##        HIT_ARRAY_MA3 = hstack_push(HIT_ARRAY_MA3, HIT_ARRAY[3])
###        print HIT_ARRAY_MA3
##
####        HIT_ARRAY[0] = int(round(movingaverage(HIT_ARRAY_MA0,HAMA_SIZE)))
####        HIT_ARRAY[1] = int(round(movingaverage(HIT_ARRAY_MA1,HAMA_SIZE)))
####        HIT_ARRAY[2] = int(round(movingaverage(HIT_ARRAY_MA2,HAMA_SIZE)))
####        HIT_ARRAY[3] = int(round(movingaverage(HIT_ARRAY_MA3,HAMA_SIZE)))
##
##
### Instead of moving average, let's try bleeding of adjacent channels. e.g. 1010 would return 1110 and 1210 would return 2210
##
##        if (HIT_ARRAY[0] == 1 and HIT_ARRAY[1] == 0 and HIT_ARRAY[2] ==1 and HIT_ARRAY[3] == 0):
##            HIT_ARRAY[1] == 1
##        elif (HIT_ARRAY[0] == 0 and HIT_ARRAY[1] == 1 and HIT_ARRAY[2] ==0 and HIT_ARRAY[3] == 1):
##            HIT_ARRAY[2] == 1
##        elif (HIT_ARRAY[0] == 0 and HIT_ARRAY[1] >= 2 and HIT_ARRAY[2] ==0 and HIT_ARRAY[3] == 0):
##            HIT_ARRAY[0] == 2
##        elif (HIT_ARRAY[0] == 1 and HIT_ARRAY[1] >= 2 and HIT_ARRAY[2] ==0 and HIT_ARRAY[3] == 0):
##            HIT_ARRAY[0] == 2
##        elif (HIT_ARRAY[0] == 1 and HIT_ARRAY[1] >= 2 and HIT_ARRAY[2] ==1 and HIT_ARRAY[3] == 0):
##            HIT_ARRAY[0] == 2
##        elif (HIT_ARRAY[0] == 0 and HIT_ARRAY[1] >= 0 and HIT_ARRAY[2] ==2 and HIT_ARRAY[3] == 0):
##            HIT_ARRAY[3] == 2
##        elif (HIT_ARRAY[0] == 0 and HIT_ARRAY[1] >= 0 and HIT_ARRAY[2] ==2 and HIT_ARRAY[3] == 1):
##            HIT_ARRAY[3] == 2
##        elif (HIT_ARRAY[0] == 0 and HIT_ARRAY[1] >= 1 and HIT_ARRAY[2] ==2 and HIT_ARRAY[3] == 1):
##            HIT_ARRAY[3] == 2
##                        
##        GPIO.output(LED0_RED, LED_OFF)
##        GPIO.output(LED0_YEL, LED_OFF)
##        GPIO.output(LED0_GRN, LED_OFF)
##        if (HIT_ARRAY[0] == 1):
##            GPIO.output(LED0_YEL, LED_ON)
##        elif (HIT_ARRAY[0] >= 2 and HIT_ARRAY[0] <= 4):
##            GPIO.output(LED0_GRN, LED_ON)
##        elif (HIT_ARRAY[0] > 4):
##            GPIO.output(LED0_RED, LED_ON)
##            
##        GPIO.output(LED1_RED, LED_OFF)
##        GPIO.output(LED1_YEL, LED_OFF)
##        GPIO.output(LED1_GRN, LED_OFF)
##        if (HIT_ARRAY[1] == 1):
##            GPIO.output(LED1_YEL, LED_ON)
##        elif (HIT_ARRAY[1] >= 2 and HIT_ARRAY[1] <= 4):
##            GPIO.output(LED1_GRN, LED_ON)
##        elif (HIT_ARRAY[1] > 4):
##            GPIO.output(LED1_RED, LED_ON)
##            
##        GPIO.output(LED2_RED, LED_OFF)
##        GPIO.output(LED2_YEL, LED_OFF)
##        GPIO.output(LED2_GRN, LED_OFF)
##        if (HIT_ARRAY[2] == 1):
##            GPIO.output(LED2_YEL, LED_ON)
##        elif (HIT_ARRAY[2] >= 2 and HIT_ARRAY[2] <= 4):
##            GPIO.output(LED2_GRN, LED_ON)
##        elif (HIT_ARRAY[2] > 4):
##            GPIO.output(LED2_RED, LED_ON)
##            
##        GPIO.output(LED3_RED, LED_OFF)
##        GPIO.output(LED3_YEL, LED_OFF)
##        GPIO.output(LED3_GRN, LED_OFF)
##        if (HIT_ARRAY[3] == 1):
##            GPIO.output(LED3_YEL, LED_ON)
##        elif (HIT_ARRAY[3] >= 2 and HIT_ARRAY[3] <= 4):
##            GPIO.output(LED3_GRN, LED_ON)
##        elif (HIT_ARRAY[3] > 4):
##            GPIO.output(LED3_RED, LED_ON)
##                             
##        debug_print('\r\n-----------------------\r\nhit array: '+\
##                    str(HIT_ARRAY[0])+str(HIT_ARRAY[1])+ \
##                    str(HIT_ARRAY[2])+str(HIT_ARRAY[3])+ \
##                    '\r\nhit count: '+str(HIT_COUNT)+ \
##                    '\r\n-----------------------')
##
##        if max(TEMPERATURE_ARRAY) > BURN_HAZARD_TEMP:
##            PERSON_STATE = STATE_BURN

        if MONITOR:
            for i in range(arraySize):
                if TEMPERATURE_ARRAY[i] >= 80:
                    temp_hit += 1

                screen.fill(temp_to_rgb(TEMPERATURE_ARRAY[i]), square[i])

                text = font.render(str(i+1), 1, (255,255,255))
                text_pos = text.get_rect()
                text_pos.center = (center[i][0], SCREEN_DIMS[1] - cellHeight + 18)
                screen.blit(text, text_pos)

                text = font.render(str(int(TEMPERATURE_ARRAY[i])) + chr(0xb0) + "F", 1, (255,255,255))
                text_pos = text.get_rect()
                text_pos.center = center[i]
                screen.blit(text, text_pos)

            hit_time = time.time() - hit_start_time

            if temp_hit > 3:
                person_detect = True
                hit_start_time = time.time()
            elif temp_hit <= 3 and hit_time > 10:
                person_detect = False

# Area for distance measurement
            screen.fill(webcolors.name_to_rgb("cornflowerblue"), (0,0,SCREEN_DIMS[0],180))
            text = font2.render('Distance: '+str(voltage), 1, webcolors.name_to_rgb("sienna"))
            text_pos = text.get_rect()
            text_pos.center = (SCREEN_DIMS[0]/2,90)
            screen.blit(text, text_pos)

# Area for motion sensing
            screen.fill(webcolors.name_to_rgb("teal"), (0,180,SCREEN_DIMS[0],180))
            text = font2.render('Motion: '+str(voltage), 1, webcolors.name_to_rgb("gold"))
            text_pos = text.get_rect()
            text_pos.center = (SCREEN_DIMS[0]/2,270)
            screen.blit(text, text_pos)

# push it to the screen
            pygame.display.update()
            time.sleep(0.01)

#############################
### Burn Hazard Detected !
#############################
##        if (PERSON_STATE == STATE_BURN):
##            debug_print('STATE: BURN: Burn Hazard cnt: ' \
##                       +str(BURN_HAZARD_CNT)+' ROAM COUNT = ' \
##                       +str(ROAM_COUNT))
##            ROAM_COUNT = 0
##            BURN_HAZARD_CNT += 1
##            LED_STATE = True
##            GPIO.output(LED_GPIO_PIN, LED_STATE)
##            if MONITOR:
##                SCREEN_DISPLAY.fill(name_to_rgb('red'), \
##                                    MESSAGE_AREA)
##                SCREEN_TEXT = FONT.render("WARNING! Burn danger!", \
##                                          1, name_to_rgb('yellow'))
##                SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
##                SCREEN_TEXT_POS.center = MESSAGE_AREA_XY
##                SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)
### update the screen
##                pygame.display.update()
##
##            debug_print('\r\n'+"Burn hazard temperature is " \
##                       +"%.1f"%max(TEMPERATURE_ARRAY)+" degrees")
##            
##            # play this only once, otherwise, its too annoying
##            if (BURN_HAZARD_CNT == 1):
##                play_sound(MAX_VOLUME, BURN_FILE_NAME)
##                debug_print('Played Burn warning audio')
##
##            MOVE_DIST = 0
##            MOVE_CW = True
##            HAZARD_POSITION = 0
##            
##            if (HIT_ARRAY[0] > BURN_HAZARD_HIT and \
##                HIT_ARRAY[1] < BURN_HAZARD_HIT and \
##                HIT_ARRAY[2] < BURN_HAZARD_HIT and \
##                HIT_ARRAY[3] < BURN_HAZARD_HIT):
##                    MOVE_DIST = MOVE_DIST_SHORT
##                    MOVE_CW = True
##
##            elif (HIT_ARRAY[0] < BURN_HAZARD_HIT and \
##                  HIT_ARRAY[1] < BURN_HAZARD_HIT and \
##                  HIT_ARRAY[2] < BURN_HAZARD_HIT and \
##                  HIT_ARRAY[3] > BURN_HAZARD_HIT):
##                      MOVE_DIST = MOVE_DIST_SHORT
##                      MOVE_CW = False
##
##            if (MOVE_DIST > 0):
##                HAZARD_POSITION = \
##                    resolve_new_position(MOVE_CW, \
##                                         SERVO_POSITION, \
##                                         MOVE_DIST)
##                debug_print('hazard_position: Pos: '+ \
##                            str(HAZARD_POSITION))
##                SERVO_POSITION = move_head(HAZARD_POSITION, \
##                                           SERVO_POSITION)
##
##            if max(TEMPERATURE_ARRAY) > BURN_HAZARD_TEMP:
##                PERSON_STATE = STATE_BURN
##            else:
### Drop back to looking for a person
##                PERSON_STATE = STATE_NOTHING
##
####                if CONNECTED:
####                    try:
####                        speakSpeechFromText("The temperature is "+ \
####                            "%.1f"%max(TEMPERATURE_ARRAY)+ \
####                            " degrees fahrenheit", "mtemp.mp3")
####                        play_sound(MAX_VOLUME, "mtemp.mp3")
####                    except:
####                        continue
##
#############################
### No Person Detected
#############################
### State 0: NOTHING - no heat source in view
###     Event 0: No change - outcome: continue waiting for a person
###     Event 1: One or more sensors cross the person threshold
###
##        elif (PERSON_STATE == STATE_NOTHING):
##            SENSITIVITY = 3
##            debug_print('STATE: NOTHING: No Person cnt: '+str(NO_PERSON_COUNT) \
##                       +' ROAM COUNT = '+str(ROAM_COUNT) \
##                       +' Hit Cnt = '+str(HIT_COUNT) \
##                       +' Prev Hit Cnt = '+str(PREVIOUS_HIT_COUNT) \
##                       +' sensitivity = '+str(SENSITIVITY) \
##                       +' p_detect_count = '+str(P_DETECT_COUNT) \
##                       +' Max temp in array: '+"%.1f"%max(TEMPERATURE_ARRAY) \
##                       +' Servo pos: '+str(SERVO_POSITION)
##                       +' CPU Temp: '+str(CPU_TEMP))
##            NO_PERSON_COUNT += 1
##            P_DETECT_COUNT = 0
##            BURN_HAZARD_CNT = 0
##            if MONITOR and PREV_PERSON_STATE != STATE_NOTHING:
##                SCREEN_DISPLAY.fill(name_to_rgb('white'), \
##                                    MESSAGE_AREA)
##                SCREEN_TEXT = FONT.render("Waiting...", 1, \
##                                   name_to_rgb('blue'))
##                SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
##                SCREEN_TEXT_POS.center = MESSAGE_AREA_XY
##                SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)
### update the screen
##                pygame.display.update()
##
### move servo to the next roam position
##            (ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
##             LAST_KNOWN_LED_POS, LIT_LED) = \
##            servo_roam(ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
##                       LAST_KNOWN_LED_POS, LIT_LED)
##
### check to see if there is a person there
##            P_DETECT, PERSON_POSITION = \
##                person_position_x_hit(HIT_ARRAY, SERVO_POSITION)
##
### if there is a person, go to the next state
##            if (P_DETECT and HIT_COUNT > 3):    # must have some hits > 1 to get next state
##                PERSON_STATE = STATE_POSSIBLE
##                if (PREV_PERSON_STATE == STATE_POSSIBLE):
##                    NOTHING_TO_POSSIBLE_COUNT += 1
##                    if (NOTHING_TO_POSSIBLE_COUNT > NTPC_LIMIT):
### reset the servo position if hits and we just came back from the next state
##                        NOTHING_TO_POSSIBLE_COUNT = 0
##                        debug_print('Jumping back and forth between Nothing and Possible. Resetting Servo')
##                        P_DETECT, PERSON_POSITION = \
##                            person_position_x_hit(HIT_ARRAY, SERVO_POSITION)
##                        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
##                            if SERVO_POSITION < MAX_SERVO_POSITION+50 or SERVO_POSITION >= MIN_SERVO_POSITION-50:
##                                PERSON_POSITION = CTR_SERVO_POSITION
##                            if SERVO_POSITION > PERSON_POSITION:    # fix roaming direction
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##                            else:
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                        else:
##                            if SERVO_POSITION <= MIN_SERVO_POSITION+50 or SERVO_POSITION >= MAX_SERVO_POSITION-50:
##                                PERSON_POSITION = CTR_SERVO_POSITION
##                            if SERVO_POSITION > PERSON_POSITION:    # fix roaming direction
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                            else:
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##
##                        SERVO_POSITION = \
##                            set_servo_to_position(PERSON_POSITION)
##
##                        if SERVO_POSITION == CTR_SERVO_POSITION:
### reverse the direction of roaming too
##                            if (SERVO_DIRECTION == SERVO_CUR_DIR_CCW):
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                            else:
##                                SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##
##                        PERSON_STATE = STATE_NOTHING
##                else:
##                    SERVO_POSITION = \
##                        move_head(PERSON_POSITION, \
##                                  SERVO_POSITION)
##            else:
### if no person detected, stay in this state
##                PERSON_STATE = STATE_NOTHING
##
##            STATE_POSSIBLE_COUNT = 0
##            STATE_LIKELY_COUNT = 0
##            STATE_PROBABLE_COUNT = 0
##            STATE_DETECTED_COUNT = 0
##            PREV_PERSON_STATE = STATE_NOTHING
##                
#############################
### Possible Person Detected
#############################
### State 1: Possible person in view - one or more sensors had a hit
###     Event 0: No hits - blip, move to State 0
###     Event 1: One hit - move head to try to center on the hit
###     Event 2: More than one hit - state 2
###
##        elif (PERSON_STATE == STATE_POSSIBLE):
##            SENSITIVITY = 2
##            debug_print('STATE POSSIBLE cnt: '+str(STATE_POSSIBLE_COUNT) \
##                       +' ROAM COUNT = '+str(ROAM_COUNT) \
##                       +' Hit Cnt = '+str(HIT_COUNT) \
##                       +' Prev Hit Cnt = '+str(PREVIOUS_HIT_COUNT) \
##                       +' sensitivity = '+str(SENSITIVITY) \
##                       +' p_detect_count = '+str(P_DETECT_COUNT) \
##                       +' Max temp in array: '+"%.1f"%max(TEMPERATURE_ARRAY) \
##                       +' Servo pos: '+str(SERVO_POSITION)
##                       +' CPU Temp: '+str(CPU_TEMP))
##            BURN_HAZARD_CNT = 0
##            STATE_POSSIBLE_COUNT += 1
##            NO_PERSON_COUNT += 1
##
##            P_DETECT, PERSON_POSITION = \
##                person_position_2_hit(HIT_ARRAY, SERVO_POSITION)
##            # stay in possible state
##            if (P_DETECT and HIT_COUNT > HIT_COUNT_LIMIT):
##                PERSON_STATE = STATE_PROBABLE
##                if (PREV_PERSON_STATE == STATE_PROBABLE):
##                    POSSIBLE_TO_PROBABLE_COUNT += 1
##                    if (POSSIBLE_TO_PROBABLE_COUNT > PTPC_LIMIT):
### reset the servo position if hits and we just came back from the next state
##                        POSSIBLE_TO_PROBABLE_COUNT = 0
##                        debug_print('Jumping back and forth between Possible and Probable. Resetting Servo')
##                        P_DETECT, PERSON_POSITION = \
##                            person_position_x_hit(HIT_ARRAY, SERVO_POSITION)
##                        SERVO_POSITION = \
##                            set_servo_to_position(PERSON_POSITION)
### reverse the direction of roaming too
##                        if (SERVO_DIRECTION == SERVO_CUR_DIR_CCW):
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                        else:
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##                        PERSON_STATE = STATE_NOTHING
##                else:
##                    SERVO_POSITION = \
##                        move_head(PERSON_POSITION, \
##                                  SERVO_POSITION)
### if no person detected, go to nothing
##            else:
##                if (SAID_HELLO == 1 and SAID_GOODBYE == 0):
##                    say_goodbye()
##                    SAID_GOODBYE = 1
##                    SAID_HELLO = 0
##                    
##                PERSON_STATE = STATE_NOTHING
##
##            PREV_PERSON_STATE = STATE_POSSIBLE
##            
###
### NOTE: Likely state has been removed to speed up detection.
###
#############################
### Likely Person Detected
#############################
### State 2: Likely person in view - more than one sensor had a hit
###     Event 0: No hits - blip, move to State 1
###     Event 1: One hit - noise, no change
###     Event 2: more than one sensor still has a hit, move head, State 3
###
##        elif (PERSON_STATE == STATE_LIKELY):
##            SENSITIVITY = 2
##            BURN_HAZARD_CNT = 0
##            STATE_LIKELY_COUNT += 1
##            debug_print('STATE: LIKELY cnt: '+str(STATE_LIKELY_COUNT) \
##                       +' ROAM COUNT = '+str(ROAM_COUNT) \
##                       +' Hit Cnt = '+str(HIT_COUNT) \
##                       +' Prev Hit Cnt = '+str(PREVIOUS_HIT_COUNT) \
##                       +' sensitivity = '+str(SENSITIVITY) \
##                       +' p_detect_count = '+str(P_DETECT_COUNT) \
##                       +' Max temp in array: '+"%.1f"%max(TEMPERATURE_ARRAY) \
##                       +' Servo pos: '+str(SERVO_POSITION)
##                       +' CPU Temp: '+str(CPU_TEMP))
##            NO_PERSON_COUNT += 1
##
##            P_DETECT, PERSON_POSITION = \
##                      person_position_x_hit(HIT_ARRAY, \
##                                            SERVO_POSITION)
##            if (P_DETECT and HIT_COUNT > HIT_COUNT_LIMIT):
##                PERSON_STATE = STATE_PROBABLE
##                if (PREV_PERSON_STATE == STATE_PROBABLE):
##                    LIKELY_TO_PROBABLE_COUNT += 1
##                    if (LIKELY_TO_PROBABLE_COUNT > LTPC_LIMIT):
### reset the servo position if hits and we just came back from the next state
##                        LIKELY_TO_PROBABLE_COUNT = 0
##                        debug_print('Jumping back and forth between Likely and Probable. Resetting Servo')
##                        P_DETECT, PERSON_POSITION = \
##                            person_position_x_hit(HIT_ARRAY, SERVO_POSITION)
##                        SERVO_POSITION = \
##                            set_servo_to_position(PERSON_POSITION)
### reverse the direction of roaming too
##                        if (SERVO_DIRECTION == SERVO_CUR_DIR_CCW):
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                        else:
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##                        PERSON_STATE = STATE_NOTHING
##                else:
##                    SERVO_POSITION = \
##                        move_head(PERSON_POSITION, \
##                                  SERVO_POSITION)
##            else:
##                PERSON_STATE = STATE_POSSIBLE
##                    
##            PREV_PERSON_STATE = STATE_LIKELY
##
#############################
### Probable Person Detected
#############################
### State 3: Probably a person in view
###     Event 0: No hits - noise, move to State 2
###     Event 1: One hit - noise, move to state 2
###     Event 2: more than one sensor has a hit, move head, say hello
###
##        elif (PERSON_STATE == STATE_PROBABLE):
##            SENSITIVITY = 2
##            BURN_HAZARD_CNT = 0
##            STATE_PROBABLE_COUNT += 1
##            debug_print('STATE: PROBABLE count: '+str(STATE_PROBABLE_COUNT) \
##                       +' ROAM COUNT = '+str(ROAM_COUNT) \
##                       +' Hit Cnt = '+str(HIT_COUNT) \
##                       +' Prev Hit Cnt = '+str(PREVIOUS_HIT_COUNT) \
##                       +' sensitivity = '+str(SENSITIVITY) \
##                       +' p_detect_count = '+str(P_DETECT_COUNT) \
##                       +' Max temp in array: '+"%.1f"%max(TEMPERATURE_ARRAY) \
##                       +' Servo pos: '+str(SERVO_POSITION)
##                       +' CPU Temp: '+str(CPU_TEMP))
##
##            P_DETECT, PERSON_POSITION = \
##                      person_position_x_hit(HIT_ARRAY, \
##                                            SERVO_POSITION)
##            if (P_DETECT and HIT_COUNT > HIT_COUNT_LIMIT):
##                detected_time_stamp = get_uptime()
##                debug_print('Person detected at '+str(detected_time_stamp))
##                PERSON_STATE = STATE_DETECTED
##                if (PREV_PERSON_STATE == STATE_DETECTED):
##                    PROBABLE_TO_DETECTED_COUNT += 1
##                    if (PROBABLE_TO_DETECTED_COUNT > PTDC_LIMIT):
### reset the servo position if hits and we just came back from the next state
##                        PROBABLE_TO_DETECTED_COUNT = 0
##                        debug_print('Jumping back and forth between Probable and Detected. Resetting Servo')
##                        P_DETECT, PERSON_POSITION = \
##                            person_position_x_hit(HIT_ARRAY, SERVO_POSITION)
##                        SERVO_POSITION = \
##                            set_servo_to_position(PERSON_POSITION)
### reverse the direction of roaming too
##                        if (SERVO_DIRECTION == SERVO_CUR_DIR_CCW):
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CW
##                        else:
##                            SERVO_DIRECTION = SERVO_CUR_DIR_CCW
##                        PERSON_STATE = STATE_NOTHING
##                else:
##                    SERVO_POSITION = \
##                        move_head(PERSON_POSITION, \
##                                  SERVO_POSITION)
#######################
### Say Hello!
#######################
##                    if (SAID_GOODBYE == 1 and SAID_HELLO == 0):
##                        say_hello()
##                        SAID_HELLO = 1
##                        SAID_GOODBYE = 0
##            else:
##                PERSON_STATE = STATE_POSSIBLE
##
##            PREV_PERSON_STATE = STATE_PROBABLE
##
#############################
### Person Detected !
#############################
### State 4: Person detected
###     Event 0: No hits - person left, say goodbye, move to state 0
###     Event 1: One hit - person left, say goodbye, move to state 1
###     Event 2: more than one sensor, move head to position, stay
###     
##        elif (PERSON_STATE == STATE_DETECTED):
##            SENSITIVITY = 3
##            debug_print('STATE: DETECTED count: '+str(STATE_DETECTED_COUNT) \
##                       +' ROAM COUNT = '+str(ROAM_COUNT) \
##                       +' Hit Cnt = '+str(HIT_COUNT) \
##                       +' Prev Hit Cnt = '+str(PREVIOUS_HIT_COUNT) \
##                       +' sensitivity = '+str(SENSITIVITY) \
##                       +' p_detect_count = '+str(P_DETECT_COUNT) \
##                       +' Max temp in array: '+"%.1f"%max(TEMPERATURE_ARRAY) \
##                       +' Servo pos: '+str(SERVO_POSITION)
##                       +' CPU Temp: '+str(CPU_TEMP))
##
##            BURN_HAZARD_CNT = 0
##            STATE_POSSIBLE_COUNT = 0
##            STATE_LIKELY_COUNT = 0
##            STATE_PROBABLE_COUNT = 0
##            STATE_DETECTED_COUNT += 1
##            ROAM_COUNT = 0
##            NO_PERSON_COUNT = 0
##            LED_STATE = True
##            GPIO.output(LED_GPIO_PIN, LED_STATE)
##            P_DETECT_COUNT += 1
##            CPU_TEMP = getCPUtemperature()
##
### every 20 minutes that a person is detected, have the bot remind
### the person to get some excersize.
##
##            uptime_now = get_uptime()
##            if (uptime_now - detected_time_stamp) >= EXERSIZE_TIMEOUT:
##                SAMPLED_AVERAGE_TEMP = numpy.mean(TEMPERATURE_ARRAY)
##                debug_print('SAMPLED_AVERAGE_TEMP = '+str(SAMPLED_AVERAGE_TEMP))
##                play_sound(MAX_VOLUME, STRETCH_FILE_NAME)
##                detected_time_stamp = uptime_now    # reset
##            P_DETECT, PERSON_POSITION = \
##                      person_position_2_hit(HIT_ARRAY, \
##                                            SERVO_POSITION)
##            if (P_DETECT and HIT_COUNT > HIT_COUNT_LIMIT):
##                SERVO_POSITION = move_head(PERSON_POSITION, \
##                                           SERVO_POSITION)
##                PERSON_STATE = STATE_DETECTED
##            else:
##                PERSON_STATE = STATE_PROBABLE
##
##            PREV_PERSON_STATE = STATE_DETECTED
##
#############################
### Invalid state
#############################
##        else:
##            PERSON_STATE = STATE_NOTHING
##            (ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
##             LAST_KNOWN_LED_POS, LIT_LED) = \
##            servo_roam(ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
##                       LAST_KNOWN_LED_POS, LIT_LED)
                       
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
    # do not close the logfile here
    # allows the previous logfile to stay intact for a forensic analysis
##    GPIO.output(LED0_GRN, LED_OFF)
##    GPIO.output(LED1_GRN, LED_OFF)
##    GPIO.output(LED2_GRN, LED_OFF)
##    GPIO.output(LED3_GRN, LED_OFF)
##    GPIO.output(LED0_YEL, LED_OFF)
##    GPIO.output(LED1_YEL, LED_OFF)
##    GPIO.output(LED2_YEL, LED_OFF)
##    GPIO.output(LED3_YEL, LED_OFF)
##    for g in range(0, 9):
##        GPIO.output(LED0_RED, LED_ON)
##        GPIO.output(LED1_RED, LED_ON)
##        GPIO.output(LED2_RED, LED_ON)
##        GPIO.output(LED3_RED, LED_ON)
##        time.sleep(0.3)
##        GPIO.output(LED0_RED, LED_OFF)
##        GPIO.output(LED1_RED, LED_OFF)
##        GPIO.output(LED2_RED, LED_OFF)
##        GPIO.output(LED3_RED, LED_OFF)
##        time.sleep(0.3)
    GPIO.cleanup()

