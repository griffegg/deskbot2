#! /usr/bin/python
"""
Created 7/5/15 by Greg Griffes based on the Quick Reaction game
published on the Raspberry Pi Learning Resources pages at
www.raspberrypi.org
"""

import RPi.GPIO as GPIO     # GPIO is the handle to control pins
import time                 # for time delays
import random               # for random delays
import webcolors            # for converting a color name to RGB

class eye_led(object):

    def __init__(self, red_gpio_pin, red_level, green_gpio_pin, green_level, blue_gpio_pin, blue_level):

        self.R = red_gpio_pin
        self.G = green_gpio_pin
        self.B = blue_gpio_pin
        self.level = 100    # 100 = off, 0 = full on

        self.R_level = red_level
        self.G_level = green_level
        self.B_level = blue_level

        self.ON = 0
        self.OFF = 1

        GPIO.setmode(GPIO.BCM)  # set GPIO to use BCM pin numbers
        GPIO.setwarnings(False) # warnings off

#        print ('red gpio = '+str(self.R))
        GPIO.setup(self.R, GPIO.OUT)
        GPIO.setup(self.G, GPIO.OUT)
        GPIO.setup(self.B, GPIO.OUT)

        self.R_lvl = GPIO.PWM(self.R, 100)
        self.R_lvl.start(100)
        self.G_lvl = GPIO.PWM(self.G, 100)
        self.G_lvl.start(100)
        self.B_lvl = GPIO.PWM(self.B, 100)
        self.B_lvl.start(100)

##        level = 100
##        while level > 0:
###            print ('level = '+str(level))
##            self.R_lvl.ChangeDutyCycle(level)
##            level -= 2
##            time.sleep(0.05)
##        self.R_lvl.ChangeDutyCycle(100)
##
##        level = 100
##        while level > 0:
###            print ('level = '+str(level))
##            self.G_lvl.ChangeDutyCycle(level)
##            level -= 2
##            time.sleep(0.05)
##        self.G_lvl.ChangeDutyCycle(100)
##
##        level = 100
##        while level > 0:
###            print ('level = '+str(level))
##            self.B_lvl.ChangeDutyCycle(level)
##            level -= 2
##            time.sleep(0.05)
##        self.B_lvl.ChangeDutyCycle(100)

        self.R_lvl.ChangeDutyCycle(self.R_level)
        self.G_lvl.ChangeDutyCycle(self.G_level)
        self.B_lvl.ChangeDutyCycle(self.B_level)

    def __del__(self):
        GPIO.cleanup()          # reset GPIOs

    def color(self, color_name):
        (self.colorR, self.colorG, self.colorB) = webcolors.name_to_rgb(color_name)

        self.R_lvl.ChangeDutyCycle(self.convert_to_pwm(self.colorR))
        self.G_lvl.ChangeDutyCycle(self.convert_to_pwm(self.colorG))
        self.B_lvl.ChangeDutyCycle(self.convert_to_pwm(self.colorB))

    def convert_to_pwm(self, rgb_value):
        return 100-(float(100*rgb_value/255))

    def off(self):
        self.R_lvl.ChangeDutyCycle(100)
        self.G_lvl.ChangeDutyCycle(100)
        self.B_lvl.ChangeDutyCycle(100)
    
    def r(self, brightness):
        self.R_lvl.ChangeDutyCycle(brightness)
    
    def g(self, brightness):
        self.G_lvl.ChangeDutyCycle(brightness)
    
    def b(self, brightness):
        self.B_lvl.ChangeDutyCycle(brightness)
    
if __name__ == '__main__':  #test code
    left_eye = eye_led(13, 100, 17, 100, 6, 100)
    right_eye = eye_led(12, 100, 16, 100, 5, 100)
    time.sleep(0.5)

    right_eye.off()
    left_eye.off()
    right_eye.r(50)
    left_eye.r(50)
    time.sleep(1)

    right_eye.off()
    left_eye.off()
    right_eye.g(50)
    left_eye.g(50)
    time.sleep(1)

    right_eye.off()
    left_eye.off()
    right_eye.b(50)
    left_eye.b(50)
    time.sleep(1)

    right_eye.color("teal")
    left_eye.color("teal")
    time.sleep(1)

    right_eye.color("fuchsia")
    left_eye.color("fuchsia")
    time.sleep(1)

    right_eye.color("goldenrod")
    left_eye.color("goldenrod")
    time.sleep(1)

    right_eye.color("lime")
    left_eye.color("lime")
    time.sleep(1)

    right_eye.color("Chocolate")
    left_eye.color("Chocolate")
    time.sleep(1)

    right_eye.color("darkmagenta")
    left_eye.color("darkmagenta")
    time.sleep(1)

    right_eye.off()
    left_eye.off()
