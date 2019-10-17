#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
# 
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports A and D. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor A will run to match the position of motor D. Manually rotate motor D, and motor A will follow.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
STATUS = 0
POWER = 1
POSITION = 2
VELOCITY = 3

def move(cms):
    pos_per_cm = 654 / 40
    start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]

    while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos< pos_per_cm * cms:
        print(BP.get_motor_status(BP.PORT_A)[POSITION])
        BP.set_motor_dps(BP.PORT_A, 50)
        BP.set_motor_dps(BP.PORT_D, 50)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def turn(degrees):
    pos_per_degrees = 170 / 90
    start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]

    while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos< pos_per_degrees * degrees:
        print(BP.get_motor_status(BP.PORT_A))
        BP.set_motor_dps(BP.PORT_A, 50)
        BP.set_motor_dps(BP.PORT_D, -50)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

try:
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
    except IOError as error:
        print(error)

    for i in range(4):
        move(40)
        turn(90)

    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

    """
    while True:
        # Each of the following BP.get_motor_encoder functions returns the encoder value.
        try:
            target = BP.get_motor_encoder(BP.PORT_D) # read motor D's position
        except IOError as error:
            print(error)
        
        BP.set_motor_position(BP.PORT_A, target)    # set motor A's target position to the current position of motor D
        print(("Motor A Target Degrees Per Second: %d" % target), "  Motor A Status: ", BP.get_motor_status(BP.PORT_A))
        
        try:
            print("Motor A target: %6d  Motor A position: %6d" % (target, BP.get_motor_encoder(BP.PORT_A)))
        except IOError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
    """

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.