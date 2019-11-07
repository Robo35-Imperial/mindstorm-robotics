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

import random
import numpy as np
import math

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
STATUS = 0
POWER = 1
POSITION = 2
VELOCITY = 3

NUM_PARTICLES = 100
E = (0, 0.1)
F = (0, 1)
G = (0, 1)

start = (0, 0, 0, 1 / NUM_PARTICLES)
particles = [start] * NUM_PARTICLES

def move(cms):
    print("Distance:", cms)
    pos_per_cm = 664 / 40
    start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]

    while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos< pos_per_cm * cms:
    #    print(BP.get_motor_status(BP.PORT_A)[POSITION])
        BP.set_motor_dps(BP.PORT_A, 250)
        BP.set_motor_dps(BP.PORT_C, 250)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def turn(degrees):
    print("Turning:", degrees)
    pos_per_degrees = 254 / 90

    if degrees >= 0:
        start_pos = BP.get_motor_status(BP.PORT_C)[POSITION]
        while BP.get_motor_status(BP.PORT_C)[POSITION] - start_pos < pos_per_degrees * degrees:
            BP.set_motor_dps(BP.PORT_A, -150)
            BP.set_motor_dps(BP.PORT_C, 150)
    else:
        start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]
        while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos < pos_per_degrees * degrees:
            BP.set_motor_dps(BP.PORT_A, 150)
            BP.set_motor_dps(BP.PORT_C, -150)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def updateParticlesStraightLine(d):
    global particles
    new_particles = []
    for p in particles:
        e = random.gauss(*E)
        f = random.gauss(*F)
        new_particles.append((
            p[0] + (d + e) * math.cos(math.radians(p[2])),
            p[1] + (d + e) * math.sin(math.radians(p[2])),
            p[2] + f,
            p[3]
        ))
        
    particles = new_particles

def updateParticlesTurn(a):
    global particles
    new_particles = []
    for p in particles:
        g = random.gauss(*G)
        new_particles.append((
            p[0],
            p[1],
            p[2] + a + g,
            p[3]
        ))
        
    particles = new_particles

def estimatePosition():
    np_particles = np.array(particles)
    mean_x = np.dot(np_particles[:,0], np_particles[:,3])
    mean_y = np.dot(np_particles[:,1], np_particles[:,3])
    mean_theta = np.dot(np_particles[:,2], np_particles[:,3])

    return (mean_x, mean_y, mean_theta)

def navigateToWaypoint(x, y):
    (mean_x, mean_y, mean_theta) = estimatePosition()

    print((mean_x, mean_y, mean_theta))

    dx, dy = x * 100 - mean_x, y * 100 - mean_y

    alpha = math.degrees(math.atan2(dy, dx))
    beta = alpha - mean_theta

    if (beta > 180):
        beta -= 360
    elif (beta <= -180):
        beta += 360

    turn(beta)
    updateParticlesTurn(beta)

    d = math.sqrt(dx ** 2 + dy ** 2)
    move(d)
    updateParticlesStraightLine(d)

try:
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
    except IOError as error:
        print(error)
    
    while True:
        x = float(input("Please enter a value for x: "))
        y = float(input("Please enter a value for y: "))
        navigateToWaypoint(x, y)
    #    navigateToWaypoint(0.4, 0)
    #    navigateToWaypoint(0.4, 0.4)
    #    navigateToWaypoint(0, 0.4)
    #    navigateToWaypoint(0, 0)

    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
