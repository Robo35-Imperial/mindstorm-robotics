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
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

STATUS = 0
POWER = 1
POSITION = 2
VELOCITY = 3

NUM_PARTICLES = 100
E = (0, 0.1)
F = (0, 1)
G = (0, 1)

MAP_POINTS = (
    (0, 0), 
    (0, 168), 
    (84, 168),
    (84, 126),
    (84, 210),
    (168, 210),
    (168, 84),
    (210, 84),
    (210, 0)
)

MAP_WALLS = (
    (MAP_POINTS[0], MAP_POINTS[1]),
    (MAP_POINTS[1], MAP_POINTS[2]),
    (MAP_POINTS[2], MAP_POINTS[3]),
    (MAP_POINTS[3], MAP_POINTS[4]),
    (MAP_POINTS[4], MAP_POINTS[5]),
    (MAP_POINTS[5], MAP_POINTS[6]),
    (MAP_POINTS[6], MAP_POINTS[7]),
    (MAP_POINTS[7], MAP_POINTS[8]),
    (MAP_POINTS[8], MAP_POINTS[0])
)

LIKELIHOOD_SD = 2.5
LIKELIHOOD_CONST = 0

RESAMPLE_FREQ = 0.2

start = (0, 0, 0, 1 / NUM_PARTICLES)
particles = [start] * NUM_PARTICLES

def move(cms):
    print("Distance:", cms)
    pos_per_cm = 664 / 40
    start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]

    while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos< pos_per_cm * cms:
    #    print(BP.get_motor_status(BP.PORT_A)[POSITION])
        BP.set_motor_dps(BP.PORT_A, 250)
        BP.set_motor_dps(BP.PORT_D, 250)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
    
    updateParticlesStraightLine(cms)

def turn(degrees):
    if (degrees > 180):
        degrees -= 360
    elif (degrees <= -180):
        degrees += 360
    

    print("Turning:", degrees)
    pos_per_degrees = 254 / 90

    if degrees >= 0:
        start_pos = BP.get_motor_status(BP.PORT_A)[POSITION]
        while BP.get_motor_status(BP.PORT_A)[POSITION] - start_pos < pos_per_degrees * degrees:
            BP.set_motor_dps(BP.PORT_A, 150)
            BP.set_motor_dps(BP.PORT_D, -150)
    else:
        start_pos = BP.get_motor_status(BP.PORT_D)[POSITION]
        while BP.get_motor_status(BP.PORT_D)[POSITION] - start_pos < - pos_per_degrees * degrees:
            BP.set_motor_dps(BP.PORT_A, -150)
            BP.set_motor_dps(BP.PORT_D, 150)
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

    updateParticlesTurn(degrees)

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
    mcl()

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
    mcl()

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
    d = math.sqrt(dx ** 2 + dy ** 2)

    while d > 0.2:
        alpha = math.degrees(math.atan2(dy, dx))
        beta = alpha - mean_theta

        turn(beta)

        move(20)
        
        (mean_x, mean_y, mean_theta) = estimatePosition()

        dx, dy = x * 100 - mean_x, y * 100 - mean_y
        d = math.sqrt(dx ** 2 + dy ** 2)

    alpha = math.degrees(math.atan2(dy, dx))
    beta = alpha - mean_theta
    turn(beta)
    move(d)


def whichWall(x, y, theta):
    closest_wall = None
    closest_dist = float('Inf')
    
    for wall in MAP_WALLS:
        ax = wall[0][0]
        ay = wall[0][1]
        bx = wall[1][0]
        by = wall[1][1]

        m = ((by - ay) * (ax - x) - (bx - ax) * (ay - y)) 
            / ((by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta))

        mx = x + m * math.cos(theta)
        my = y + m * math.sin(theta)

        wallLength = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

        maLength = math.sqrt((ax - mx) ** 2 + (ay - my) ** 2)
        mbLength = math.sqrt((bx - mx) ** 2 + (by - my) ** 2)

        if (maLength <= wallLength && mbLength <= wallLength):
            if (closest_dist < m):
                closest_wall = wall
                closest_dist = m

    return closest_wall, m

def calculateLikelihood(x, y, theta, z):
    (wall, dist) = whichWall(x, y, theta)

    sample = math.exp((z - dist) ** 2 / (2 * LIKELIHOOD_SD ** 2)) + LIKELIHOOD_CONST

    return sample

def normaliseWeights(z):
    global particles

    weights = [calculateLikelihood(x, y, theta, z) for (x, y, theta, _) in particles]
    weights_sum = sum(weights)

    new_particles = []

    for p, w in zip(particles, weights):
        new_particles.append((
            p[0],
            p[1],
            p[2],
            w / weights_sum
        ))

    particles = new_particles

def resampleWeights():
    global particles

    new_particles = []
    for _ in range(len(particles)):
        rand = random.random()
        cumulative_weight = 0
        curr_weight_index = 0
        while cumulative_weight < rand:
            curr_weight_index += 1
            cumulative_weight += particles[curr_weight_index][3]

        new_particles.append(particles[curr_weight_index])

    particles = new_particles

def mcl():
    try:
        z = BP.get_sensor(BP.PORT_1)
        print(z)
    except brickpi3.SensorError as error:
        print(error)
    
    normaliseWeights(z)
    resampleWeights()

"""
try:
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
    except IOError as error:
        print(error)
    
    navigateToWaypoint(0.4, 0)
    navigateToWaypoint(0.4, 0.4)
    navigateToWaypoint(0, 0.4)
    navigateToWaypoint(0, 0)

    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
"""

try:
    while True:
        x = float(input("X coord: "))
        y = float(input("Y coord: "))

        navigateToWaypoint(x, y)

        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
