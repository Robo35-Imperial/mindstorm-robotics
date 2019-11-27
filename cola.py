import numpy as np
import math
import sys
from collections import namedtuple
from waypoint_sonar import navigateToWaypoint, estimatePosition, turn, turnSensor, getSonarMeasurement, BP, draw, drawMap

Pos = namedtuple('Pos', ['x', 'y', 'theta'])
LEFT = 1
RIGHT = 2
BOTTOM = 3


class OccupancyMap:

    def __init__(self, start, top_left, shape, orientation):
        self.start = start
        self.top_left = top_left
        self.map = np.full(shape, 0.0)
        self.closeU = -0.3
        self.depthU = 10
        self.orientation = orientation

    def update(self, pos, theta, depth):
        processed_squares = set()
        for t in range(theta-15, theta+15, 1):
            for r in range(0, depth+1, 1):
                if self.orientation == LEFT:
                    localPos = Pos(x=abs(self.top_left.x - pos.x), y=abs(self.top_left.y - pos.y), theta=pos.theta)
                    x = math.floor(r * math.cos(math.radians(t)) + localPos.x)
                    y = math.floor(r * math.sin(math.radians(t)) + localPos.y)
                if self.orientation == RIGHT:
                    localPos = Pos(x=abs(self.top_left.x - pos.x), y=abs(self.top_left.y - pos.y), theta=180 - pos.theta)
                    x = math.floor(-r * math.cos(math.radians(t)) + localPos.x)
                    y = math.floor(r * math.sin(math.radians(t)) + localPos.y)
                if self.orientation == BOTTOM:
                    localPos = Pos(x=abs(self.top_left.x - pos.x), y=abs(self.top_left.y - pos.y), theta=90 - pos.theta)
                    y = math.floor(-r * math.cos(math.radians(t)) + localPos.y)
                    x = math.floor(r * math.sin(math.radians(t)) + localPos.x)

                if ((x >= 0 and x < self.map.shape[0]) and (y >= 0 and y < self.map.shape[1])) and (x,y) not in processed_squares:
                    processed_squares.add((x,y))
                    if r == depth:
                        self.map[x][y] += self.depthU
                    else:
                        self.map[x][y] += self.closeU


    def getMostLikelyBottlePosition(self):
        mostLikelyProb = self.map[0][0]
        mostLikelyPos = Pos(0,0,0)
        for x in range(self.map.shape[0]):
            for y in range(self.map.shape[1]):
                if self.map[x][y] > mostLikelyProb:
                    mostLikelyProb = self.map[x][y]
                    mostLikelyPos = Pos(x,y,0)

        print(mostLikelyProb, mostLikelyPos)
        return (self.top_left.x + mostLikelyPos.x, self.top_left.y - mostLikelyPos.y)

occupancyMap1 = OccupancyMap(Pos(126, 42, 0), Pos(126, 64, 0), (60,44), LEFT)
occupancyMap2 = OccupancyMap(Pos(126, 84, 90), Pos(104, 190, 0), (44,106), BOTTOM)
occupancyMap3 = OccupancyMap(Pos(64, 88, 180), Pos(20, 148, 0), (44,106), RIGHT)

def scanAndUpdate(pos, occupancyMap):
    # Go through -90 to 90 in 10 degree steps
    turnSensor(-90, 120)

    step = 20
    for theta in range(-90, 90, step):
        print("SCANNING: ", theta)
        z = getSonarMeasurement()
        occupancyMap.update(pos, theta, z)
        turnSensor(step, 25)

    z = getSonarMeasurement()
    occupancyMap.update(pos, theta, z)
    
    turnSensor(-90, 120)

def navigateToMap(occupancyMap):
    
    # Go to the start of the area
    if navigateToWaypoint(occupancyMap.start.x, occupancyMap.start.y):
        return True
    
    eX, eY, eTheta = estimatePosition()
    print("NAVIGATING END: I'm at {}, {} and facing {}! ".format(eX, eY, eTheta))

                
    # Turn to face the area properly and move sonar to correct initial pos
    print("IM ON THE LEFT MAP, turning {} and eTheta is {}".format(180-eTheta, eTheta))
    turn(occupancyMap.start.theta-eTheta)

    # if occupancyMap.orientation == LEFT:
    #     print("IM ON THE RIGHT MAP")
    #     turn(-eTheta)
    # if occupancyMap.orientation == RIGHT:
    #     turn(180-eTheta)
    # if occupancyMap.orientation == BOTTOM:
    #     print("IM ON THE TOP MAP")
    #     turn(90-eTheta)

    eX, eY, eTheta = estimatePosition()
    print("ENDED AT: I'm at {}, {} and facing {}! ".format(eX, eY, eTheta))

    return False

def hitBottle(occupancyMap):
    distanceFromBottle = float('inf')
    while distanceFromBottle > 20:

        # Scan and update occupancy map
        eX, eY, eTheta = estimatePosition()
        print("HITTING BOTTLE: I'm at {}, {} and facing {}! ".format(eX, eY, eTheta))
        scanAndUpdate(Pos(x=eX, y=eY, theta=eTheta), occupancyMap)

        # Move closer to most likely position of bottle 
        mostLikelyPos = occupancyMap.getMostLikelyBottlePosition()
        print(mostLikelyPos)

        distanceToMoveX = (mostLikelyPos[0] - eX)
        distanceToMoveY = (mostLikelyPos[1] - eY)
        distanceFromBottle = (distanceToMoveX**2 + distanceToMoveY**2)**0.5

        hit = navigateToWaypoint(eX + distanceToMoveX/2, eY + distanceToMoveY/2, 100)
        if hit: return

    navigateToWaypoint(eX + distanceToMoveX, distanceToMoveY, 100)


def main():
    maps = [occupancyMap2, occupancyMap1, occupancyMap3]

    for m in maps:
        drawMap(getWalls(m))

    for m in maps:
        if navigateToMap(m): continue # If the bot hits the bottle on the way to the waypoint then skip
        hitBottle(m)

def getWalls(o):
    X = o.map.shape[0]
    Y = o.map.shape[1]

    top = ((o.top_left.x, o.top_left.y), (o.top_left.x + X, o.top_left.y))
    right = ((o.top_left.x + X, o.top_left.y), (o.top_left.x + X, o.top_left.y - Y))
    bottom = ((o.top_left.x + X, o.top_left.y - Y), (o.top_left.x, o.top_left.y - Y))
    left = ((o.top_left.x, o.top_left.y - Y), (o.top_left.x, o.top_left.y))
    
    return (top, left, right, bottom)
    # top = [(o.top_left.x, o.top_left.y, 1, 0), (o.top_left.x + X, o.top_left.y, 1, 0), (o.top_left.x + X, o.top_left.y + Y, 1, 0), (o.top_left.x, o.top_left.y + Y, 1, 0)]

    # return top


if __name__ == "__main__":
    try:
        drawMap()
        main()
    except KeyboardInterrupt:
        BP.reset_all()
    finally:
        BP.reset_all()
    
