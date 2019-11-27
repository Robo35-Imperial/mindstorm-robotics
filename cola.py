import numpy as np
import math
import sys
from collections import namedtuple
from waypoint_sonar import navigateToWaypoint, estimatePosition, turn, turnSensor, getSonarMeasurement, BP

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
                if orientation == LEFT:
                     localPos = Pos(x=abs(self.top_left.x - pos.x), y=abs(self.top_left.y - pos.y), theta=pos.theta)
                    x = math.floor(r * math.cos(math.radians(t)) + localPos.x)
                    y = math.floor(r * math.sin(math.radians(t)) + localPos.y)
                if orientation == RIGHT:
                     localPos = Pos(x=abs(self.top_left.x - pos.x), y=abs(self.top_left.y - pos.y), theta=180 - pos.theta)
                    x = math.floor(-r * math.cos(math.radians(t)) + localPos.x)
                    y = math.floor(r * math.sin(math.radians(t)) + localPos.y)
                if orientation == BOTTOM:
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
    eX, eY, eTheta = estimatePosition()
    

    # Go to the start of the area
    navigateToWaypoint(occupancyMap.start.x, occupancyMap.start.y)

    # Turn to face the area properly and move sonar to correct initial pos
    if occupancyMap.orientation == LEFT:
        turn(-eTheta)
    if occupancyMap.orientation == RIGHT:
        turn(eTheta-180)
    if occupancyMap.orientation == BOTTOM:
        turn(eTheta-90)

def hitBottle(occupancyMap):
    distanceFromBottle = float('inf')
    while distanceFromBottle > 20:

        # Scan and update occupancy map
        eX, eY, eTheta = estimatePosition()
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
    occupancyMap1 = OccupancyMap(start=Pos(126, 42, 0), top_left=Pos(126, 80, 0), (76,76), LEFT)
    occupancyMap2 = OccupancyMap(start=Pos(126, 88, 0), top_left=Pos(88, 206, 0), (76,118), BOTTOM)
    occupancyMap3 = OccupancyMap(start=Pos(80, 88, 0), top_left=Pos(4, 164, 0), (76,118), RIGHT)
    maps = [occupancyMap2, occupancyMap1, occupancyMap3]
    for m in maps:
        navigateToMap(m)
        hitBottle(m)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        BP.reset_all()
    finally:
        BP.reset_all()
    


