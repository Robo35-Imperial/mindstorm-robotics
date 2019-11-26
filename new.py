from waypoint_sonar import turnSensor, BP
import time

for i in range(10):
    x = 1
    if i % 2 == 0:
        x = -1
    turnSensor(x*90, 60)
    time.sleep(0.1)
BP.reset_all()

