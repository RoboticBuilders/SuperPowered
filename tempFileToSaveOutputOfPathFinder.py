from utilities import *

def Run6():
    # Code automatically generated by PathFinder.
    # Download the code below to the Robot and run it.

    # Drive from (20,10) to mission:HydroDam at ( 52, 45)
    turnToAngle(targetAngle=-47, speed=50)
    gyroStraight(distance=47, speed=50, backward=False, targetAngle=-47)

    # Code to get past mission: HydroDam
    turnToAngle(targetAngle=0, speed=50)
    gyroStraight(distance=44, speed=50, backward=False, targetAngle=0)

    # Drive from (97.0,45.0) to (97,94)
    turnToAngle(targetAngle=-90, speed=50)
    gyroStraight(distance=49, speed=50, backward=False, targetAngle=-90)

    # Drive from (97,94) to (80,64)
    turnToAngle(targetAngle=120, speed=50)
    gyroStraight(distance=34, speed=50, backward=False, targetAngle=120)

