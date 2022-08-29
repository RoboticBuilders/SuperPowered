from spike import PrimeHub, MotionSensor, ColorSensor, Motor, MotorPair
import time
from math import *

hub = PrimeHub()
motors = MotorPair('C', 'E')

motorD = Motor("D")
motorF = Motor("F")

def moveArm(degrees = 0, speed = 0, motor = motorD):
    """
    MoveArm Information:
    - Purpose - MoveArm turns the specified amount of degrees
    - Parameters: Degrees(Int), Speed(Int), Motor(motorD or motorF)
    - Issues: None known

    """
    startDegrees = motor.get_degrees_counted()
    currentDegrees = motor.get_degrees_counted()
    motor.start_at_power(speed)
    while abs(currentDegrees) - abs(startDegrees) < abs(degrees):
        currentDegrees = motor.get_degrees_counted()
        print("MotorD smaller: " + str(abs(currentDegrees) - abs(startDegrees)))
    motor.stop()

moveArm(45, -25, motorD)
