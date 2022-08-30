# from typing import final
# from audioop import mul
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from spike import PrimeHub, MotionSensor, ColorSensor, Motor, MotorPair
import time
from math import *

hub = PrimeHub()

motorC = Motor("C")
motorE = Motor("E")

motors = MotorPair('C', 'E')

motorD = Motor("D")
motorF = Motor("F")
_CM_PER_INCH = 2.54

def initialize():
    motors.set_stop_action("hold")
    motors.set_motor_rotation(27.6, 'cm')

#Utilities
def turn(angle = 0, leftSpeed = 0, rightSpeed = 0):
    """
    Turn Information:
    - Purpose - Turn turns the drivebase to the specified angle
    - Parameters: Angle(Int), leftSpeed(Int), rightSpeed(Int)
    - Issues: Off by ~5 degrees
    - Optimal Speed: 25
    """

    '''this goes on the start of the program'''
    #motors.set_stop_action("hold")
    #motors.set_motor_rotation(27.6, 'cm')

    hub.motion_sensor.reset_yaw_angle()
    startAngle = hub.motion_sensor.get_yaw_angle()
    currentAngle = hub.motion_sensor.get_yaw_angle()
    motors.start_tank_at_power(leftSpeed, rightSpeed)
    while abs(hub.motion_sensor.get_yaw_angle()) - abs(startAngle) + 15 < abs(angle):
        currentAngle = hub.motion_sensor.get_yaw_angle()
        print("Distance turned: " + str(abs(currentAngle) - abs(startAngle)))
    motors.stop()

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

def goF(distancein, speed,):
    motors.move_tank(distancein, 'in', speed, speed)

def energyStorage():
    goF(20, 30)
    turn(95, 25, -25)
    goF(6, 30)
    turn(95, -25, 25)
    goF(10, 30)
    moveArm(1000, 30, motorD)

#energyStorage()

def turnToAngle(targetAngle = 0, speed = 25, forceTurn = "None", slowTurnRatio = 0.4):
    # Initialize function
    motors.stop()
    currentAngle = hub.motion_sensor.get_yaw_angle()
    print("TurnToAngle current angle is " + str(currentAngle) + " and target angle is" + str(targetAngle))

    # Make the target and current angles positive if they are not already
    if currentAngle < 0:
        currentAngle = currentAngle + 360

    if targetAngle < 0:
        targetAngle = targetAngle + 360

    # Detect which way is faster to turn
    degreesToTurnRight = 0
    degreesToTurnLeft = 0
    degreesToTurn = 0
    
    if targetAngle > currentAngle:
        degreesToTurnRight = targetAngle - currentAngle
        degreesToTurnLeft = 360 - currentAngle + targetAngle

    else:
        degreesToTurnRight = 360 - currentAngle + targetAngle
        degreesToTurnLeft = currentAngle - targetAngle

    if forceTurn == "None":
        if degreesToTurnLeft < degreesToTurnRight:
            degreesToTurn = degreesToTurnLeft * -1
        else:
            degreesToTurn = degreesToTurnRight
        
    elif forceTurn == "Right":
        degreesToTurn = degreesToTurnRight
    
    elif forceTurn == "Left":
        degreesToTurn = degreesToTurnLeft * -1

    print("TurnToAngle degrees to turn is " + str(degreesToTurn))

    # Call the internal function that actually turns the robot
    _turnRobot(degreesToTurn, speed, slowTurnRatio)

def _turnRobot(angleInDegrees, speed, slowTurnRatio):
    # Initialize the function
    motors.stop()
    slowTurnDegrees = slowTurnRatio * abs(angleInDegrees)
    initialAngle = hub.motion_sensor.get_yaw_angle()
    finalAngle = initialAngle + angleInDegrees
    print("TurnToAngle initial gyro angle is " + str(initialAngle) + " and target angle is " + str(finalAngle))
    initialTurn = 0

    # Calculate the initial turn distance
    if angleInDegrees > 0:
        initialTurn = angleInDegrees - slowTurnDegrees
    else:
        initialTurn = angleInDegrees + slowTurnDegrees

    # Turn the initial amount
    motors.move_tank(initialTurn, "degrees", speed, -1 * speed)
    motors.stop()

    # Initialize slow turn
    initialSlowSpeed = 5

    robotAngle = hub.motion_sensor.get_yaw_angle()
    print("TurnToAngle gyro angle before slow turn is " + str(robotAngle))

    # Turn slowly
    while abs(robotAngle - finalAngle) > 1:
        if robotAngle > finalAngle:
            motors.start_tank(initialSlowSpeed * -1, initialSlowSpeed)
        else:
            motors.start_tank(initialSlowSpeed, initialSlowSpeed * -1)
        robotAngle = hub.motion_sensor.get_yaw_angle()
        

    motors.stop()

    print("TurnToAngle final robot angle is " + str(hub.motion_sensor.get_yaw_angle()))

def testTurnToAngle():
    # TurnToAngle Testing
    hub.motion_sensor.reset_yaw_angle()
    multiplier = 1
    print("____________________________________________")
    start = time.ticks_us()
    while multiplier < 5:
        turnToAngle(90 * multiplier, 25)
        multiplier = multiplier + 1
        time.sleep(1)
    time.sleep(1)        
    print("Final Angle is " + str(hub.motion_sensor.get_yaw_angle()))
    end = time.ticks_us()
    print("Total time = " + str(end - start))



def gyroStraight(targetAngle, distance, speed = 20, backward = False):
    wheelDiameter = 8.8 # could be 8.8
    degreesToCover = (distance * 360)/(wheelDiameter * 3.14)
    #print(degreesToCover)
    position_start = motorE.get_degrees_counted();
    #print("Position Start = " + str(position_start))
    if (backward): 
        while ((motorE.get_degrees_counted() - position_start)  > degreesToCover * -1  ):
            #print("degrees = " + str(right_large_motor.get_degrees_counted() - position_start))
            currentAngle = hub.motion_sensor.get_yaw_angle()
            correction = targetAngle - currentAngle
            motors.start(correction, speed * -1  )
    else:
         while ((motorE.get_degrees_counted() - position_start)  < degreesToCover  ):
           # print("degrees = " + str(right_large_motor.get_degrees_counted() - position_start))
            currentAngle = hub.motion_sensor.get_yaw_angle()
            correction = targetAngle - currentAngle
            motors.start(correction, speed)

    motors.stop()

def testGyro():
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*16)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward =True)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward=True)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3, backward=True)


initialize()
testTurnToAngle()
