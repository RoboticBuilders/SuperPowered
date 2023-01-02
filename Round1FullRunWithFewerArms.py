# LEGO type:standard slot:0
# This is meant to be round1, in this run we go to left home from 
# the power plant. This means we give up on one unit. 
# The idea is that we save time by doing this and thus enabling us
# to finish the run.
from spike import PrimeHub, ColorSensor,  Motor, MotorPair
from math import *
import collections
# Note that the "hub" import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from spike.operator import *
from spike.control import wait_for_seconds
from spike.control import *
import gc
import math
import random

# Various robot constants
AXLE_DIAMETER_CM = 12.7
AXLE_DIAMETER_CM_CORRECTED = 12.2
WHEEL_RADIUS_CM = 4.4
GLOBAL_LEVEL = 0
ANYA_RUN_START_OFFSET_TO_MAT_WEST = 0
TOTAL_DEGREES_TURNED = 0
LAST_TURN_LEFT = False

# Which Marvin is this.
# Amogh: A
# Rishabh-Nami: RN
# Anya:Arisha: AA
ROBOT = "A" 

primeHub = PrimeHub()

# Left large motor
motorC = Motor("C")
left_large_motor = motorC
# Right large motor
motorE = Motor("E")
right_large_motor = motorE

# The motor pair
wheels = MotorPair('C', 'E')

# Right medium motor
motorD = Motor("D")
right_medium_motor = motorD
# Left medium motor
motorF = Motor("F")
left_medium_motor = motorF

#Right color sensor
colorB = ColorSensor("B")
rightColorSensor = colorB # Easier alias to use in code

#Left color sensor
colorA = ColorSensor("A")
leftColorSensor = colorA #Easier alias to use in code

_CM_PER_INCH = 2.54

testX2 = [10]
testY2 = [10]

# This is based on emperical tests and by looking at the color sensor.
# This is also based on the new sensor mount which roughly puts the sensor
# at about 18-20mm of the ground.
BLACK_COLOR = 20
WHITE_COLOR = 90

def driverWithFewerArms():
    counter = 1
    arm_change_end_time = 0
    arm_change_start_time = 0
    while True:
        if counter == 7: 
            break
        # Skip printing for the first time the loop runs.
        if (counter != 1):
            arm_change_start_time = time.ticks_ms()
            logMessage("Waiting for arm change", level=0)

        primeHub.speaker.beep(90, 1)
        primeHub.right_button.wait_until_pressed()
        if (counter != 1):
            arm_change_end_time = time.ticks_ms()      
            logMessage("Time for arm change time(ms): {}".format(str(time.ticks_diff(arm_change_end_time, arm_change_start_time))), level=0)

        if counter == 1:
            doRunWithTiming(_fasterRun1)
        if counter == 2:
            doRunWithTiming(_fasterRun2)
        if counter == 3:
            doRunWithTiming(_run3)
        if counter == 4:
            doRunWithTiming(_run4)
        if counter == 5:
            doRunWithTiming(_run6)
        counter = counter + 1

#region Utilities
def _initialize(): 
    print("___________________________________________________")
    global TOTAL_DEGREES_TURNED
    TOTAL_DEGREES_TURNED = 0
    primeHub.motion_sensor.reset_yaw_angle()
    wheels.set_stop_action("brake")
    wheels.set_motor_rotation(2*3.14*WHEEL_RADIUS_CM, 'cm')
    isBatteryGood()

def resetTotalDegreesTurned():
    global TOTAL_DEGREES_TURNED
    TOTAL_DEGREES_TURNED = 0

def getyawangle():
    return primeHub.motion_sensor.get_yaw_angle()
    
def measureColor():
    while(True):
        primeHub.right_button.wait_until_pressed()
        sumLeftColor = 0
        sumRightColor = 0
        counter = 1
        while(counter < 200):
            left_light = colorA.get_reflected_light()
            right_light = colorB.get_reflected_light()
            sumLeftColor += left_light
            sumRightColor += right_light
            counter = counter + 1
        
        avgLeftColor = sumLeftColor / counter
        avgRightColor = sumRightColor / counter
        logMessage("Left color={} Right Color={}".format(str(avgLeftColor), str(avgRightColor)), level=0)
            
def doRunWithTiming(run):
    logMessage("Starting run {}".format(str(run)), level=0)
    start_time = time.ticks_ms()  
    run()
    end_time = time.ticks_ms()
    logMessage("Time for run {} time(ms): {}".format(str(run), str(time.ticks_diff(end_time, start_time))), level=0)

def logMessage(message = "", level=1):
    """
    level: parameter between 1-5. 5 is the most detailed level.

    Prints the message that is passed to the function
    The printing is controlled by the level parameter.
    The function will only print if the passed level is higher than the global level.
    If the global level is set to zero nothing will print.
    """
    if (level <= GLOBAL_LEVEL):
        print(message)
   

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
    while abs(currentDegrees - startDegrees) < abs(degrees):
        currentDegrees = motor.get_degrees_counted()
        
    motor.stop()


def correctedGyroAngleZeroTo360():
    """
        Returns a number between 0-360. Note that it will not return 180 because the yaw angle is never 180.
    """
    yaw = getyawangle()
    if (yaw < 0):
        return 360 + yaw
    else:
        return yaw

def gyroAngleZeroTo360():
        """
        Returns a number between 0-360. Note that it will not return 180 because the yaw angle is never 180.
        """
        yaw = primeHub.motion_sensor.get_yaw_angle()
        #logMessage("In gyroAngleZeroTo360: yaw is " + str(yaw), level=5)
        if (yaw < 0):
            return 360 + yaw
        else:
            return yaw

def calculateReducedTargetAngleAndCorrection(angle, correction):
    if correction == 0:
        return calculateReducedTargetAngle(angle), 0
    else:
        return angle, correction


def calculateReducedTargetAngle(angle):
    '''
    angle : between -179 and + 179
    '''
    global TOTAL_DEGREES_TURNED
    currentAngle = gyroAngleZeroTo360()

    anglein360 = angle
    if (angle < 0):
        anglein360 = angle + 360
    
    # Compute whether the left or the right
    # turn is smaller.
    degreesToTurnRight = 0
    degreesToTurnLeft = 0
    if (anglein360 > currentAngle):
        degreesToTurnRight = anglein360 - currentAngle
        degreesToTurnLeft = (360-anglein360) + currentAngle
    else:
        degreesToTurnLeft = currentAngle - anglein360
        degreesToTurnRight = (360-currentAngle) + anglein360
     
    def _calculatecorrection(degreesToTurn):
        global TOTAL_DEGREES_TURNED
        # Adjust the global degrees turned
        TOTAL_DEGREES_TURNED += degreesToTurn
    
        # Reduce the angle to turn based on correction.
        if (abs(degreesToTurn) <= 60):
            percentageToCorrect = 50-(0.73*abs(degreesToTurn))
            return (-1*degreesToTurn * percentageToCorrect) / 100
        elif (abs(degreesToTurn) <= 90):
            percentageToCorrect = 13.32 - (0.12*abs(degreesToTurn))
            return (-1*degreesToTurn * percentageToCorrect) / 100
        else:
            percentageToCorrect = 2.5
            return (-1*degreesToTurn * percentageToCorrect) / 100

        # Use this function if using Amogh's robot.
        return int((-1 * 0.025 * TOTAL_DEGREES_TURNED)) 

    degreesToTurn = 0
    if (degreesToTurnLeft < degreesToTurnRight):
        degreesToTurn = degreesToTurnLeft * -1  
    else:
        degreesToTurn = degreesToTurnRight
    
    degreesToCorrect = _calculatecorrection(degreesToTurn)
    reducedTargetAngle = angle + degreesToCorrect
    
    # Switch the reduced target angle to the proper -179 to + 179 space.
    if (reducedTargetAngle > 180):
        reducedTargetAngle = reducedTargetAngle - 360
    
    if (reducedTargetAngle < -180):
        reducedTargetAngle = 360 + reducedTargetAngle

    if reducedTargetAngle == 180:
        reducedTargetAngle = 179

    if reducedTargetAngle == -180:
        reducedTargetAngle = -179

    logMessage("currentAngle={} angleIn360={} angle={} reducedTargetAngleIn179Space={} TOTAL_DEGREES_TURNED={}".format(
        str(currentAngle),str(anglein360),str(angle), str(reducedTargetAngle),str(TOTAL_DEGREES_TURNED),str()))
    return  int(reducedTargetAngle)


def flushForTime(speed=30, timeInSeconds=2):
    wheels.start(steering=0, speed=speed)
    wait_for_seconds(timeInSeconds)
    wheels.stop()
    
def _turnToAngle(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.4, correction=0.05, oneWheelTurn="None"):
    """Turns the robot the specified angle.
    It calculates if the right or the left turn is the closest
    way to get to the target angle. Can handle both negative 
    targetAngle and negative gyro readings.
    targetAngle -- the final gyro angle to turn the robot to. This should be between -179 and +179
    speed -- the speed to turn.
    forceTurn -- Can be "None", "Right" or "Left" strings, forcing
    the robot to turn left or right independent of the shortest 
    path.
    slowTurnRatio -- A number between 0.1 and 1.0. Controls the 
    amount of slow turn. If set to 1.0 the entire turn is a slow turn
    the default value is 0.2, or 20% of the turn is slow.
    correction -- The correction value in ratio. If its set to 0.05, we are going to 
    addjust the turnAngle by 5%, if you dont want any correction set it to 0
    oneWheelTurn -- "Left", "Right" or "None"(default). Useful if one of your wheels is in perfect
    position and you just want the robot to turn with the other wheel

    Note about the algorithm. There are three angle spaces involved in this algo.
    1. Spike prime gyro angles: -179 to +179. This is the input targetAngle and also the readings from the gyro.
    2. Spike prime 0-360 space. We first convert spike prime gyro angles to 0-360 
       (this is because its easier to think in this space)
    """
    logMessage("TurnToAngleStart current_angle={} targetAngle={}".format(str(getyawangle()), targetAngle), level=4)
    wheels.stop()
    currentAngle = gyroAngleZeroTo360()
    
    if (targetAngle < 0):
        targetAngle = targetAngle + 360
    
    # Compute whether the left or the right
    # turn is smaller.
    degreesToTurnRight = 0
    degreesToTurnLeft = 0
    if (targetAngle > currentAngle):
        degreesToTurnRight = targetAngle - currentAngle
        degreesToTurnLeft = (360-targetAngle) + currentAngle
    else:
        degreesToTurnLeft = currentAngle - targetAngle
        degreesToTurnRight = (360-currentAngle) + targetAngle
     
    degreesToTurn = 0
    direction = "None"
    if (forceTurn == "None"):
        if (degreesToTurnLeft < degreesToTurnRight):
            degreesToTurn = degreesToTurnLeft * -1
            direction = "Left"
        else:
            degreesToTurn = degreesToTurnRight
            direction = "Right"
    elif (forceTurn == "Right"):
        degreesToTurn = degreesToTurnRight
        direction = "Right"
    elif (forceTurn == "Left"):
        degreesToTurn = degreesToTurnLeft * -1
        direction = "Left"

    # Use the correction to correct the target angle and the degreesToTurn
    # note that the same formula is used for both left and right turns
    # this works because the degreesToTurn is +ve or -ve based
    # on which way we are turning.
    reducedTargetAngle = targetAngle
    if (correction != 0):
        if (abs(degreesToTurn) > 20):
            reducedTargetAngle = targetAngle - (degreesToTurn * correction)
            degreesToTurn = degreesToTurn * (1-correction)

    # Put the target angle back in -179 to 179 space.    
    reducedTargetAngleIn179Space = reducedTargetAngle
    # Changed from targetAngle to reducedTargetAngle as it goes into loop
    if (reducedTargetAngleIn179Space >= 180):
        reducedTargetAngleIn179Space = reducedTargetAngle - 360

    _turnRobotWithSlowDown(degreesToTurn, reducedTargetAngleIn179Space, speed, slowTurnRatio, direction, oneWheelTurn=oneWheelTurn)    
    currentAngle = correctedGyroAngleZeroTo360()
    logMessage("TurnToAngle complete. GyroAngle:{} reducedtargetAngle(0-360):{} ".format(str(getyawangle()), str(reducedTargetAngleIn179Space)), level=4)

def _turnRobotWithSlowDown(angleInDegrees, targetAngle, speed, slowTurnRatio, direction, oneWheelTurn="None"):
    """
    Turns the Robot using a fast turn loop at speed and for the slowTurnRatio
    turns the robot at SLOW_SPEED.

    angleInDegrees -- Angle in degrees to turn. Can be +ve or -ve.
    targetAngle -- targetAngle should be in the -179 to 179 space
    speed -- Fast turn speed. 
    slowTurnRatio -- This is the % of the turn that we want to slow turn.
                     For example 0.2 means that 20% of the turn we want
                     to slow turn.
    oneWheelTurn -- Optional parameter with "None" as the default. Values can be "Left", "Right", "None".
    """
    SLOW_SPEED = 10
    currentAngle = getyawangle()
    
    # First we will do a fast turn at speed. The amount to turn is 
    # controlled by the slowTurnRatio.
    _turnRobot(direction, speed, oneWheelTurn)
    fastTurnDegrees =  (1 - slowTurnRatio) * abs(angleInDegrees)
    while (abs(currentAngle - targetAngle) > fastTurnDegrees):
        currentAngle = getyawangle()

    # After the initial fast turn that is done using speed, we are going to do a 
    # slow turn using the slow speed.
    _turnRobot(direction, SLOW_SPEED, oneWheelTurn)
    while (abs(currentAngle - targetAngle) > 1):
        currentAngle = getyawangle()

    wheels.stop()
    
def _turnRobot(direction, speed, oneWheelTurn):
    if (oneWheelTurn == "None"):
        if (direction == "Right"):
            wheels.start_tank(speed, speed * -1)
        if (direction == "Left"):
            wheels.start_tank(speed * -1, speed)
    elif (oneWheelTurn == "Left"):
        left_large_motor.start(speed)
    else:
        right_large_motor.start(speed)

def gyroStraight(distance, speed = 20, backward = False, targetAngle = 0, multiplier=1.0, gradualAcceleration=True, slowDown=True):
    logMessage("=========== GyroStraight Start distance={} current_angle={} targetAngle={}".format(str(distance), str(getyawangle()),str(targetAngle)), level=4)
    correctionMultiplier = multiplier
    initialDeg = abs(motorE.get_degrees_counted())
    if(distance < _CM_PER_INCH*3):
        _gyroStraightNoSlowDownNoStop(distance = distance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = correctionMultiplier)
        wheels.stop()
        return
    
    gradualAccelerationDistance = 0
    slowDistance = 0
    if slowDown == True:
        slowDistance = 0.2 * distance
        if(slowDistance > _CM_PER_INCH*2):
            slowDistance = _CM_PER_INCH*2

    # Run slow if the gradual acceleration is on.    
    if gradualAcceleration == True:
        gradualAccelerationDistance = _CM_PER_INCH*1
        _gyroStraightNoSlowDownNoStop(distance = gradualAccelerationDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = correctionMultiplier)
    
    # Do the middle part of the run
    _gyroStraightNoSlowDownNoStop(distance = distance - slowDistance - gradualAccelerationDistance, speed = speed, targetAngle=targetAngle, backward=backward, correctionMultiplier = correctionMultiplier)
    
    # Slow down at the end.
    if slowDown == True:
        _gyroStraightNoSlowDownNoStop(distance = slowDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = correctionMultiplier)

    wheels.stop()

    finalDeg = abs(motorE.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = {} error = {}".format(str(totalDistanceTravelled), str(distance-totalDistanceTravelled)), level=4)
    logMessage("=========== GyroStraight complete distance={} current_angle={}".format(str(distance), str(getyawangle())), level=4)
    

def _gyroStraightNoSlowDownNoStop(distance, speed = 20, backward = False, targetAngle = 0, correctionMultiplier = 2):
    underBiasErrorMultiplier = 1 # 1.106
    errorAdjustedDistanceInCm = distance*underBiasErrorMultiplier

    #logMessage("GYROSTRAIGHT START: targetAngle  is {}".format(str(targetAngle)), level=4)
    degreesToCover = (errorAdjustedDistanceInCm * 360)/(WHEEL_RADIUS_CM * 2 * 3.1416)
    position_start = motorE.get_degrees_counted()
    if (backward): 
        while ((motorE.get_degrees_counted() - position_start)  >= degreesToCover * -1):
            # currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = getCorrectionForDrive(targetAngle, correctionMultiplier = correctionMultiplier) # - currentAngle
            wheels.start(steering = -correction, speed=speed * -1)
    else:
         while ((motorE.get_degrees_counted() - position_start)  <= degreesToCover):           
            # currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = getCorrectionForDrive(targetAngle, correctionMultiplier = correctionMultiplier)
            wheels.start(steering = correction, speed=speed)   

def _turnToAngle2(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.4, correction=0.05, oneWheelTurn="None"):
    degreesToTurn = getCorrectionForDrive(targetAngle=targetAngle, correctionMultiplier=1)
    motorDegToTurn = getMotorRotationDegreesForTurn(degreesToTurn=abs(degreesToTurn), oneWheelTurn=oneWheelTurn)
    # print("Deg to turn = " + str(motorDegToTurn))
    if(degreesToTurn < 0):
        direction = "Left"
    else:
        direction = "Right"
    turnForMotorRotations(degreesOfRotation=motorDegToTurn, direction=direction, speed=speed, oneWheelTurn=oneWheelTurn)
    
def getMotorRotationDegreesForTurn(degreesToTurn, oneWheelTurn="None"):
    motorRotationDegrees = degreesToTurn*(AXLE_DIAMETER_CM_CORRECTED / (2*WHEEL_RADIUS_CM)) # (2πr_Axle/360) * degreesToTurn * (360/2πr_Wheel)
    if(oneWheelTurn != "None"):
        motorRotationDegrees = 2*motorRotationDegrees
    return motorRotationDegrees

def turnForMotorRotations(degreesOfRotation, direction, speed=40, oneWheelTurn="None"):
    position_start_right = right_large_motor.get_degrees_counted()
    position_start_left = left_large_motor.get_degrees_counted()
    
    leftSpeed = 0
    rightSpeed = 0
    turnLeftRemaining = 0
    turnRightRemaining = 0
    if(direction == "Left"):
        if(oneWheelTurn == "Left" or oneWheelTurn == "None"):
            leftSpeed = speed
            turnLeftRemaining = degreesOfRotation
            # print("Left Turn: Right speed=" + str(rightSpeed) + ", Left Remaining=" + str(turnRightRemaining))
        if(oneWheelTurn == "Right" or oneWheelTurn == "None"):
            rightSpeed = speed
            turnRightRemaining = degreesOfRotation
            # print("Left Turn: Right speed=" + str(rightSpeed) + ", Right Remaining=" + str(turnRightRemaining))
    if(direction == "Right"):
        if(oneWheelTurn == "Left" or oneWheelTurn == "None"):
            leftSpeed = -1*speed
            turnLeftRemaining = degreesOfRotation
            # print("Right Turn: Left speed=" + str(leftSpeed) + ", Left Remaining=" + str(turnLeftRemaining))
        if(oneWheelTurn == "Right" or oneWheelTurn == "None"):
            rightSpeed = -1*speed
            turnRightRemaining = degreesOfRotation
            # print("Right Turn: Right speed=" + str(rightSpeed) + ", Right Remaining=" + str(turnRightRemaining))


    allowedError = 4 * pow(2, int(speed/10)-1) # Picked after some ad hoc testing for different speeds
    # print("Start positions: Left=" + str(position_start_left) + ", Right=" + str(position_start_right))
    while(abs(turnLeftRemaining) > allowedError or abs(turnRightRemaining) > allowedError):
        # print("left: " + str(turnLeftRemaining) + ", right: " + str(turnRightRemaining))
        if(abs(turnLeftRemaining) > allowedError):
            left_large_motor.start(speed=leftSpeed) #degrees = turnLeftRemaining, speed=20)
            turnLeftRemaining = position_start_left + (leftSpeed/abs(leftSpeed))*degreesOfRotation - left_large_motor.get_degrees_counted()
        else:
            left_large_motor.stop()
        if(abs(turnRightRemaining) > allowedError):
            right_large_motor.start(speed=rightSpeed) #degrees = -1*turnRightRemaining, speed=20)
            turnRightRemaining = position_start_right + (rightSpeed/abs(rightSpeed))*degreesOfRotation - right_large_motor.get_degrees_counted()
        else:
            right_large_motor.stop()
    left_large_motor.stop()
    right_large_motor.stop()


    # if (backward): 
    #     while ((motorE.get_degrees_counted() - position_start)  >= degreesToCover * -1):
           
    #         # currentAngle = primeHub.motion_sensor.get_yaw_angle()
    #         correction = getCorrectionForDrive(targetAngle, correctionMultiplier = correctionMultiplier) # - currentAngle
    #         wheels.start(steering = -correction, speed=speed * -1)
    # else:
    #      while ((motorE.get_degrees_counted() - position_start)  <= degreesToCover):
           
    #         # currentAngle = primeHub.motion_sensor.get_yaw_angle()
    #         correction = getCorrectionForDrive(targetAngle, correctionMultiplier = correctionMultiplier) # targetAngle - currentAngle
    #         wheels.start(steering = correction, speed=speed)

def getCorrectionForDrive(targetAngle, correctionMultiplier = 2):
    currentAngle = getyawangle()
    #primeHub.motion_sensor.get_yaw_angle()
    #logMessage("CurrentAngle: " + str(currentAngle) + " and targetAngle: " + str(targetAngle), 5)
    if( (currentAngle <= 0 and targetAngle <=0) or
            (currentAngle>=0 and targetAngle > 0) or
            (abs(currentAngle) <= 90 and abs(targetAngle)<=90)):
        correction = targetAngle - currentAngle
    elif (currentAngle >= 90):
        correction = (360 - abs(currentAngle) - abs(targetAngle))
    else:
        correction = -1*(360 - abs(currentAngle) - abs(targetAngle))

    return int(correction * correctionMultiplier)

def _turnAndDrive(targetAngle, distance, speed):
    #angle = targetAngle
    angle = calculateReducedTargetAngle(targetAngle)
    _turnToAngle(targetAngle = angle, speed = 25, correction=0)
    if distance != 0:
        gyroStraight(distance=distance, speed = speed, backward = False, targetAngle = angle)

def convertDegToCM(degrees):
    return degrees * WHEEL_RADIUS_CM * pi * 2 / 360

def converCMToDeg(distance):
    return distance * 360 / (WHEEL_RADIUS_CM * pi * 2)

def convertInchesToCM(distanceInInches):
    """
    Convert Inches To CM
    ____________________
    """
    return _CM_PER_INCH * distanceInInches

def _driveTillLine(speed, distanceInCM, target_angle, gain = 1, colorSensorToUse="Left", blackOrWhite="Black", slowSpeedRatio = 0.6):
    """
    Drive
    _____
    This function drives the robot FORWARD using the motion sensor and the 80-20 formula.
    80% of distance at speed
    20% of distance with FINAL_SLOW_SPEED
    _____
    Speed - Speed the wheels travel at. Integer from -100 to 100
    DistanceInCM - Distance to travel in centimeters. Integer greater than 0
    TargetAngle - The angle the robot should drive at. Integer from 0 to 360
    Gain - The multiplier off the error. Integer greater than 0
    colorSensorToUse - "Left" or "Right". 
    blackOrWhite - "Black" or "White".
    """
    wheels.stop()
   
    logMessage("driveStraight for distance:{} target angle:{}".format(str(distanceInCM), str(target_angle)), level=2)
    initialDeg = abs(motorC.get_degrees_counted())
    remainingDistance = distanceInCM
    
    # First establish which color sensor to use.
    colorSensor = None
    if (colorSensorToUse == "Left"):
        colorSensor = colorA
    else:
        colorSensor = colorB

    # Now establish the termination condition to use.
    stoppingCondition = None
    if (blackOrWhite == "Black"):
        def blackStoppingCondition():
            light = colorSensor.get_reflected_light()
            return light <= BLACK_COLOR
        stoppingCondition = blackStoppingCondition
    elif (blackOrWhite == "White"):
        def whiteStoppingCondition():
            light = colorSensor.get_reflected_light()
            return light >= WHITE_COLOR
        stoppingCondition = whiteStoppingCondition
    elif (blackOrWhite == "Green"):
        stoppingCondition = lambda: colorSensor.get_color() == 'green'

    FINAL_SLOW_SPEED=10
    # If the distance is small, then just drive over that distance at FINAL_SLOW_SPEED.
    if (distanceInCM < 5):
        reachedStoppingCondition = _driveStraightWithSlowDownTillLine(distanceInCM, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)    
    else:
        # First drive 60% of the distance at speed
        distance60 = distanceInCM * slowSpeedRatio
        reachedStoppingCondition = _driveStraightWithSlowDownTillLine(distance60, speed, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)
        if reachedStoppingCondition == False:
            # Drive the remaining distance at slow speed
            distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
            remainingDistance = distanceInCM - distanceTravelled
            logMessage("_driveTillLine: Distance travelled after first part = {} error={}".format(str(distanceTravelled),str(distanceTravelled-distance60)), level=4)
            reachedStoppingCondition = _driveStraightWithSlowDownTillLine(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)

    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("_driveTillLine: Total distance travelled={} error={}".format(str(totalDistanceTravelled), str(totalDistanceTravelled-distanceInCM)), level=2)
    return reachedStoppingCondition
    
def _driveStraightWithSlowDownTillLine(distance, speed, target_angle, gain, slowDown, reachedStoppingCondition):
    """
    Drive Straight
    ______________
    This is a internal function do not call directly. Call drive instead.

    The algorithm goes from speed to speed 10 until distance is travelled.
    slowDown: True if you want to slow down over the distance. Note that
    the function uses a combination of distance and the reachedStoppingCondition
    checks to stop.

    reachedStoppingCondition: This is a function that does not take any parameter and is expected to retur true when the loop should be terminated.
    return the output of the stoppingCondition.
    """
    startDistanceInDeg = abs(motorC.get_degrees_counted())
    logMessage("startDistanceInDeg={}".format(str(int(startDistanceInDeg))), level=5)
    distanceInDeg = converCMToDeg(distance)
    currentSpeed = speed

    if (target_angle == -180):
        target_angle = 180

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0
    
    FINAL_SLOW_SPEED=15
    wheels.start(0, int(currentSpeed))
    correction = previousCorrection = 0
    stopCondition = False
    while  distanceInDegTravelled <= distanceInDeg and stopCondition == False:
        if (slowDown == True):
            currentSpeed = currentSpeed-1
            if(currentSpeed < 15):
                currentSpeed = 15
       
        current_yaw_angle = primeHub.motion_sensor.get_yaw_angle()

        # This hackery is needed to handle 180 or -180 straight run.
        if (target_angle == 180 and current_yaw_angle < 0):
            current_yaw_angle = (360 + current_yaw_angle)

        previousCorrection = correction
        correction = target_angle - current_yaw_angle
        
        turn_rate = correction * gain
        #logMessage("Left color={} Right color={} currrentSpeed={} distanceInDegTravelledInCM={} distanceInCM={} distanceInDegTravelled={} distanceToTravelInDeg={} target_angle={} current_yaw_angle={} correction={}".format(
        #    str(colorA.get_reflected_light()), str(colorB.get_reflected_light()), str(int(currentSpeed)), str(convertDegToCM(distanceInDegTravelled)), str(distance),
        #    str(distanceInDegTravelled), str(distanceInDeg), str(target_angle), str(current_yaw_angle), str(correction)), level=5)

        if (abs(correction) > 1):
            wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
        stopCondition = reachedStoppingCondition()

    logMessage("DrivestraightWiuthSlowDownTillLine completed", level=5)
    return stopCondition
    
def _driveBackwardTillLine(distance, speed, target_angle, colorSensorToUse="Left", blackOrWhite="Black", gain=1, useAngularCorrection=True):
    wheels.stop()
    # First establish which color sensor to use.
    colorSensor = None
    if (colorSensorToUse == "Left"):
        colorSensor = colorA
    else:
        colorSensor = colorB

    # Now establish the termination condition to use.
    stoppingCondition = None
    if (blackOrWhite == "Black"):
        def blackStoppingCondition():
            light = colorSensor.get_reflected_light()
            return light <= BLACK_COLOR
        stoppingCondition = blackStoppingCondition
    elif (blackOrWhite == "White"):
        def whiteStoppingCondition():
            light = colorSensor.get_reflected_light()
            return light >= WHITE_COLOR
        stoppingCondition = whiteStoppingCondition
    elif (blackOrWhite == "Green"):
        stoppingCondition = lambda: colorSensor.get_color() == 'green'

    startDistanceInDeg = abs(motorC.get_degrees_counted())
    distanceInDeg = converCMToDeg(distance)
    currentSpeed = -1*speed

    if (target_angle == -180):
        target_angle = 180

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0
    
    FINAL_SLOW_SPEED=15
    wheels.start(0, int(currentSpeed))
    stopCondition = False
    while  distanceInDegTravelled <= distanceInDeg and stopCondition == False:
        if useAngularCorrection == True:
            current_yaw_angle = primeHub.motion_sensor.get_yaw_angle()

            # This hackery is needed to handle 180 or -180 straight run.
            if (target_angle == 180 and current_yaw_angle < 0):
                current_yaw_angle = (360 + current_yaw_angle)
            correction = target_angle - current_yaw_angle
            
            turn_rate = correction * gain
            if (abs(correction) > 1):
                wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
        stopCondition = stoppingCondition()

    logMessage("DrivestraightWiuthSlowDownTillLine completed", level=5)
    return stopCondition
    
# Line squares on the black line. Call this function once one of the color sensors hits the line.    
def lineSquare():
    sensor = "Left"
    logMessage("left color: " + str(colorA.get_reflected_light()) + " Right color: " + str(colorB.get_reflected_light()), level=4)

    # First find out which color sensor hit the line.
    if (colorA.get_reflected_light() <= BLACK_COLOR):
        sensor = "Left"
    elif (colorB.get_reflected_light() <= BLACK_COLOR):
        sensor = "Right"
    else:
        assert False

    logMessage("Found Black Line on : " + sensor, level=4)
    if (sensor == "Left"):
        # Turn left till the right color sensor also hits black.
        wheels.start(100, -15)
        while (abs(colorB.get_reflected_light() - colorA.get_reflected_light()) >= 1):
            logMessage("left color: " + str(colorA.get_reflected_light()) + " Right color: " + str(colorB.get_reflected_light()), level=4)

            continue
        wheels.stop()
    else:
        # Turn right till the Left color sensor also hits black.
        wheels.start(100, 15)
        while (colorA.get_reflected_light() >= BLACK_COLOR):
            logMessage("left color: " + str(colorA.get_reflected_light()) + " Right color: " + str(colorB.get_reflected_light()), level=4)
            continue
        wheels.stop()
        
    
def isGyroGood():
    """
    Is Gyro Good
    ____________
    This function checks if the gyro is drifting or not. If it is not, then is returns true.
    Otherwise, it returns false and plays a beep noise.
    ____________
    It is recommended to use this function BEFORE the match, as it takes ~ 6 seconds to complete.
    """
    
    angle1 = primeHub.motion_sensor.get_yaw_angle()
    time.sleep(5)
    angle2 = primeHub.motion_sensor.get_yaw_angle()

    if abs(angle1 - angle2) > 2:
        primeHub.speaker.beep(60, 1)
        print("Please reset gyro")
        return False
    
    return True
   

def isBatteryGood():
    
    logMessage("Battery voltage: " + str(hub.battery.voltage()), level=1)
    
    if hub.battery.voltage() < 7600:
        primeHub.speaker.beep(120, 1)
        print("Please recharge robot")
        return False

    return True
   
def wait_until_either_color(sensor1, sensor2, color, message):
    while True:
        if((color == "black" and (sensor1.get_reflected_light() <= BLACK_COLOR or sensor2.get_reflected_light() <= BLACK_COLOR)) or (color == "white" and (sensor1.get_reflected_light() >= WHITE_COLOR or sensor2.get_reflected_light() >= WHITE_COLOR))):
            logMessage(message, level=1)
            return

def wait_until_both_color(sensor1, sensor2, color, message):    
    while True:
        if((color == "black" and (sensor1.get_reflected_light() <= BLACK_COLOR and sensor2.get_reflected_light() <= BLACK_COLOR)) or (color == "white" and (sensor1.get_reflected_light() >= WHITE_COLOR and sensor2.get_reflected_light() >= WHITE_COLOR))):
                logMessage(message, level=1)
                return

def wait_until_color(sensor, color, message):
    while(True):
        if ((color == "black" and sensor.get_reflected_light() <= BLACK_COLOR) or (color == "white" and  sensor.get_reflected_light() >= WHITE_COLOR)):
            logMessage(message, level=1)
            wheels.stop()
            return

# This code assumes that ahead of Marvin there is a black line that it has
# to stand square to
def lineSquaring(speed):
    lowspeed = 10
    numTimes = 1
    # move straight until one of the left sensor or right sensor lands on black line
    wheels.start(0, speed)
    wait_until_either_color(rightColorSensor, leftColorSensor, "black", "First hit")
    wheels.stop()
 
    if(rightColorSensor.get_reflected_light() <= BLACK_COLOR):
        logMessage("right sensor found black", level=3)
        logMessage("move left wheel forward", level=3)
        wheels.start_tank(lowspeed, -1)
        wait_until_color(leftColorSensor, "black", "left sensor found black")
        # at this point we are as close t straight as we could get with that speed
        # pull back both wheels off the black line
        for x in range(numTimes):
            wheels.start_tank(-lowspeed, -lowspeed)
            wait_until_both_color(rightColorSensor, leftColorSensor, "white", "back off from black line")
            wheels.stop()
    
            logMessage(str(leftColorSensor.get_reflected_light()), level=3)
            logMessage(str(rightColorSensor.get_reflected_light()), level=3)

            #start again
            wheels.start(0, lowspeed)
            wait_until_either_color(rightColorSensor, leftColorSensor, "black", "second time black hit")
            wheels.stop()
            logMessage(str(leftColorSensor.get_reflected_light()), level=3)
            logMessage(str(rightColorSensor.get_reflected_light()), level=3)

            if(rightColorSensor.get_reflected_light() <= BLACK_COLOR):
                logMessage("right sensor found black", level=3)
                logMessage("move left wheel forward", level=3)
                wheels.start_tank(lowspeed, 0)
                wait_until_color(leftColorSensor, "black", "left sensor found black")
                
                logMessage("move right wheel backward", level=3)
                wheels.start_tank(0, -lowspeed)
                wait_until_color(rightColorSensor, "white", "right sensor found white")

                logMessage("move left wheel backward", level=3)
                wheels.start_tank(-lowspeed, 0)
                wait_until_color(leftColorSensor, "white", "left sensor found white")
                
                logMessage("move right wheel forward", level=3)
                wheels.start_tank(0, lowspeed)
                wait_until_color(rightColorSensor, "black", "right sensor found black")
                
                logMessage("move left wheel forward", level=3)
                wheels.start_tank(lowspeed, 0)
                wait_until_color(leftColorSensor, "black", "left sensor found black")

                logMessage("***********ENOUGH**********", level=3)
 

    elif (leftColorSensor.get_reflected_light() <= BLACK_COLOR):
        logMessage("left sensor found black", level=3)
        logMessage("move right wheel forward", level=3)
        wheels.start_tank(-1, lowspeed)
        wait_until_color(rightColorSensor, "black", "right sensor found black")

        logMessage(str(leftColorSensor.get_reflected_light()), level=3)
        logMessage(str(rightColorSensor.get_reflected_light()), level=3)   
        
        # at this point we are as close t straight as we could get with that speed
        # pull back both wheels off the black line
        for x in range(numTimes):
            wheels.start_tank(-lowspeed, -lowspeed)
            wait_until_both_color(rightColorSensor, leftColorSensor, "white", "back off from black line")
            wheels.stop()
            
            #start again
            wheels.start(0, lowspeed)
            wait_until_either_color(rightColorSensor, leftColorSensor, "black", "second time black hit")
            wheels.stop()

            if(leftColorSensor.get_reflected_light() <= BLACK_COLOR):
                logMessage("left sensor found black", level=3)
                logMessage("move right wheel forward", level=3)
                wheels.start_tank(0, lowspeed)
                wait_until_color(rightColorSensor, "black", "right sensor found black")
                
                logMessage("move left wheel backward", level=3)
                wheels.start_tank(-lowspeed, 0)
                wait_until_color(leftColorSensor, "white", "left sensor found white")

                logMessage("move right wheel backward", level=3)
                wheels.start_tank(0, -lowspeed)
                wait_until_color(rightColorSensor, "white", "right sensor found white")
                
                logMessage("move left wheel forward", level=3)
                wheels.start_tank(lowspeed, 0)
                wait_until_color(leftColorSensor, "black", "left sensor found black")
                
                logMessage("move right wheel forward", level=3)
                wheels.start_tank(0, lowspeed)
                wait_until_color(rightColorSensor, "black", "right sensor found black")

                logMessage("***********ENOUGH**********", level=3)

   

def testLineSquaring():
    lineSquaring(30)

 # ------------------------------------------------------------------- End Utilities --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 #endregion

#region Arisha
def _run4():
    primeHub.motion_sensor.reset_yaw_angle()

    getToOilPlatform_v2Point2()
    activeOilPlatform()
    goBackHomeFromOilPlatform()

    # pullTruckGoStraight()

def getToOilPlatform_v2Point2():
    #print("Running now getToOilPlatform")
    #working version1
    gyroStraight(distance=_CM_PER_INCH*16.8, speed=60, targetAngle=0) #was 90
    # time.sleep(5)
    _turnToAngle(45)
    # time.sleep(5)
    gyroStraight(distance=_CM_PER_INCH*3, speed=60, targetAngle=45) #was 90
    _driveTillLine(speed = 30, distanceInCM = _CM_PER_INCH*6, target_angle = 45, blackOrWhite="White") # was 12 inches ####
    #time.sleep(5)
    _turnToAngle(targetAngle=-5, oneWheelTurn="Right", speed=40)
    #time.sleep(5)
    gyroStraight(distance=_CM_PER_INCH*11.5, speed=30, targetAngle=-2) # was 10.5 ####
    #time.sleep(10)
    #motorD.start(speed=-30)
    # gyroStraight(distance=_CM_PER_INCH*9.5, speed=30, targetAngle=0) # was 10.5 ####

def activeOilPlatform():
    # gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2.5, speed=40)
    # motorD.start(speed=-30)

    # time.sleep(10)
    for i in range(3):
        motorF.run_for_degrees(degrees=700, speed=100)
        if (i<=1):
            motorF.run_for_degrees(degrees=-700, speed=100)
        # if (i>=1):
        # motorD.start(speed=-30)
        # time.sleep(5)
    # gyroStraight(distance=2, speed=40, targetAngle=0, backward=True)
    # time.sleep(0.5)
    motorD.run_for_degrees(degrees=-1000, speed=100)

    
    wheels.move(amount = 4, unit = "in", steering = 0, speed = -40)
    # motorD.stop()


def goBackHomeFromOilPlatform():
    _turnToAngle(30)
    gyroStraight(distance=24*_CM_PER_INCH, speed=100, targetAngle=30, backward=True) # Back home doesnt require accuracy
    # wheels.move(amount = 20, unit = "in", steering = 0, speed = -100) # Back home doesnt require accuracy
   #turnToAngle(30)
   #wheels.move(amount = 11, unit = "in", steering = 0, speed = -30) # Back home doesnt require accuracy

def _run5():
    primeHub.motion_sensor.reset_yaw_angle()
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH * 10, speed=40)
    motorF.run_for_degrees(degrees=-1000, speed=100)
    # motorF.start(speed=-100)
    # time.sleep(2)
    wheels.move(amount = 14, unit = "in", steering = 0, speed = -30)

def scale(amt):
    in_min  =  BLACK_COLOR
    in_max  =  WHITE_COLOR
    out_min = -10
    out_max =  10
    return (amt - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#endregion Arisha
#region Nami    

def _run3():
    # This is the new run3 with just doing the hydrodam. 
    # It picks the water unit in front of hydrodam and brings back the hydrodam unit.

    primeHub.motion_sensor.reset_yaw_angle()
    angle = 0
    #Lift arm
    moveArm(degrees = 120, speed = -75, motor = motorF)
    #Go forward towards the hydrodam
    gyroStraight(distance = 27,speed = 50, targetAngle = angle)
    #Hit the black part of hydrodam
    moveArm(degrees = 90, speed = 75, motor = motorF)
    #Move the arm back up so the energy unit can fall out
    moveArm(degrees = 90, speed = -75, motor= motorF)
    # This is needed to let the energy unit fall out from hydrodam
    time.sleep_ms(500)
    #Move arm back down to bring the water unit and energy unit home
    moveArm(degrees = 120, speed = 75,motor = motorF)
    #Go back home
    gyroStraight(distance = 27, speed = 50, backward = True, targetAngle = angle)
    
def _run6():
    primeHub.motion_sensor.reset_yaw_angle()
    # Drive forward first. Drive at a slight angle to avoid hitting the power plant.
    gyroStraight(distance= 50, speed= 65, targetAngle= -5)
    
    # Turn slightly to catch the n-s line in front of the power plant
    _driveTillLine(speed=30, distanceInCM=35, target_angle=-8, colorSensorToUse="Left", blackOrWhite="Black", slowSpeedRatio=0.9)
    
    # Turn towards the power plant and then try to catch the black line running e-w line in front of the smart grid.
    angle = -87
    _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.7)
    _driveTillLine(speed=45, distanceInCM=70, target_angle = angle, colorSensorToUse="Left", blackOrWhite="Black")
    #gyroStraight(distance=3, speed = 20, backward = True, targetAngle = angle)

    # Turn towards the hydro-electric plant and then drop the water units. 
    # This also drops the energy units and the innovation project.
    angle = 150
    _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.9)
    gyroStraight(speed=25, distance=26, targetAngle=angle)

    # Drop the water units    
    moveArm(degrees = 2200, speed = -100, motor = motorF)

    # Start bringing up the arm
    motorF.start_at_power(60)
    
    # Backoff to leave the water reservoir
    gyroStraight(distance=15, speed = 20, backward = True, targetAngle = angle)

    motorF.stop()    
    _dorun6ToyFactory()

def _dorun6ToyFactory():
    # turn to get to the toyfactory.
    angle = -175
    _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.9)
    gyroStraight(distance=18, speed = 35, backward = True, targetAngle = angle)

    angle = -135
    _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.9)
    gyroStraight(distance=10, speed = 35, backward = True, targetAngle = angle)

def _dropRechargeableBatteryAndOilTruckWithGyroReset():
    # Start bringing up the arm
    motorF.start_at_power(60)
    
    # Backoff to leave the water reservoir
    gyroStraight(distance=15, speed = 20, backward = True, targetAngle = 0)

    # Uncomment this for a serial run.
    #moveArm(degrees = 3000, speed = 100, motor = motorF)

    motorF.stop()    

    _doToyFactory()

    '''
    # Turn towards the toy factory
    _turnToAngle(targetAngle = 0 + zero_adjustment, speed = 20, slowTurnRatio=0.9)

    # Drive past the n-s line in front of the toy factory
    gyroStraight(distance=20, speed = 35, backward = False, targetAngle = 0 + zero_adjustment)

    # We expect the arm to have come up by now.
    motorF.stop()
    '''
    
    # Commented to remove the _run6 past the robot clearing the hydro plant.
    # This is done to remove the oil truck dropoff.
    # This change was done on 11/6/2022 after the team meeting where we decided to remove
    # _run5 and shorten _run6
    # Turn to catch e-w line in front of smartgrid
    _turnToAngle(targetAngle = 170, speed = 20, slowTurnRatio=0.9)

    # Its possible that this line is missed occasionally. 
    _driveTillLine(speed=35, distanceInCM=10, target_angle= 170, colorSensorToUse="Left", blackOrWhite="Black")

    # Turn to catch n-s line near toy factory
    # 10/23/2022: was 5+zero_adjustment
    _turnToAngle(targetAngle = 5 + zero_adjustment, speed = 20, slowTurnRatio=0.9)
    _driveTillLine(speed=35, distanceInCM=13, target_angle=5 + zero_adjustment, colorSensorToUse="Right", blackOrWhite="Black")

    # Move and turn towards rechargeable battery
    gyroStraight(distance=21, speed = 35, backward = False, targetAngle = 5 + zero_adjustment)
    _turnToAngle(targetAngle = 33 + zero_adjustment, speed = 20, slowTurnRatio=0.9)
    gyroStraight(distance=12, speed = 25, backward = False, targetAngle = 33 + zero_adjustment)

    # Drop energy units in rechargeable battery
    # Used to be 3000 made it 1800, by lowering the arm a litte.
    moveArm(degrees = 1800, speed = 100, motor = motorD)
    
    # Start picking up the arm. Uncomment this for the competition. Instead of the waiting for bringing up the arm.
    motorD.start_at_power(-100)
    #moveArm(degrees = 3000, speed = -100, motor = motorD)
    
    # Backup to the fuel station with the oil truck to finish
    _turnToAngle(targetAngle = 80 + zero_adjustment, speed = 20)
    gyroStraight(distance=30, speed = 35, backward = True, targetAngle = 80 + zero_adjustment)
    
    # Uncomment this for the competition.
    motorD.stop()

#endregion Nami

#region Rishabh

def _fasterRun1():
    def _watchTV():
        angle=0
        # Drive to Watch Television. We do this in two parts. First fast and
        # then do it slow so the energy unit does not fall off.
        gyroStraight(distance = 35, speed = 60, backward = False, targetAngle = angle)
        gyroStraight(distance = 5, speed = 20, backward = False, targetAngle = angle)
        
        # Backup from Watch Television
        gyroStraight(distance = 8, speed = 50, backward = True, targetAngle =angle)

    def _getToWindTurbine():
        turnspeed = 30
        angle = -30    
        _turnToAngle(targetAngle = angle, speed = turnspeed, slowTurnRatio = 0.9, correction=0.16)

        # We should have turned such that we are able to find the black line in front of the wind turbine.
        if (_driveTillLine(speed=60, distanceInCM=30, target_angle=angle, colorSensorToUse="Right", blackOrWhite="Black", slowSpeedRatio=0.9) == False):
            logMessage("NOTE -----------> Missed Catching the line before wind turbine", level=0)
        gyroStraight(distance = 6, speed = 20, backward = False, targetAngle = angle)

        # After catching the black line, drive forward and turn to face the windmill
        angle=40
        _turnToAngle(targetAngle = angle, speed = turnspeed, slowTurnRatio = 0.9, correction=0.16)

    def _windTurbine():
        angle = 40
        gyroStraight(distance=18, speed = 20, backward = False, targetAngle = angle)

        # Push the lever the remaining two times
        for i in range(3): 
            # Backup so the robot can push the Wind Turbine again
            gyroStraight(distance=7, speed = 30, backward = True, targetAngle = angle)
            # Drive forward to push the Wind Turbine
            #flushForTime(speed=20, timeInSeconds=0.6)
            gyroStraight(distance=10, speed = 20, backward = False, targetAngle = angle)

    def _pickUpRechargeableBattery():
        turnspeed = 35
        # Backoff from the windwill and flush against the rechargeable battery.
        # We go back fast first, and then slow down to flush.
        
        # First back off a little to be able to turn.
        angle = 40
        gyroStraight(distance=10, speed = 55, backward = True, targetAngle = angle)

        angle = 40
        _turnToAngle(targetAngle = angle, speed = turnspeed)
        gyroStraight(distance=5, speed = 45, backward = True, targetAngle = angle)
        
        # Flush against the toy factory
        angle = 40
        _turnToAngle(targetAngle = angle, speed = turnspeed)
        #gyroStraight(distance=6, speed = 35, backward = True, targetAngle = angle)
        flushForTime(speed=-35, timeInSeconds=0.6)
        
        # Reset the gyro since we aligned.
        primeHub.motion_sensor.reset_yaw_angle()
        
    def _hybridCar():
        angle = 0

        # Drive forward a little to be ale to turn
        gyroStraight(distance=2, speed = 20, backward = False, targetAngle = 0)

        # Drive forward towards the hybrid car.
        angle = -97
        _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.2)
        if (_driveTillLine(speed=40, distanceInCM=30, target_angle=angle, colorSensorToUse="Left", blackOrWhite="Black") == False):
            logMessage("NOTE -----------> Missed Catching the line before hybrid car", level=0)
        
        #Bring arm down to get ready for hybrid car
        moveArm(degrees = 140, speed = -50, motor = motorD)
        gyroStraight(distance=2, speed = 25, backward = False, targetAngle = angle)
            
        angle = -80
        _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.2)
        gyroStraight(distance=4, speed = 25, backward = False, targetAngle = angle)
        
        # Lift up the hybrid car.
        moveArm(degrees = 140, speed = 75, motor = motorD)

    def _goHome():
        time.sleep(0.1)
        moveArm(degrees = 140, speed = -25, motor = motorD)

        time.sleep(0.1)
        moveArm(degrees = 140, speed = 100, motor = motorD)

        # Backoff from the hybrid car enough to let the car drop
        angle = -87
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6)
        gyroStraight(distance=50, speed = 40, backward = True, targetAngle = angle)

        angle = -80
        _turnToAngle(targetAngle = angle, speed = 35, slowTurnRatio = 0.6)
        gyroStraight(distance=45, speed = 90, backward = True, targetAngle = angle)

        #angle = -70
        #_turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6)
        #gyroStraight(distance=35, speed = 80, backward = True, targetAngle = angle)

    #moveArm(degrees = 150, speed = 50, motor = motorD)
    _watchTV()
    _getToWindTurbine()
    _windTurbine()
    _pickUpRechargeableBattery()
    _hybridCar()
    _goHome()

#endregion


def _fasterRun2():

    def _doRechargablebattery():
        straightSpeed = 50
        angle = 0
        correction=0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        gyroStraight(distance=33, speed=straightSpeed,targetAngle=angle,backward=False)

        angle = -67
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.4)

        # Start picking up the arm in parallel
        #motorF.start_at_power(-75)
        gyroStraight(distance=25, speed=straightSpeed,targetAngle=angle,backward=False)
        #motorF.stop()

        # Drop off the units. 
        moveArm(degrees = 140, speed = -75, motor = motorF)

        # Backoff from the toy factory.
        gyroStraight(distance=8,speed=straightSpeed,targetAngle=angle,backward=True)   

    def _gotoSmartGrid():
        # This cab be as high as 50.
        straightSpeed = 40
        correction = 0
        angle = 0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction=correction)
        gyroStraight(distance=25, speed=straightSpeed,targetAngle=angle,backward=False)

        if ROBOT == "A":
            # Amogh's robot is newer motors and 90d turn seems to work better with this robot.
            angle = -90
        else:
            angle=-92
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)

        # Intentionally doing a slow turn.
        _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.2, correction=correction)
        if _driveTillLine(speed=straightSpeed, distanceInCM=45, target_angle=angle, colorSensorToUse="Left", blackOrWhite="Black", slowSpeedRatio=1) == False:
            logMessage("Note --------------------> Missed line between hybrid car and toy factory", level=0)

        angle=-110
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction=correction)
        gyroStraight(distance=20,speed=straightSpeed,targetAngle=angle,backward=False)
        if _driveTillLine(speed=straightSpeed, distanceInCM=15, target_angle=angle, colorSensorToUse="Right", blackOrWhite="Black",slowSpeedRatio=1) == False:
            logMessage("Note --------------------> Missed n-s line in front of smart grid", level=0)
        else:
            gyroStraight(distance=4,speed=35,targetAngle=angle,backward=False)
                
    def _doSmartGrid():
        # Turn towards the smart grid
        angle = 0
        correction = 0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        # This turn is very slow intentionally.
        _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.1, correction=correction)

        # First backoff to ensure that we are behind the black line
        #gyroStraight(distance=8, speed=35, targetAngle=angle,backward=True)
        
        # Catch the black line in front of the smart grid
        if _driveTillLine(speed=35, distanceInCM=20, target_angle=angle, colorSensorToUse="Left", blackOrWhite="Black") == False:
            logMessage("Note --------------------> Missed e-w line in front of smart grid", level=0)
        gyroStraight(distance=2, speed=35, targetAngle=angle,backward=False)
        
        # Bring down the arm to pull the smart grid.
        moveArm(degrees = 120, speed = 75, motor = motorF)

        motorF.start_at_power(75)
        gyroStraight(distance=8, speed = 25, backward = True, targetAngle = angle)
        motorF.stop()

        # Pick up the arm again to pick up the water units
        moveArm(degrees = 200, speed = -75, motor = motorF)

    def _picktwoWaterUnits():
        # Turn towards the water reservoir to pick up the two water units
        angle = -100
        correction=0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle=angle, speed=25, slowTurnRatio=0.6, correction=correction)
        gyroStraight(distance=14, speed = 25, backward = False, targetAngle = angle)
       
        # Drop the arm to get the water units.
        moveArm(degrees = 150, speed = 75, motor = motorF)
        gyroStraight(distance=10, speed = 25, backward = True, targetAngle = angle)

    def _pickSolarFarmUnitwithFlushing():
         # Now go to solar farm to pick the energy unit
        angle = 150
        correction=0.02
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, correction = correction)
        #gyroStraight(distance = 8, speed = 20, backward = True, targetAngle = angle)
        _driveBackwardTillLine(distance = 10, speed = 35, target_angle = angle, colorSensorToUse = "Left", blackOrWhite = "Black")
        gyroStraight(distance=7, speed = 35, backward = True, targetAngle = angle)

        # Collect the solar farm unit
        angle = -178
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, correction = correction)
        
        # flush with the wall
        flushForTime(speed=-30, timeInSeconds=1)
        primeHub.motion_sensor.reset_yaw_angle()        

        # Reset the gyro after the flush.
        angle = 0
        if _driveTillLine(speed = 35, distanceInCM = 15, target_angle = angle, colorSensorToUse = "Left", blackOrWhite = "Black") == False:
            logMessage("Note --------------------> Missed e-w line in front of solar farm grid", level=0)
           
        #gyroStraight(distance = 4, speed = 40, backward = False, targetAngle = angle)

    def _goToToyFactory():
        angle = -43
        correction=0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction = correction)
        gyroStraight(distance = 57, speed = 60, backward = False, targetAngle = angle)

        # Flush with the toy factory
        angle=45
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction=correction)

        # Replaced backward gyrostraight with flushfortime for flushing
        # Also dropping the arm in parallel when flushing.
        motorD.start_at_power(-50)
        if (ROBOT == "A"):
            flushForTime(speed=-30, timeInSeconds=1)
        else:
            flushForTime(speed=-30, timeInSeconds=1)
        motorD.stop()
        #gyroStraight(distance=8, speed = 35, backward = True, targetAngle = angle)

    def _goToPowerPlant():
        # Bring arm down to get ready for power plant
        # moveArm(degrees = 120, speed = -50, motor = motorD)
       
        # Reset yaw angle as we are aligned against toyfactory
        resetTotalDegreesTurned()
        primeHub.motion_sensor.reset_yaw_angle()

        angle = 0
        correction=0
        # Move ahaead a little so we can turn
        gyroStraight(distance = 4, speed = 25, backward = False, targetAngle = angle)
        
        # First catch the n-s line infront of the power plant.
        angle = 45
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction=correction)
        if _driveTillLine(speed=40, distanceInCM=20, target_angle=angle, colorSensorToUse="Left", blackOrWhite="Black") == False:
            logMessage("Note --------------------> Missed n-s line in front of power plant", level=0)
        else:
            gyroStraight(distance=4, speed = 25, backward = False, targetAngle = angle)
        
        # Turn left towards the power plant and do the power plant.
        angle = -45
        correction = 0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.1, correction=correction)
        gyroStraight(distance=20, speed = 20, backward = False, targetAngle = angle)
            
    def _doPowerPlant():
        # This is the original code to lift the power plant.
        # Either use this or uncomment the code below.
        '''
        correction = 0
        angle = -45
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        moveArm(degrees = 175, speed = 75, motor = motorD)
        gyroStraight(distance=5, speed = 15, backward = True, targetAngle = angle)
        '''
        # First lift the power plant. Sometimes we are not able to lift
        # we backoff a little and try again before moving on.
        motorD.start_at_power(75)
        wait_for_seconds(0.2)
        motorD.stop()
        angle= -45
        # In case we were stuck, bring down the arm first and then backoff a little and try again.
        moveArm(degrees = 120, speed = -75, motor = motorD)
        gyroStraight(distance = 1, speed = 20, backward=True, targetAngle=angle)
        moveArm(degrees = 175, speed = 100, motor = motorD)
        gyroStraight(distance=5, speed = 15, backward = True, targetAngle = angle)

    def _goHome():
        # Go Home
        if ROBOT == "A":
            angle = 40
        else:
            angle = 40
        correction = 0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 35, slowTurnRatio = 0.6, correction=correction)
        gyroStraight(distance=90, speed = 80, backward = False, targetAngle = angle, multiplier=1.0, gradualAcceleration=False, slowDown=False)

    def _doPowerPlantFromSmartGrid():
        angle = 0
        gyroStraight(distance = 4, speed = 40, backward = False, targetAngle = angle)

        #Turn to get to the n-s line in front of the smart  grid
        angle = -45
        correction=0.03
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6, correction = correction)
        _driveTillLine(speed=30, distanceInCM=15, target_angle=angle, colorSensorToUse="Left", blackOrWhite="Black")
        gyroStraight(distance=3, speed = 30, backward = False, targetAngle = angle)

        # Now drive towards powerplant
        angle = 0
        _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio=0.1)
        moveArm(degrees = 120, speed = -50, motor = motorD)
        gyroStraight(distance = 47, speed = 40, backward = False, targetAngle = angle)
        # Now do the powerplant mission
        moveArm(degrees = 175, speed = 75, motor = motorD)

        # Backup from the power plant
        gyroStraight(distance=5, speed = 15, backward = True, targetAngle = angle)

    def _goHomeAfterStraight():
        # Go Home
        angle = -80
        correction = 0
        angle, correction = calculateReducedTargetAngleAndCorrection(angle, correction)
        _turnToAngle(targetAngle = angle, speed = 35, slowTurnRatio = 0.6, correction=correction)
        gyroStraight(distance=90, speed = 80, backward = False, targetAngle = angle)

    primeHub.motion_sensor.reset_yaw_angle()
    #moveArm(degrees = 150, speed = 50, motor = motorD)
    
    _doRechargablebattery()
    _gotoSmartGrid()
    _doSmartGrid()
    _picktwoWaterUnits()
    _pickSolarFarmUnitwithFlushing()
    _goToToyFactory()
    _goToPowerPlant()
    _doPowerPlant()
    _goHome()
    #_doPowerPlantFromSmartGrid()
    #_goHomeAfterStraight()


#region Function Calls

def _newrun4smallerattachment():
    # Initialize the speed variables for this run.
    turnSpeed = 25
    slowSpeed = 25
    alignSpeed = 35
    fastSpeed = 50
    goHomeSpeed = 75

    # Reset the Gyro Sensor
    primeHub.motion_sensor.reset_yaw_angle()
    #moveArm(degrees = 3000, speed = 75, motor = motorD)
    # Drive towards the Oil Platform
    gyroStraight(distance = 30, speed = slowSpeed, backward = False, targetAngle = 0)

    # Turn to go around the Oil Platform
    angle = 30
    _turnToAngle(targetAngle = angle, speed = turnSpeed, slowTurnRatio = 0.9)
    gyroStraight(distance = 43, speed = slowSpeed, backward = False, targetAngle = angle)

    # Drive towards the Energy Storage
    angle = 0
    _turnToAngle(targetAngle = angle, speed = turnSpeed, slowTurnRatio = 0.9)
    gyroStraight(distance = 35, speed = alignSpeed, backward = False, targetAngle = angle)

    for i in range(3):
        #Raise the oil platform
        moveArm(degrees = 1750, speed = -75, motor = motorF)
        #Lower the oil platform
        moveArm(degrees = 1750, speed = 75, motor = motorF)


    #moveArm(degrees = 3000, speed = -75, motor = motorD)
    # Turn and back up to home
    gyroStraight(distance = 8, speed = goHomeSpeed, backward = True, targetAngle = angle)
    angle = 20
    _turnToAngle(targetAngle = angle, speed = fastSpeed, slowTurnRatio = 0.1)
    gyroStraight(distance = 45, speed = goHomeSpeed, backward = True, targetAngle = angle)
    


    '''
    # Lower the Oil Platform arm so that it fits underneath the Oil Platform lever
    #gyroStraight(distance = 2, speed = slowSpeed, backward = True, targetAngle = angle)
    #moveArm(degrees = 300, speed = 75, motor = motorF)

    # Back up to the Oil Platform so that the Oil Platform arm is under the lever
    backUpDist = 3
    angle = 10
    gyroStraight(distance = backUpDist, speed = slowSpeed, backward = True, targetAngle = angle)
    _turnToAngle(targetAngle = angle, speed = turnSpeed, slowTurnRatio = 0.9)

    # Repeatedly raise and lower the Oil Platform arm to expell the units. Drive forward and backward so that the arm does not end up on top of the lever
    for i in range(3):
        moveArm(degrees = 1500, speed = -100, motor = motorF)
        gyroStraight(distance = backUpDist + 1, speed = slowSpeed, backward = False, targetAngle = primeHub.motion_sensor.get_yaw_angle())
        _turnToAngle(targetAngle = angle, speed = turnSpeed, slowTurnRatio = 0.9)
        moveArm(degrees = 1500, speed = 100, motor = motorF)
        gyroStraight(distance = backUpDist, speed = slowSpeed, backward = True, targetAngle = primeHub.motion_sensor.get_yaw_angle())
    
    # Drive forward towards the Energy Storage
    gyroStraight(distance = 5, speed = slowSpeed, backward = False, targetAngle = angle)

    # Lower the Solar Farm and Energy Storage Tray arms to collect them
    moveArm(degrees = 3000, speed = -75, motor = motorF)

    # Back out of the Energy Storage
    gyroStraight(distance = 15, speed = fastSpeed, backward = True, targetAngle = angle)
    
    # Turn and back up to home
    angle = 30
    _turnToAngle(targetAngle = angle, speed = fastSpeed, slowTurnRatio = 0.1)
    gyroStraight(distance = 40, speed = goHomeSpeed, backward = True, targetAngle = angle)
    '''

def tryPowerPlantWithRun3Arm():
    #testTurnToAngle(speed = 35, slowturnratio=0.6, correction=0.30)
    gyroStraight(distance=11, speed = 20, backward = False, targetAngle = 0)
    moveArm(degrees = 250, speed = 100, motor = motorD)
    gyroStraight(distance=8, speed = 20, backward = True, targetAngle = 0)
    moveArm(degrees = 120, speed = -50, motor = motorF)
    gyroStraight(distance=2, speed = 20, backward = False, targetAngle = 0)
    moveArm(degrees = 120, speed = 50, motor = motorF)
  
    #gyroStraight(distance=3, speed = 20, backward = False, targetAngle = 0)
    #moveArm(degrees = 150, speed = -50, motor = motorF)

def _runhome1tohome2():
    primeHub.motion_sensor.reset_yaw_angle()
    gyroStraight(distance=180, speed = 100, backward = False, targetAngle = 0,multiplier=1)

print("Battery voltage: " + str(hub.battery.voltage())) 
_initialize()
#doRunWithTiming(_run6)
driverWithFewerArms()
raise SystemExit
#endregion

