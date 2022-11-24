# LEGO type:standard slot:0
from spike import PrimeHub, ColorSensor,  Motor, MotorPair
# autostart
from math import *
import collections
# Note that the "hub" import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from spike.operator import *
import gc
import math

# Various robot constants
AXLE_DIAMETER_CM = 12.7
AXLE_DIAMETER_CM_CORRECTED = 12.2
WHEEL_RADIUS_CM = 4.4
GLOBAL_LEVEL = 0
ANYA_RUN_START_OFFSET_TO_MAT_WEST = 0

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

def driver():
    counter = 1
    while True:
        primeHub.speaker.beep(90, 1)
        primeHub.right_button.wait_until_pressed()

        if counter == 1:
            _run1()
        if counter == 2:
            _run1point5()
        if counter == 3:
            _run2()
        if counter == 4:
            _run3()
        if counter == 5:
            _run4()
        if counter == 6:
            _run6()    
        counter = counter + 1

#region Utilities
def _initialize(): 
    print("___________________________________________________")
    primeHub.motion_sensor.reset_yaw_angle()
    wheels.set_stop_action("brake")
    wheels.set_motor_rotation(2*3.14*WHEEL_RADIUS_CM, 'cm')
    isBatteryGood()

class robot:
    currentLocationX = 0
    currentLocationY = 0
    currentRobotAngle = 0

    def __init__(self,x,y):
        self.currentLocationX = x
        self.currentLocationY = y

    def goto(self,x2,y2,endAngle,speed):
        global angle, slope, quadrant2, distance
        angle = 0
        slope = 0
        quadrant2 = 0
        distance = 0
        x1 = self.currentLocationX
        y1 = self.currentLocationY
        a1 = self.currentRobotAngle

        def _calculateSlope(x1,y1,x2,y2):
            global slope
            lineSlope = (y1 - y2)/(x1-x2)
            slope = lineSlope

        def _findQuadrant(x1,y1,x2,y2):
            greatestDistance = 0
            xDiff = x1 - x2
            yDiff = y1 - y2
            maxX = 0
            maxY = 0
            minX = 0
            minY = 0
            if abs(xDiff) > abs(yDiff):
                greatestDistance = abs(xDiff)
            if abs(yDiff) > abs(xDiff):
                greatestDistance = abs(yDiff)
            if abs(xDiff) == abs(yDiff):
                greatestDistance = abs(xDiff)
            else:
                print("URGENT: Error in finding quadrants around robot.")

            maxX = x1 + greatestDistance
            minX = x1 - greatestDistance
            maxY = y1 + greatestDistance
            minY = y1 - greatestDistance
            _findEndQuadrant(x1,y1,x2,y2,maxX,maxY,minX,minY)

        def _findEndQuadrant(x1,y1,x2,y2,maxX,maxY,minX,minY):
            global quadrant2
            endQuadrant = 0
            print(x1,y1,x2,y2,maxX,maxY,minX,minY)
            if x2 >= x1 and x2 <= maxX and y2 >= y1 and y2 <= maxY:
                endQuadrant = 1
            
            if x2 >= minX and x2 <= x1 and y2 >= y1 and y2 <= maxY:
                endQuadrant = 2
            
            if x2 >= minX and x2 <= x1 and y2 >= minY and y2 <= y1:
                endQuadrant = 3
            
            if x2 >= x1 and x2 <= maxX and y2 >= minY and y2 <= y1:
                endQuadrant = 4

            if endQuadrant == 0:
                print("URGENT: Error in finding quadrant of end location")

            quadrant2 = endQuadrant
            
        def _calculateAngle(slope):
            global angle
            lineSlope = slope
            if x1 == 0 and x2 == 0:
                angle = 0
            angleRadians = math.atan(lineSlope)
            angleDegrees = math.degrees(angleRadians)
            angle = round(angleDegrees)

        def _fixAngle(endQuadrant, rAngle):
            global angle
            turnAngle = rAngle

            if endQuadrant == 3:
                turnAngle = -1 * rAngle - 90

            if endQuadrant == 4:
                turnAngle = -1 * rAngle + 90

            angle = round(turnAngle)

        def _findDistance(x1,y1,x2,y2):
            global distance
            distanceToDrive = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
            distance = round(distanceToDrive)
        def _move(speed):
            global angle, distance
            _turnToAngle(targetAngle = angle, speed = speed)
            drive(speed = speed, distanceInCM = distance, target_angle = angle)

        _findQuadrant(x1,y1,x2,y2)
        _calculateSlope(x1,y1,x2,y2)
        _calculateAngle(slope)
        _fixAngle(quadrant2, angle)
        _findDistance(x1,y1,x2,y2)
        _move(speed)
        print(str(angle))
        print(str(distance))
        self.currentLocationX = x2
        self.currentLocationY = y2
        _turnToAngle(targetAngle = endAngle, speed = speed)

def testCoordinateSystem():
    for index in range(len(testX2)):
        robotTest = robot(0,0)
        robotTest.goto(testX2[index], testY2[index], 0, 10)

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

def gyroAngleZeroTo360():
        """
        Returns a number between 0-360. Note that it will not return 180 because the yaw angle is never 180.
        """
        
        yaw = primeHub.motion_sensor.get_yaw_angle()
        if (yaw < 0):
            return 360 + yaw
        else:
            return yaw
        

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
    logMessage("Starting turn", level=4)
    wheels.stop()
    currentAngle = gyroAngleZeroTo360()
    
    if (targetAngle < 0):
        targetAngle = targetAngle + 360
    
    # After this point the targetAngle and the
    # currentAngle is in the 0-360 space for the remainder of this
    # function until we call the helper function.
    #logMessage("TurnToAngle current_angle:" + str(currentAngle) + " targetAngle (360 space):" + str(targetAngle), level=4)
    logMessage("TurnToAngle current_angle: {} targetAngle (360 space): {} ".format(str(currentAngle),  str(targetAngle)), level=4)
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
    if (abs(degreesToTurn) > 20):
        reducedTargetAngle = targetAngle - (degreesToTurn * correction)
        degreesToTurn = degreesToTurn * (1-correction)

    # Put the target angle back in -179 to 179 space.    
    reducedTargetAngleIn179Space = reducedTargetAngle
    if (targetAngle >= 180):
        reducedTargetAngleIn179Space = reducedTargetAngle - 360
    '''
    logMessage("TargetAngle(360 space, reduced)=" + str(reducedTargetAngle) 
        + " TargetAngle(-179 to +179 space, reduced)=" + str(reducedTargetAngleIn179Space)
        + " direction:" + direction, level=4)
    '''
    logMessage("TargetAngle(360 space, reduced)= {} TargetAngle(-179 to +179 space, reduced)= {}  direction: {}".format(str(reducedTargetAngle), str(reducedTargetAngleIn179Space),direction), level=4)
    
    _turnRobotWithSlowDown(degreesToTurn, reducedTargetAngleIn179Space, speed, slowTurnRatio, direction, oneWheelTurn=oneWheelTurn)
    
    currentAngle = gyroAngleZeroTo360()
    #logMessage("TurnToAngle complete current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=4)
    logMessage("TurnToAngle complete current_angle: {}  targetAngle: {} ".format(str(currentAngle),str(targetAngle)), level=4)
    

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
    
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    
    # First we will do a fast turn at speed. The amount to turn is 
    # controlled by the slowTurnRatio.
    turnRobot(direction, speed, oneWheelTurn)

    fastTurnDegrees =  (1 - slowTurnRatio) * abs(angleInDegrees)
    while (abs(currentAngle - targetAngle) > fastTurnDegrees):
        time.sleep_ms(7)
        currentAngle = primeHub.motion_sensor.get_yaw_angle()    

    # After the initial fast turn that is done using speed, we are going to do a 
    # slow turn using the slow speed.
    turnRobot(direction, SLOW_SPEED, oneWheelTurn)
    # if (direction == "Right"):
    #     wheels.start_tank(SLOW_SPEED, SLOW_SPEED * -1)
    # if (direction == "Left"):
    #     wheels.start_tank(SLOW_SPEED * -1, SLOW_SPEED)

    while (abs(currentAngle - targetAngle) > 2):
        time.sleep_ms(7)
        currentAngle = primeHub.motion_sensor.get_yaw_angle()

    wheels.stop()
   
    """
    motorCDiff = abs(motorC.get_degrees_counted()) - motorCInitialDeg
    motorEDiff = abs(motorE.get_degrees_counted()) - motorEInitialDeg
    avgDiff = (abs(motorCDiff) + abs(motorEDiff)) / 2
    robotTurn = 0.346*avgDiff
    logMessage("In SlowTurn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed) + " motorCDegDiff=" + 
        str(motorCDiff) + " motorEDegDiff=" + str(motorEDiff) + " expectedRobotTurn=" + str(robotTurn), level=5)  

    """
    
def turnRobot(direction, speed, oneWheelTurn):
    if (oneWheelTurn == "None"):
        if (direction == "Right"):
            wheels.start_tank(speed, speed * -1)
        if (direction == "Left"):
            wheels.start_tank(speed * -1, speed)
    elif (oneWheelTurn == "Left"):
        left_large_motor.start(speed)
    else:
        right_large_motor.start(speed)

def gyroStraight(distance, speed = 20, backward = False, targetAngle = 0):
    initialDeg = abs(motorE.get_degrees_counted())
    if(distance < _CM_PER_INCH*3):
        _gyroStraightNoSlowDownNoStop(distance = distance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
        wheels.stop()
        return
    
    gradualAccelerationDistance = _CM_PER_INCH*1
    slowDistance = 0.2 * distance
    if(slowDistance > _CM_PER_INCH*2):
        slowDistance = _CM_PER_INCH*2
    _gyroStraightNoSlowDownNoStop(distance = gradualAccelerationDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    _gyroStraightNoSlowDownNoStop(distance = distance - slowDistance - gradualAccelerationDistance, speed = speed, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    _gyroStraightNoSlowDownNoStop(distance = slowDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    wheels.stop()

    finalDeg = abs(motorE.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = {} error = {}".format(str(totalDistanceTravelled), str(distance-totalDistanceTravelled)), level=4)
    logMessage("======== gyroStraight done for distance {}".format(str(distance)), 4)

def _gyroStraightNoSlowDownNoStop(distance, speed = 20, backward = False, targetAngle = 0, correctionMultiplier = 2):
    initialDeg = abs(motorE.get_degrees_counted())

    underBiasErrorMultiplier = 1 # 1.106
    errorAdjustedDistanceInCm = distance*underBiasErrorMultiplier

    logMessage("GYROSTRAIGHT START: targetAngle  is {}".format(str(targetAngle)), level=4)
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
            correction = getCorrectionForDrive(targetAngle, correctionMultiplier = correctionMultiplier) # targetAngle - currentAngle
            wheels.start(steering = correction, speed=speed)   

def turnToAngle2(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.4, correction=0.05, oneWheelTurn="None"):
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
    if(oneWheelTurn is not "None"):
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
        if(oneWheelTurn is "Left" or oneWheelTurn is "None"):
            leftSpeed = speed
            turnLeftRemaining = degreesOfRotation
            # print("Left Turn: Right speed=" + str(rightSpeed) + ", Left Remaining=" + str(turnRightRemaining))
        if(oneWheelTurn is "Right" or oneWheelTurn is "None"):
            rightSpeed = speed
            turnRightRemaining = degreesOfRotation
            # print("Left Turn: Right speed=" + str(rightSpeed) + ", Right Remaining=" + str(turnRightRemaining))
    if(direction == "Right"):
        if(oneWheelTurn is "Left" or oneWheelTurn is "None"):
            leftSpeed = -1*speed
            turnLeftRemaining = degreesOfRotation
            # print("Right Turn: Left speed=" + str(leftSpeed) + ", Left Remaining=" + str(turnLeftRemaining))
        if(oneWheelTurn is "Right" or oneWheelTurn is "None"):
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
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    logMessage("CurrentAngle: " + str(currentAngle) + " and targetAngle: " + str(targetAngle), 5)
    if( (currentAngle <= 0 and targetAngle <=0) or
            (currentAngle>=0 and targetAngle > 0) or
            (abs(currentAngle) <= 90 and abs(targetAngle)<=90)):
        correction = targetAngle - currentAngle
    elif (currentAngle >= 90):
        correction = (360 - abs(currentAngle) - abs(targetAngle))
    else:
        correction = -1*(360 - abs(currentAngle) - abs(targetAngle))

    logMessage("Correction needed = {}".format(str(correction)), 4)
    return correction * correctionMultiplier

def testGyro():
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*16)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward =True)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward=True)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3, backward=True)
  

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

def _calculateDistanceTravelled(motorCInitialDeg, motorDInitialDeg):
    motorCDegTravelled = abs(motorC.get_degrees_counted()) - motorCInitialDeg
    motorDDegTravelled = abs(motorD.get_degrees_counted()) - motorDInitialDeg
    avgDistanceTravelled = (motorCDegTravelled + motorDDegTravelled) / 2
    return avgDistanceTravelled

def driveWithSlowStart(speed, distanceInCM, target_angle, gain = 1, dontSlowDown=False):
    """
    Drive
    _____
    This function drives the robot FORWARD using the motion sensor and the 80-15-5 formula.
    80% of distance at speed
    15% of distance with linear slow down from speed to 10
    5% of distance with speed of FINAL_SLOW_SPEED.
    _____
    Speed - Speed the wheels travel at. Integer from -100 to 100
    DistanceInCM - Distance to travel in centimeters. Integer greater than 0
    TargetAngle - The angle the robot should drive at. Integer from 0 to 360
    Gain - The multiplier off the error. Integer greater than 0
    dontSlowDown - Set this to true to run at speed all the way. Typically used for going home. 
    """
    
    #wheels.move_tank(distanceInCM, "cm", speed, speed)
    #return
    
    wheels.stop()
    logMessage("driveStraight for distance: {} and target angle: {}".format(str(distanceInCM),str(target_angle)), level=2)
    
    #Added to try the fix for negative degreescounted and high error rate
    FINAL_SLOW_SPEED = 20
    initialDeg = abs(motorC.get_degrees_counted())
    remainingDistance = distanceInCM
    
    # If the distance is small, then just drive over that distance at speed.
    if (distanceInCM < 5):
        _driveStraightWithSlowDown(distanceInCM, speed, target_angle, gain, slowDown=False)    

    # Setup the multiplier for the initial distance to drive at speed.
    # If we pass in dontSlowDown = True, typically for going Home, then 
    # this code will run home all the way at Speed.
    if (dontSlowDown == False):
        initialMultiplier = 0.6
    else:
        initialMultiplier = 1.0

    # First drive 10% of the distance at 20 to start slow
    distance10 = distanceInCM * 0.1
    _driveStraightWithSlowDown(distance10, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False)

    # First drive 60% of the remaining distance at speed
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    distance80 = remainingDistance * initialMultiplier
    _driveStraightWithSlowDown(distance80, speed, target_angle, gain, slowDown=False)
    
    # This code is commented for now, the above code seems to be providing the best algorithm
    # We will reevaluate if needed.
    # Drive the next 16% of the distance at a speed that reduces from speed to speed=FINAL_SLOW_SPEED
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("Distance travelled after first part = {} error={}".format(str(distanceTravelled),str(distance80-distanceTravelled)), level=4)
    logMessage("remainingDistance after 1st travel={}".format(str(remainingDistance)), level=2)

    # Only run this whole thing in case there is any remaining distance.
    # this check is meant to catch the case when for some reason
    # the currently travelled distance returned by motors are wrong for some
    # reason.
    if (remainingDistance > 0):
        distance16 = remainingDistance * 0.6
        _driveStraightWithSlowDown(distance16, speed, target_angle, gain, slowDown=True)
        
        distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
        remainingDistance = distanceInCM - distanceTravelled
        logMessage("Distance travelled after second part= {} error={}".format(str(distanceTravelled),str((distance80+distance16)- distanceTravelled)), level=4)
        logMessage("remainingDistance after 2nd travel={}".format(str(remainingDistance)), level=2)
        
        # Drive the final 4% of the distance at speed 5
        # If we have already overshot the target, then dont correct for it.
        if remainingDistance > 1:
            _driveStraightWithSlowDown(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False)
            #motors.move_tank(remainingDistance, "cm", FINAL_SLOW_SPEED, FINAL_SLOW_SPEED)
    
    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())
    wheels.set_stop_action("brake")

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Drive: Total distance travelled = {} error={}".format(str(totalDistanceTravelled),str(distanceInCM-totalDistanceTravelled)), level=2)


def drive(speed, distanceInCM, target_angle, gain = 1, dontSlowDown=False):
    """
    Drive
    _____
    This function drives the robot FORWARD using the motion sensor and the 80-15-5 formula.
    80% of distance at speed
    15% of distance with linear slow down from speed to 10
    5% of distance with speed of FINAL_SLOW_SPEED.
    _____
    Speed - Speed the wheels travel at. Integer from -100 to 100
    DistanceInCM - Distance to travel in centimeters. Integer greater than 0
    TargetAngle - The angle the robot should drive at. Integer from 0 to 360
    Gain - The multiplier off the error. Integer greater than 0
    dontSlowDown - Set this to true to run at speed all the way. Typically used for going home. 
    """
    
    #wheels.move_tank(distanceInCM, "cm", speed, speed)
    #return
    
    wheels.stop()
    logMessage("driveStraight for distance:{} and target angle:{}".format(str(distanceInCM),str(target_angle)), level=2)
    #Added to try the fix for negative degreescounted and high error rate
    
    initialDeg = abs(motorC.get_degrees_counted())
    remainingDistance = distanceInCM
    
    # If the distance is small, then just drive over that distance at speed.
    if (distanceInCM < 5):
        _driveStraightWithSlowDown(distanceInCM, speed, target_angle, gain, slowDown=False)    

    # Setup the multiplier for the initial distance to drive at speed.
    # If we pass in dontSlowDown = True, typically for going Home, then 
    # this code will run home all the way at Speed.
    if (dontSlowDown == False):
        initialMultiplier = 0.6
    else:
        initialMultiplier = 1.0

    # First drive 60% of the distance at speed
    distance80 = distanceInCM * initialMultiplier
    _driveStraightWithSlowDown(distance80, speed, target_angle, gain, slowDown=False)
    
    # This code is commented for now, the above code seems to be providing the best algorithm
    # We will reevaluate if needed.
    # Drive the next 16% of the distance at a speed that reduces from speed to speed=FINAL_SLOW_SPEED
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("Distance travelled after first part={} error={}".format(str(distanceTravelled), str(distance80-distanceTravelled)), level=4)
    logMessage("remainingDistance after 1st travel={}".format(str(remainingDistance)), level=2)

    # Only run this whole thing in case there is any remaining distance.
    # this check is meant to catch the case when for some reason
    # the currently travelled distance returned by wheels are wrong for some
    # reason.
    if (remainingDistance > 0):
        distance16 = remainingDistance * 0.6
        _driveStraightWithSlowDown(distance16, speed, target_angle, gain, slowDown=True)
        
        distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
        remainingDistance = distanceInCM - distanceTravelled
        logMessage("Distance travelled after second part={} error={}".format(str(distanceTravelled), str((distance80+distance16)- distanceTravelled)), level=4)
        logMessage("remainingDistance after 2nd travel={}".format(str(remainingDistance)), level=2)
        
        # Drive the final 4% of the distance at speed 5
        # If we have already overshot the target, then dont correct for it.
        if remainingDistance > 1:
            FINAL_SLOW_SPEED = 15
            _driveStraightWithSlowDown(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False)
            #wheels.move_tank(remainingDistance, "cm", FINAL_SLOW_SPEED, FINAL_SLOW_SPEED)
    
    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Drive: Total distance travelled = {} error={}".format(str(totalDistanceTravelled),str(distanceInCM-totalDistanceTravelled)), level=2)

def _driveStraightWithSlowDown(distance, speed, target_angle, gain, slowDown):
    """
    Drive Straight
    ______________
    This is a internal function do not call directly. Call drive instead.

    The algorithm goes from speed to FINAL_SLOW_SPEED until distance is travelled.
    slowDown: True if you want to slow down over the distance. 
    """
    startDistanceInDeg = abs(motorC.get_degrees_counted())
    logMessage("startDistanceInDeg = " + str(int(startDistanceInDeg)), level=5)
    distanceInDeg = converCMToDeg(distance)
    currentSpeed = speed

    if (target_angle == -180):
        target_angle = 180

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0

    FINAL_SLOW_SPEED = 15
    wheels.start(0, int(currentSpeed))
    correction = previousCorrection = 0
    while  distanceInDegTravelled <= distanceInDeg:
        if (slowDown == True):
            currentSpeed = currentSpeed - 1
            if (currentSpeed < 15):
                currentSpeed = 15
       
        current_yaw_angle = primeHub.motion_sensor.get_yaw_angle()

        # This hackery is needed to handle 180 or -180 straight run.
        if (target_angle == 180 and current_yaw_angle < 0):
            current_yaw_angle = (360 + current_yaw_angle)

        previousCorrection = correction
        correction = target_angle - current_yaw_angle
        
        turn_rate = correction * gain
        logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelledInCM = " + str(convertDegToCM(distanceInDegTravelled)) + " distanceInCM=" + str(distance) 
          + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceToTravelInDeg=" + str(distanceInDeg) 
         + " target_angle= " + str(target_angle) + " current_yaw_angle = " + str(current_yaw_angle) +" correction= " + str(correction), level=5)

        logMessage("currentSpeed={}  distanceInDegTravelledInCM={} distanceInCM={}  distanceInDegTravelled={} distanceToTravelInDeg={}  target_angle={} current_yaw_angle={} correction={}".format(
          str(int(currentSpeed)), str(convertDegToCM(distanceInDegTravelled)), str(distance),
          str(distanceInDegTravelled), str(distanceInDeg), str(target_angle), str(current_yaw_angle), str(correction)), level=5)

        if (abs(correction) > 1):
            wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
    logMessage("Drivestraight completed", level=5)


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
    assert(colorSensorToUse == "Left" or colorSensorToUse == "Right")
    assert(blackOrWhite == "Black" or blackOrWhite == "White" or blackOrWhite == "Green")

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
        #stoppingCondition = lambda: colorSensor.get_reflected_light() <= BLACK_COLOR
        def blackStoppingCondition():
            light = colorSensor.get_reflected_light()
            logMessage(" color={}".format(str(light)), level=5)
            return light <= BLACK_COLOR
        stoppingCondition = blackStoppingCondition
    elif (blackOrWhite == "White"):
        #stoppingCondition = lambda: colorSensor.get_reflected_light() >= WHITE_COLOR
        def whiteStoppingCondition():
            light = colorSensor.get_reflected_light()
            logMessage(" color={}".format(str(light)), level=5)
            return light >= WHITE_COLOR
        stoppingCondition = whiteStoppingCondition
    elif (blackOrWhite == "Green"):
        stoppingCondition = lambda: colorSensor.get_color() == 'green'

    FINAL_SLOW_SPEED=10
    # If the distance is small, then just drive over that distance at FINAL_SLOW_SPEED.
    if (distanceInCM < 5):
        _driveStraightWithSlowDownTillLine(distanceInCM, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)    
    else:
        # First drive 60% of the distance at speed
        distance60 = distanceInCM * slowSpeedRatio
        if _driveStraightWithSlowDownTillLine(distance60, speed, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition) == False:
            # Drive the remaining distance at slow speed
            distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
            remainingDistance = distanceInCM - distanceTravelled
            logMessage("_driveTillLine: Distance travelled after first part = {} error={}".format(str(distanceTravelled),str(distanceTravelled-distance60)), level=4)
            _driveStraightWithSlowDownTillLine(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)

    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("_driveTillLine: Total distance travelled={} error={}".format(str(totalDistanceTravelled), str(totalDistanceTravelled-distanceInCM)), level=2)
    
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
        logMessage("Left color={} Right color={} currrentSpeed={} distanceInDegTravelledInCM={} distanceInCM={} distanceInDegTravelled={} distanceToTravelInDeg={} target_angle={} current_yaw_angle={} correction={}".format(
            str(colorA.get_reflected_light()), str(colorB.get_reflected_light()), str(int(currentSpeed)), str(convertDegToCM(distanceInDegTravelled)), str(distance),
            str(distanceInDegTravelled), str(distanceInDeg), str(target_angle), str(current_yaw_angle), str(correction)), level=5)

        if (abs(correction) > 1):
            wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
        stopCondition = reachedStoppingCondition()

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
    
 
def _testTurnToAngle():
    global GLOBAL_LEVEL
    # TurnToAngle Testing
    
    t1_start = time.ticks_ms()
    _turnToAngle(targetAngle = 90, speed = 25)
    t1_end = time.ticks_ms()
    print("Time taken to turn 0 to 90 degrees: " + str( time.ticks_diff(t1_end,t1_start)) + " milliseconds")
    time.sleep(1)
   
    t2_start = time.ticks_ms()    
    _turnToAngle(targetAngle = 179, speed = 25)
    t2_end = time.ticks_ms()
    print("Time taken to turn 90 to 180 degrees: " + str( time.ticks_diff(t2_end,t2_start)) + " milliseconds")
    time.sleep(1)
    
    t3_start = time.ticks_ms()  
    _turnToAngle(targetAngle = -90, speed = 25)
    t3_end = time.ticks_ms()
    print("Time taken to turn 180 to -90 degrees: " + str( time.ticks_diff(t3_end,t3_start)) + " milliseconds")
    time.sleep(1)
   
    t4_start = time.ticks_ms()  
    _turnToAngle(targetAngle = 0, speed = 25)
    t4_end = time.ticks_ms()
    print("Time taken to turn -90 to 0 degrees: " + str( time.ticks_diff(t4_end,t4_start)) + " milliseconds")
    time.sleep(1)

    _turnToAngle(targetAngle = -90, speed = 25)
    time.sleep(1)
    _turnToAngle(targetAngle = -179, speed = 25)
    time.sleep(1)
    _turnToAngle(targetAngle = 90, speed = 25)
    time.sleep(1)
    _turnToAngle(targetAngle = 0, speed = 25)
   
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
    print("Running now getToOilPlatform")
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

def line_follow(distance, speed = 20):
    targetThreshold = 50
    correctionMultiplier = 0.1
    history = list()

    Kp = 0.5 #  the Constant 'K' for the 'p' proportional controller

    integral = 0 # initialize
    Ki = 0.05 #  the Constant 'K' for the 'i' integral term
    Kd = 0.75 #  the Constant 'K' for the 'd' derivative term
    lastError = 0
    i=0
    degreesToCover = (distance * 360)/(WHEEL_RADIUS_CM * 2 * 3.1416)
    position_start = motorE.get_degrees_counted()

    while i<300:
        i=i+1
        sensorVal = leftColorSensor.get_reflected_light()
        error = sensorVal - targetThreshold
        if (error == 0 or (error * lastError) < 0):
            integral = 0
        else:
            integral = integral + error 

        derivative = error - lastError
        lastError = error

        correction = (Kp*(error) + Ki*(integral) + + Kd*derivative)
        truncCorrection = max(min(correction, speed), -1*speed)

        print("Sensor = " + str(sensorVal) + ", Error=" + str(error) + ", integral=" + str(integral) + ", derivative=" + str(derivative) + ", correction=" + str(correction) + ", trunCorrection=" + str(truncCorrection) + ", time=" + str(time.time()))

        # history.append(error)
        # if(len(history) > 5):
        #     history.pop(0)
        # cumError = calcCumError(history)
        # print(str(time.time()) + ": error=" + str(error) + ", history=" + str(history) + ", sensor=" + str(leftColorSensor.get_reflected_light()))
        # # correction = min(error * correctionMultiplier, speed)
        # correction = cumError*correctionMultiplier
        # wheels.start_tank(int(speed+truncCorrection), int(speed-truncCorrection))
        wheels.start(speed=speed, steering=int(correction))
        time.sleep(0.01)
        if ((motorE.get_degrees_counted() - position_start)  >= degreesToCover):
            break
       
    wheels.stop()
    finalDeg = abs(motorE.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - position_start)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distance-totalDistanceTravelled), level=4)

def calcCumError(history: list):
    cumError = 0
    index = 10
    for error in history:
        cumError = cumError + (error*index)
        index = index+10
    print("CumError Sum=" + str(cumError) + ", avg = " + str(cumError/150))
    return cumError/150



def testingGyroStraight():
    for i in range(5):# should go to range 50 cm
        gyroStraight(targetAngle = 0,  distance = 8*_CM_PER_INCH, speed=60)
        gyroStraight(targetAngle = 0,  distance = 2*_CM_PER_INCH, speed=40)
        time.sleep(1)
#endregion Arisha

#region Anya 
#powerPlant
def _run2():
    primeHub.motion_sensor.reset_yaw_angle()
    print("Battery voltage: " + str(hub.battery.voltage()))
    getToPowerPlantFromHome2()
    ReleaseEnergyUnitsFromPowerPlant()
    goToHome1()

def goToHome1():
    wheels.move(amount = 4.5, unit = "in", steering = 0, speed = -40)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_WEST - 15, slowTurnRatio=0.8)#used to be -105 then most recently was -25
    # motors.move(amount = 35, unit = "in", steering = 0, speed = 90)#original speed 40
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 15,  distance = _CM_PER_INCH*35, speed=90) #used to be -105


def goToHome1_MissingUnit():
    wheels.move(amount = 5, unit = "in", steering = 0, speed = -40)
    # _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135)#original value -90
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = _CM_PER_INCH*8, speed=90)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_WEST - 20)
    # wheels.move(amount = 35, unit = "in", steering = 0, speed = 90)#original speed 40
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 20,  distance = _CM_PER_INCH*35, speed=90)

def getToPowerPlantFromHome2():
    # Lower the power plant arm
    #motorD.run_for_degrees(degrees=-170, speed=50)

    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 0,  distance = _CM_PER_INCH*32, speed=60) # was 29 and then 2 more at 40 speed below
    motorD.start(-30)
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*6, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 0, blackOrWhite="White")

    #ToyFactory2()
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 0,  distance = int(AXLE_DIAMETER_CM/2))

    #motorD.run_for_degrees(degrees=-170, speed=100)
    motorD.stop()
    _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_WEST - 93, speed=20, slowTurnRatio=0.8)
    # _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*6, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 90, blackOrWhite="Black")
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 93,  distance = _CM_PER_INCH*9, speed=20) #was 60 but rammed into power plant

    # Sometimes our unit collector is too far from the slide, so this turn should better align it
    left_large_motor.run_for_degrees(degrees=-45, speed=60)


def ToyFactory2():
    # Align for the toy factory
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_WEST - 15, oneWheelTurn="Left", slowTurnRatio=0.8)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 15,  distance = 5, backward = True)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_WEST - 45, oneWheelTurn="Left", slowTurnRatio=0.8)

    # Do the Toy Factory
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 45,  distance = 15, backward = True) # was 8

    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 45,  distance = 8)
    turnToAngle2(ANYA_RUN_START_OFFSET_TO_MAT_WEST - 0, slowTurnRatio=0.6)

    # Align for the Power Plant
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*9, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 0, blackOrWhite="White")

def ReleaseEnergyUnitsFromPowerPlant():
    # gyroStraight(distance = 4, speed = 35, backward = True, targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 90)
    motorD.run_for_degrees(degrees=180, speed=80)
    # _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*2, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_WEST - 90, blackOrWhite="White")
    # motorD.run_for_degrees(degrees=-120, speed=60)

    
    # motorD.run_for_degrees(degrees=200, speed=100) #was 250

    # motorD.run_for_degrees(degrees=-250, speed=100)

    #endregion Anya 

#region Nami    

def sliderArmandrun4():
    """
    Missions:
    - Hydroelectric Dam
    - Collect Water Units(For Water Reservoir)
    - Solar Farm
    """
    # The Run: 
    wheels.set_stop_action("coast") # Change the settings so that the robot coasts for smoother braking
    #drive(speed = 35, distanceInCM = 19, target_angle = 0) # Drive forward and catch the first water unit
    gyroStraight(distance = 19.5, speed = 35, backward = False, targetAngle = 0)
    #Changed on 10/19/2022 from 40deg to 45 deg
    _turnToAngle(targetAngle = 40, speed = 15, slowTurnRatio = 0.9) # Turn to go around the Hydroelectric Dam

    drive(speed = 35, distanceInCM = 30, target_angle = 40) # Drive forward so that the robot is in front of the line running from the Power Plant to the Smart Grid. Complete the Hydroelectric Dam in the process
    _driveTillLine(speed = 35, distanceInCM = 25, target_angle = 40, colorSensorToUse = "Right", blackOrWhite = "Black", slowSpeedRatio = 0.9) # Catch the line that runs from the Power Plant to the Smart Grid
    drive(speed = 35, distanceInCM = 15, target_angle = 40) # Drive forward so that the robot is in front of the Toy Factory

    _turnToAngle(targetAngle = -40,speed = 15, slowTurnRatio = 0.9) # Turn towards the Smart Grid
    _driveTillLine(speed = 35, distanceInCM = 100, target_angle = -40, colorSensorToUse="Right", blackOrWhite="White", slowSpeedRatio=0.9) # Drive forward to the East - West line in front of the Smart Grid

    gyroStraight(distance=6, speed = 20, backward = True, targetAngle = -40) # Drive backward so that the robot is in parallel with the Water Reservoir
    #Changed from -142 mto -127 on 10/21/2022
    _turnToAngle(targetAngle = -125, speed = 20, slowTurnRatio = 0.9) # Turn towards the Water Reservoir
    _driveTillLine(speed = 35, distanceInCM = 10, target_angle = -142, colorSensorToUse="Left", blackOrWhite="Black", slowSpeedRatio=0.9) # Catch the line in front of the Water Reservoir
    
    drive(speed = 35, distanceInCM = 10, target_angle = -142) # Drive forward and catch the final two water units
    
    _turnToAngle(targetAngle = -70, speed = 25, slowTurnRatio = 0.9) # Turn towards the Solar Farm
    _driveTillLine(speed = 35, distanceInCM = 15, target_angle = -70, colorSensorToUse = "Right", blackOrWhite = "Black") # Drive to the East - West line in front of the Solar Farm
    drive(speed = 25, distanceInCM = 5, target_angle = -70) # Drive to right in front of the first Solar Farm energy unit 
    _turnToAngle(targetAngle = -120, speed = 25,slowTurnRatio=0.9) # Turn so that the robot is in parallel with the Solar Farm
    drive(speed = 25, distanceInCM = 6, target_angle = -120) # Drive forward so that the Solar Farm arm is aligned at the gap between the Solar Farm energy units
    _sliderArm() # Catch the 3 Solar Farm energy units
    # _turnToAngle(targetAngle = -177, speed = 25) # Turn towards the home area
    # drive(speed = 70, distanceInCM = 35, target_angle = -179) # Drive home

def _sliderArm(targetAngle):
    # Pick up first unit
    moveArmDistance = 600
    moveArm(degrees = moveArmDistance, speed = 100, motor=motorD)
    gyroStraight(distance=13, speed = 20, backward = True, targetAngle = targetAngle)
    moveArm(degrees = -1 * moveArmDistance, speed = -100, motor=motorD)

    # open arm again, and then drive forward to pick up last two units.
    drive(speed=25, distanceInCM=10, target_angle= targetAngle)
    moveArm(degrees = moveArmDistance, speed = 100, motor=motorD)
    drive(speed=25, distanceInCM=15, target_angle= targetAngle)

    moveArm(degrees = -1 * moveArmDistance, speed = -100, motor=motorD)

def _run1_5():
    primeHub.motion_sensor.reset_yaw_angle()
    gyroStraight(distance=35, speed = 100, backward = False, targetAngle = 0)
    gyroStraight(distance=50, speed = 100, backward = True, targetAngle = 0)

def _run3():
    primeHub.motion_sensor.reset_yaw_angle()
    wheels.set_stop_action("coast")
    # Drive till the hydro plant to pick up the first water unit
    gyroStraight(distance=24, speed = 40, backward = False, targetAngle = 0)
    
    # Turn towards the hydro unit to drop the water unit from the hydro plant.
    angle = 38
    _turnToAngle(targetAngle= angle, speed = 15, slowTurnRatio=0.9)
    
    # Turns towards the n-s black line in front of the power station.
    gyroStraight(distance=35, speed = 55, backward = False, targetAngle = angle)
    _turnToAngle(targetAngle= angle, speed = 25, slowTurnRatio=0.9)
    _driveTillLine(speed=55, distanceInCM=25, target_angle = angle, colorSensorToUse="Right", blackOrWhite="Black", slowSpeedRatio=0.9)
    gyroStraight(distance=2, speed = 25, backward = False, targetAngle = angle)
    
    angle = -46
    # Turn towards the smart grid and align with the power plant.
    _turnToAngle(targetAngle=angle, speed=20, slowTurnRatio=0.9)
    gyroStraight(distance=10, speed = 25, backward = True, targetAngle = angle)
    primeHub.motion_sensor.reset_yaw_angle()

    # Turn towards the smart grid and drive forward we do fast run till the hydro unit, and then a slow drive forward
    # in order to catch the units. 
    angle = 0
    gyroStraight(distance=2, speed = 25, backward = False, targetAngle = angle)
    _turnToAngle(targetAngle=angle, speed=20, slowTurnRatio=0.9)
    gyroStraight(distance=32, speed = 55, backward = False, targetAngle = angle)

    # Drive to catch the e-w line in front of the smart grid.
    _turnToAngle(targetAngle=angle - 35, speed=20, slowTurnRatio=0.9)
    gyroStraight(distance=15, speed = 35, backward = False, targetAngle = angle - 35)
    _driveTillLine(speed=25, distanceInCM=25, target_angle=angle - 35, colorSensorToUse="Right", blackOrWhite="White")

    # Drive forward till the wall
    _turnToAngle(targetAngle=angle, speed=20, slowTurnRatio=0.9)
    gyroStraight(distance=13, speed = 35, backward = False, targetAngle = angle)
    
    # Back off a little bit before turning to go home.
    gyroStraight(distance=2, speed = 35, backward = True, targetAngle = angle)

    # Go home first do a two part turn.
    _turnToAngle(targetAngle=-60, speed = 15, slowTurnRatio=0.9)
    gyroStraight(distance=8, speed = 25, backward = False, targetAngle = -60)
    
    # Go Home
    _turnToAngle(targetAngle=-100, speed = 15, slowTurnRatio=0.9)
    gyroStraight(distance=25, speed = 60, backward = False, targetAngle = -100)
    _turnToAngle(targetAngle=-135, speed = 15, slowTurnRatio=0.9)
    gyroStraight(distance=90, speed = 95, backward = False, targetAngle = -135)
    
def _run6():
    primeHub.motion_sensor.reset_yaw_angle()
    # Drive forward first. Drive at a slight angle to avoid hitting the power plant.
    drive(speed = 65, distanceInCM = 50, target_angle = -8)
    
    # Turn slightly to catch the n-s line in front of the power plant
    _driveTillLine(speed=30, distanceInCM=35, target_angle=-8, colorSensorToUse="Left", blackOrWhite="Black")
    
    # Turn towards the power plant and then try to catch the black line running e-w line in front of the smart grid.
    angle = -87
    _turnToAngle(targetAngle = angle, speed = 20, slowTurnRatio = 0.7)
    _driveTillLine(speed=45, distanceInCM=70, target_angle = angle, colorSensorToUse="Left", blackOrWhite="Black")
    gyroStraight(distance=5, speed = 20, backward = True, targetAngle = angle)

    # Turn towards the hydro-electric plant and then drop the water units. 
    # This also drops the energy units and the innovation project.
    _turnToAngle(targetAngle = 162, speed = 20, slowTurnRatio = 0.9)
    drive(speed = 25, distanceInCM = 26, target_angle = 162)

    # Drop the water units    
    moveArm(degrees = 2200, speed = -100, motor = motorF)
    
    # Start bringing up the arm
    motorF.start_at_power(60)
    
    # Backoff to leave the water reservoir
    gyroStraight(distance=8, speed = 20, backward = True, targetAngle = 162)

    motorF.stop()    
    _doToyFactory()

def _doToyFactory():
    # turn to get to the toyfactory.
    _turnToAngle(targetAngle = -165, speed = 20, slowTurnRatio = 0.9)
    gyroStraight(distance=24, speed = 35, backward = True, targetAngle = -165)
    _turnToAngle(targetAngle = -120, speed = 20, slowTurnRatio = 0.9)
    gyroStraight(distance=15, speed = 35, backward = True, targetAngle = -120)

    # Back into the toy factory and align

    # Drop off the units.
    moveArm(degrees = -2000, speed = 100, motor = motorD)
    #moveArm(degrees = 1000, speed = 100, motor = motorD)
    
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

def _toyFactoryN():

     # Since the zero is now set to the toy factory we add zero_adjustment to all angles
     # to adjust to the new reference frame.
     zero_adjustment = -140

     # Start bringing up the arm
     motorF.start_at_power(60)
    
     # Backoff to leave the water reservoir
     gyroStraight(distance=13, speed = 20, backward = True, targetAngle = 155)

     # Uncomment this for a serial run.
     #moveArm(degrees = 3000, speed = 100, motor = motorF)

     motorF.stop()    

     _turnToAngle(targetAngle=30, speed= 30)
     gyroStraight(distance=30, speed = 20,  targetAngle = 0)
     moveArm(degrees = -1000, speed = 100, motor = motorD)
     moveArm(degrees = 1000, speed = 100, motor = motorD)


#endregion Nami

#region Rishabh
def _run1():
    def _watchTV():
        # Drive to Watch Television. We do this in two parts. First fast and
        # then do it slow so the energy unit does not fall off.
        gyroStraight(distance = 28, speed = 50, backward = False, targetAngle = 0)
        gyroStraight(distance = 5, speed = 20, backward = False, targetAngle = 0)
        
        # Backup from Watch Television
        gyroStraight(distance = 5, speed = 40, backward = True, targetAngle = 0)

    def _getToWindTurbine():
        angle = -40
        # Turn towards the hybrid car.
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.9) 

        # We should have turned such that we are able to find the black line in front of the wind turbine.
        _driveTillLine(speed=55, distanceInCM=35, target_angle=angle, colorSensorToUse="Right", blackOrWhite="Black", slowSpeedRatio=0.9)

        # After catching the black line, drive forward and turn to face the windmill
        gyroStraight(distance = 2, speed = 40, backward = False, targetAngle = angle)
        _turnToAngle(targetAngle = 40, speed = 25, slowTurnRatio = 0.9)

    def _windTurbine():
        angle = 40
        # Drive towards Wind Turbine and push the lever once
        drive(speed = 20, distanceInCM = 22, target_angle = angle)
        
        # Push the lever the remaining two times
        for i in range(3): 
            # Backup so the robot can push the Wind Turbine again
            wheels.move(amount = 7, unit = "cm", steering = 0, speed = -30) 
            
            # Drive forward to push the Wind Turbine
            drive(speed = 20, distanceInCM = 14, target_angle = angle)

    def _pickUpRechargeableBattery():
        angle = 40

        # Start bringing down the hybrid car arm. We do this slowly so the 
        # arm is high enough to clear the hybrid car, when it turns.
        motorD.start(-50)

        # Backoff from the windwill and flush against the rechargeable battery.
        # We go back fast first, and then slow down to flush.
        wheels.move(amount = 5, unit = "cm", steering = 0, speed = -45) 
        _turnToAngle(targetAngle = angle, speed = 25)
        wheels.move(amount = 20, unit = "cm", steering = 0, speed = -45)
        wheels.move(amount = 7, unit = "cm", steering = 0, speed = -25) 
        motorD.stop()

        # Reset the gyro since we aligned.
        primeHub.motion_sensor.reset_yaw_angle()
        
    def _hybridCar():
        # Drive forward a little to be able to turn
        gyroStraight(distance=12, speed = 40, backward = False, targetAngle = 0)

        # Align with the hybrid car.
        angle = -90
        _turnToAngle(targetAngle = angle, speed = 25, slowTurnRatio = 0.6)

        # Starting bringing down the hybrid car arm.
        motorD.start(-70)
        
        # Drive forward to align with the hybrid car.
        gyroStraight(distance=25, speed = 25, backward = False, targetAngle = angle)

        # Stop the arm
        motorD.stop()

        # Bring up the arm.
        moveArm(degrees = 2000, speed = 100, motor = motorD)

        # Backup from the hybrid car.
        gyroStraight(distance=20, speed = 70, backward = True, targetAngle = angle)

    def _goHome():
        _turnToAngle(targetAngle = -70, speed = 45, slowTurnRatio = 0.9)
        gyroStraight(distance=60, speed = 90, backward = True, targetAngle = -70)
    
    _watchTV()
    _getToWindTurbine()
    _windTurbine()
    _pickUpRechargeableBattery()
    _hybridCar()
    _goHome()

#endregion

#Reachargable Battery dropoff
#Run 1.5/run in the middle of 1 and 2
def _run1point5():
    primeHub.motion_sensor.reset_yaw_angle()
    gyroStraight(distance=37, speed=60, backward = True, targetAngle=0)
    gyroStraight(distance=37, speed=60, targetAngle=0)
#region Function Calls

_initialize()
doRunWithTiming(driver)

raise SystemExit
#endregion
