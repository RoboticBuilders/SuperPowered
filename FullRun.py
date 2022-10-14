# LEGO type:standard slot:1 autostart
#from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike import PrimeHub, ColorSensor,  Motor, MotorPair
#from spike.control import timer
from math import *
import collections
# Note that the "hub" import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from spike.operator import *
import gc
from math import *

AXLE_DIAMETER_CM = 12.7
WHEEL_RADIUS_CM = 4.4
GLOBAL_LEVEL = 0
ANYA_RUN_START_OFFSET_TO_MAT_NORTH = 90

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
# Left medium motor
motorF = Motor("F")

#Right color sensor
colorB = ColorSensor("B")
rightColorSensor = colorB # Easier alias to use in code

#Left color sensor
colorA = ColorSensor("A")
leftColorSensor = colorA #Easier alias to use in code

_CM_PER_INCH = 2.54

# This is based on emperical tests and by looking at the color sensor.
# This is also based on the new sensor mount which roughly puts the sensor
# at about 18-20mm of the ground.
BLACK_COLOR = 20
WHITE_COLOR = 90

#region Utilities
def initialize():
    
    print("___________________________________________________")
    primeHub.motion_sensor.reset_yaw_angle()
    wheels.set_stop_action("brake")
    wheels.set_motor_rotation(2*3.14*WHEEL_RADIUS_CM, 'cm')
    isBatteryGood()
    
  
def doRunWithTiming(run):
    logMessage("Starting run " + str(run), level=0)
    start_time = time.ticks_ms()  
    run()
    end_time = time.ticks_ms()
    #logMessage("Time for run " + str(run) + " time(ms): " + str(time.ticks_diff(end_time, start_time)), level=0)
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
    3. Spike Prime Continous space. This is the angles produced by the ContinousAngle class. This is what the loop
       in the _turnRobotWithSlowDown operates in. This space depends upon the start angle and zero crossings.
       Examples: 
       a) Turning right: 350,351,355,359,360,361,362,365...
       b) Turning left: 5,4,3,1,0,-1,-2,-3,-5....
    In the code its confusing what space a particular angle is in. So be careful when changing code.
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
    slowDistance = 0.2*distance
    if(slowDistance > _CM_PER_INCH*2):
        slowDistance = _CM_PER_INCH*2
    _gyroStraightNoSlowDownNoStop(distance = gradualAccelerationDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    _gyroStraightNoSlowDownNoStop(distance = distance - slowDistance - gradualAccelerationDistance, speed = speed, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    _gyroStraightNoSlowDownNoStop(distance = slowDistance, speed = 20, targetAngle=targetAngle, backward=backward, correctionMultiplier = 2)
    wheels.stop()

    finalDeg = abs(motorE.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distance-totalDistanceTravelled), level=4)
    logMessage("======== gyroStraight done for distance " + str(distance) + "==========", 4)

def _gyroStraightNoSlowDownNoStop(distance, speed = 20, backward = False, targetAngle = 0, correctionMultiplier = 2):
    initialDeg = abs(motorE.get_degrees_counted())

    underBiasErrorMultiplier = 1 # 1.106
    errorAdjustedDistanceInCm = distance*underBiasErrorMultiplier

    logMessage("GYROSTRAIGHT START: targetAngle  is " + str(targetAngle), level=4)
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

def getCorrectionForDrive(targetAngle, correctionMultiplier = 2):
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    logMessage("CurrentAngle: " + str(currentAngle) + " and targetAngle: " + str(targetAngle), 5)
    if( (currentAngle <= 0 and targetAngle <=0) or
            (currentAngle>0 and targetAngle > 0) or
            (abs(currentAngle) < 90 and abs(targetAngle)<90)):
        correction = targetAngle - currentAngle
    elif (currentAngle >= 90):
        correction = (360 - abs(currentAngle) - abs(targetAngle))
    else:
        correction = -1*(360 - abs(currentAngle) - abs(targetAngle))

    logMessage("Correction needed = " + str(correction), 4)
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
    logMessage("driveStraight for distance: " + str(distanceInCM) + " and target angle: " + str(target_angle), level=2)
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
    logMessage("Distance travelled after first part = "  + str(distanceTravelled) + " error=" + str(distance80-distanceTravelled), level=4)
    logMessage("remainingDistance after 1st travel=" + str(remainingDistance), level=2)

    # Only run this whole thing in case there is any remaining distance.
    # this check is meant to catch the case when for some reason
    # the currently travelled distance returned by wheels are wrong for some
    # reason.
    if (remainingDistance > 0):
        distance16 = remainingDistance * 0.6
        _driveStraightWithSlowDown(distance16, speed, target_angle, gain, slowDown=True)
        
        distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
        remainingDistance = distanceInCM - distanceTravelled
        logMessage("Distance travelled after second part= "  + str(distanceTravelled) + " error=" + str((distance80+distance16)- distanceTravelled), level=4)
        logMessage("remainingDistance after 2nd travel=" + str(remainingDistance), level=2)
        
        # Drive the final 4% of the distance at speed 5
        # If we have already overshot the target, then dont correct for it.
        if remainingDistance > 1:
            FINAL_SLOW_SPEED = 15
            _driveStraightWithSlowDown(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False)
            #wheels.move_tank(remainingDistance, "cm", FINAL_SLOW_SPEED, FINAL_SLOW_SPEED)
    
    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Drive: Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distanceInCM-totalDistanceTravelled), level=2)

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
        if (abs(correction) > 1):
            wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
    logMessage("Drivestraight completed", level=5)


def _driveTillLine(speed, distanceInCM, target_angle, gain = 1, colorSensorToUse="Left", blackOrWhite="Black"):
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
   
    logMessage("driveStraight for distance: " + str(distanceInCM) + " and target angle: " + str(target_angle), level=2)
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
        stoppingCondition = lambda: colorSensor.get_reflected_light() <= BLACK_COLOR
    elif (blackOrWhite == "White"):
        stoppingCondition = lambda: colorSensor.get_reflected_light() >= WHITE_COLOR
    elif (blackOrWhite == "Green"):
        stoppingCondition = lambda: colorSensor.get_color() == 'green'

    FINAL_SLOW_SPEED=10
    # If the distance is small, then just drive over that distance at FINAL_SLOW_SPEED.
    if (distanceInCM < 5):
        _driveStraightWithSlowDownTillLine(distanceInCM, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)    

    # First drive 80% of the distance at
    distance60 = distanceInCM * 0.6
    _driveStraightWithSlowDownTillLine(distance60, speed, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)

    # Drive the remaining distance at slow speed
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("_driveTillLine: Distance travelled after first part = "  + str(distanceTravelled) + " error=" + str(distanceTravelled-distance60), level=4)
    _driveStraightWithSlowDownTillLine(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)

    wheels.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("_driveTillLine: Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(totalDistanceTravelled-distanceInCM), level=2)
    
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
    """
    startDistanceInDeg = abs(motorC.get_degrees_counted())
    logMessage("startDistanceInDeg = " + str(int(startDistanceInDeg)), level=5)
    distanceInDeg = converCMToDeg(distance)
    currentSpeed = speed

    if (target_angle == -180):
        target_angle = 180

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0
    
    FINAL_SLOW_SPEED=15
    wheels.start(0, int(currentSpeed))
    correction = previousCorrection = 0
    while  distanceInDegTravelled <= distanceInDeg and reachedStoppingCondition() == False:
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
        logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelledInCM = " + str(convertDegToCM(distanceInDegTravelled)) + " distanceInCM=" + str(distance) 
            + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceToTravelInDeg=" + str(distanceInDeg) 
            + " target_angle= " + str(target_angle) + " current_yaw_angle = " + str(current_yaw_angle) +" correction= " + str(correction), level=5)
        if (abs(correction) > 1):
            wheels.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
    logMessage("DrivestraightWiuthSlowDownTillLine completed", level=5)
    
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
   

    """
    time.sleep(1)
    turnToAngle(targetAngle = 10, speed = 25)
    time.sleep(1)
    turnToAngle(targetAngle = 20, speed = 25)
    time.sleep(1)
    turnToAngle(targetAngle = 20, speed = 25)
    """
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
def runArisha():
    primeHub.motion_sensor.reset_yaw_angle()
    #active
    #getToOilPlatform_v2()
    # print("Left sensor: " + str(leftColorSensor.get_reflected_light()))
    # print("Right sensor: " + str(rightColorSensor.get_reflected_light()))
    # turnUntilColor(colorsensor=leftColorSensor, isColorBlack=False, isTurnLeft=True)
    # turnUntilColor(colorsensor=rightColorSensor, isColorBlack=False, isTurnLeft=True)
    # line_follow(distance=25)
    # _turnToAngle(-26)
    # line_follow(distance=20)
    getToOilPlatform_v2Point1()
    activeOilPlatform()
    goBackHomeFromOilPlatform()

    #static
    #getToOilPlatform()
    #unloadEnergyUnits()
    #goBackHomeFromOilPlatform()

    #pullTruckGoStraight()


def getToOilPlatform_v2():
    print("Running now")
   
    #gyroStraight(distance=_CM_PER_INCH*11.5, speed=20, targetAngle=0)
    gyroStraight(distance=_CM_PER_INCH*9.5, speed=40, targetAngle=0)
    _turnToAngle(45)
    gyroStraight(distance=_CM_PER_INCH*10, speed=40, targetAngle=45)
    #_driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*10, target_angle = 45)
    _turnToAngle(0)
    #will do this manually before the run starts
    #motorD.run_for_degrees(degrees=900, speed=100)   
    #time.sleep(10) 
    gyroStraight(distance=_CM_PER_INCH*10, speed=30, targetAngle=0)
    # time.sleep(5)


def getToOilPlatform_v2Point1():
    print("Running now")
    #working version1
    gyroStraight(distance=_CM_PER_INCH*9.5, speed=40, targetAngle=0)
    _turnToAngle(45)
    # time.sleep(5)
    _driveTillLine(speed = 30, distanceInCM = _CM_PER_INCH*12, target_angle = 45)
    # time.sleep(10)
    # gyroStraight(distance=_CM_PER_INCH*4, speed=20, targetAngle=45)
    # time.sleep(15)
    _turnToAngle(targetAngle=0, oneWheelTurn="Right")
    # time.sleep(5)
    #turnUntilColor(colorsensor=leftColorSensor, isColorBlack=False, isTurnLeft=True)
    # time.sleep(5)
    #turnUntilColor(colorsensor=rightColorSensor, isColorBlack=False, isTurnLeft=True)
    gyroStraight(distance=_CM_PER_INCH*10.5, speed=30, targetAngle=0)


def turnUntilColor(colorsensor, isColorBlack, isTurnLeft):
    """
    Turns until color is found. Only two colors supported: WHITE and BLACK. 
    Specify color by settng boolean isColorBlack to True for BLACK and False for WHITE
    Specify direction for turn by setting boolean isTurnLeft to True for Left and False for Right
    """
    if (isTurnLeft):
        turnDirection = -100
    else:
        turnDirection = 100
    
    while( (isColorBlack and colorsensor.get_reflected_light() > BLACK_COLOR) or
            ( not isColorBlack and colorsensor.get_reflected_light() < WHITE_COLOR)):
        print(colorsensor.get_reflected_light())
        wheels.start(steering = turnDirection, speed = 5)

    print("TurnUntilColor done for sensor " + str(colorsensor))
    wheels.stop()

def activeOilPlatform():
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, speed=40)
    # left_large_motor.run_for_degrees(degrees=90, speed=-40)
    # right_large_motor.run_for_degrees(degrees=90, speed=40)
    #time.sleep(1)
    #motorD.run_for_degrees(degrees=-2500, speed=100)
    # motorD.start(-100) # it was 1200 degrees before.Then it was 2400.
    # time.sleep(1)
    #gyroStraight(distance=1, speed=20, targetAngle=0, backward=True) # if this does not work then change it to 0.5
    # motorD.run_for_degrees(degrees=-2500, speed=100) # it was 1200 degrees before.Then it was 2400.
    gyroStraight(distance=0.5, speed=20, targetAngle=0, backward=True) # if this does not work then change it to back to 1
    motorD.run_for_degrees(degrees=-800, speed=100)# it was 1400 before (4 wheel rotations) now it is 2 wheel rotations
    # time.sleep(1)
    gyroStraight(distance=1, speed=20, targetAngle=0, backward=True)
    # motorD.stop()
    
    for i in range(3):
        motorF.run_for_degrees(degrees=-1000, speed=100)
        if (i<=1):
            motorF.run_for_degrees(degrees=1000, speed=100)
    # motorD.stop()
    wheels.move(amount = 10, unit = "in", steering = 0, speed = -40)

    #drive(speed = 25, distanceInCM = _CM_PER_INCH*8, target_angle = 0) 
    #turnToAngle(-90)
    #working code 1
    #drive(speed = 30, distanceInCM = _CM_PER_INCH*11.5, target_angle = 0)
    gyroStraight(distance=_CM_PER_INCH*11.5, speed=20, targetAngle=0)
    _turnToAngle(45)
    #drive(speed = 30, distanceInCM = _CM_PER_INCH*10, target_angle = 45)  
    gyroStraight(distance=_CM_PER_INCH*10, speed=20, targetAngle=45)
    # 
    # turnToAngle(22)
    # drive(speed = 20, distanceInCM = 49, target_angle = 22)  
    _turnToAngle(0)
    # time.sleep(5)
    motorD.run_for_degrees(degrees=600, speed=20)
    #drive(speed = 30, distanceInCM = _CM_PER_INCH*10, target_angle = 0)
    gyroStraight(distance=_CM_PER_INCH*10, speed=20, targetAngle=0)
    time.sleep(5)

def goBackHomeFromOilPlatform():
    _turnToAngle(60)
    wheels.move(amount = 20, unit = "in", steering = 0, speed = -40) # Back home doesnt require accuracy
   #turnToAngle(30)
   #wheels.move(amount = 11, unit = "in", steering = 0, speed = -30) # Back home doesnt require accuracy


def pullTruckGoStraight():
    # motorF.run_for_degrees(degrees=1000, speed=100)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH * 10, speed=40)
    motorF.run_for_degrees(degrees=-1000, speed=100)
    wheels.move(amount = 10, unit = "in", steering = 0, speed = -30)

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
def _runAnya():
    primeHub.motion_sensor.reset_yaw_angle()
    print("Battery voltage: " + str(hub.battery.voltage()))
    # raiseEnergyUnitCollectingArm(200)# at least 245 But we don't know
    getToPowerPlantFromHome2()
    # ReleaseEnergyUnitsLowerFirst()
    ReleaseEnergyUnitsRaiseFirst()
    #goToHome1()
    goToHome1_MissingUnit()

def raiseEnergyUnitCollectingArm(deg = 90, raiseArm = True):
    multiplier = 1
    if raiseArm == True:
        multiplier = -1

    motorF.run_for_degrees(degrees=deg * 24 * multiplier, speed=40)

def goToHome1():
    wheels.move(amount = 5, unit = "in", steering = 0, speed = -40)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105)#original value -90
    # wheels.move(amount = 35, unit = "in", steering = 0, speed = 90)#original speed 40
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105,  distance = _CM_PER_INCH*35, speed=90)


def goToHome1_MissingUnit():
    wheels.move(amount = 5, unit = "in", steering = 0, speed = -40)
    # _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135)#original value -90
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = _CM_PER_INCH*8, speed=90)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110)
    # wheels.move(amount = 35, unit = "in", steering = 0, speed = 90)#original speed 40
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110,  distance = _CM_PER_INCH*35, speed=90)

def getToPowerPlantFromHome2():

    # print('Starting getToPowerPlantFromHome2 function')
    # print('Going forward 9.5 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH + 0,  distance = _CM_PER_INCH*6)
    # print('Turning to angle: -90. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    # print('Going forward 36 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = _CM_PER_INCH*30, speed=60) # was 29 and then 2 more at 40 speed below
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*6, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90, blackOrWhite="White")
    # drive(speed= 60,distanceInCM= _CM_PER_INCH*29, target_angle= ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    #time.sleep(5)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = _CM_PER_INCH*2, speed=40)
    # drive(speed= 40,distanceInCM= _CM_PER_INCH*2, target_angle= ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    # time.sleep(5)

    ToyFactory2()

    # print('Turning to angle: -135. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # turnToAngle(targetAngle = -135, speed = 25)
    # print('Going back 2 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # gyroStraight(targetAngle=-135, distance=_CM_PER_INCH*5, backward = True)
    # print('Turning to angle: -179. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))

    # _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135, speed=25, oneWheelTurn="Right")
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = _CM_PER_INCH*1, speed=40, backward=True)

    _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179, speed=25, oneWheelTurn="Right")
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179,  distance = _CM_PER_INCH*8.5, speed=40)
    # gyroStraight(targetAngle = 170,  distance = _CM_PER_INCH*4)
    # _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179, speed = 25)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179,  distance = _CM_PER_INCH*3)
    # print('current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # print('getToPowerPlantFromHome2 function Done')

def ToyFactory2():
    # Align for the toy factory
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = 6, backward = True)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105, oneWheelTurn="Left")
    # time.sleep(5)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105,  distance = 3, backward = True)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135, oneWheelTurn="Left")
    #time.sleep(5)
    # Do the Toy Factory
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = 19 , backward = True) # was 8
    #time.sleep(5)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = 8)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    # time.sleep(5)
    # Align for the Power Plant
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*9, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90, blackOrWhite="White")
    # time.sleep(5)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = 5)


def ToyFactory():
    # time.sleep(5)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = _CM_PER_INCH*5.5, backward=True)
    # _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110,  distance = _CM_PER_INCH*3, backward=True)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135)
    # time.sleep(10)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = _CM_PER_INCH*4.5, backward=True)
    time.sleep(10)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 135,  distance = _CM_PER_INCH*4.5)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*9, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = 5) # Original was 8
    # time.sleep(5)


def ReleaseEnergyUnitsLowerFirst():
    motorD.run_for_degrees(degrees=-130, speed=100)
    time.sleep(0.5)
    motorD.run_for_degrees(degrees=250, speed=100)
    time.sleep(15)

def ReleaseEnergyUnitsRaiseFirst():
    #time.sleep(5)
    motorD.run_for_degrees(degrees=100, speed=100) #original values 150
    # time.sleep(5)

    motorD.run_for_degrees(degrees=-180, speed=100)#original values -150 working value 100
    # time.sleep(5)

    
    motorD.run_for_degrees(degrees=200, speed=100) #was 250

    motorD.run_for_degrees(degrees=-250, speed=100)

    #time.sleep(5)
    #endregion Anya 

#region Nami

def _run4():
    # Drive till the hydro plant to pick up the first water
    # unit
    #wheels.set_stop_action("break")
    drive(speed=25, distanceInCM=15, target_angle=0)
    _turnToAngle(targetAngle=-35,speed=20)
    drive(speed=25, distanceInCM=15, target_angle=-37)

    # This is the turn after pickup and should be very slow.
    _turnToAngle(targetAngle=0,speed=15)
    
    # Turns towards the n-s black line in front of the power station.
    # drive to catch the line. Note the -5d run.
    _driveTillLine(speed=45, distanceInCM=90, target_angle=-5, colorSensorToUse="Right", blackOrWhite="Black")
    drive(speed=25, distanceInCM=12, target_angle=0)

    # Turn towards the smart grid and drive forward to catch the e-w line in front of the smart grid.
    _turnToAngle(targetAngle=-80,speed=15)
    drive(speed=35, distanceInCM=20, target_angle=-80)
    _driveTillLine(speed=35, distanceInCM=100, target_angle=-80, colorSensorToUse="Right", blackOrWhite="White")

    # Backoff before turning.
    gyroStraight(distance=5, speed = 20, backward = True, targetAngle = -80)
    
    # Turn to pick up the last two water units
    _turnToAngle(targetAngle=-165,speed=15)
    _driveTillLine(speed=20, distanceInCM=45, target_angle=-165, colorSensorToUse="Left", blackOrWhite="Black")
    drive(speed=20, distanceInCM=11, target_angle=-165)

    # Going to solar famr now
    _turnToAngle(targetAngle=-120,speed=15)
    _driveTillLine(speed=20, distanceInCM=45, target_angle=-120, colorSensorToUse="Right", blackOrWhite="Black")
    _turnToAngle(targetAngle=-70,speed=15)
    
    drive(speed=20, distanceInCM=15, target_angle=-70)
'''
    # backup before solar farm units
    gyroStraight(distance=2, speed = 20, backward = True, targetAngle = -70)
    _turnToAngle(targetAngle=-150,speed=15)
    drive(speed=20, distanceInCM=23, target_angle=-150)

    # now backup from solar units to go back home
    gyroStraight(distance=2, speed = 20, backward = True, targetAngle = -150)    

      # Turn towards home and go home.    
    _turnToAngle(targetAngle=160,speed=20)
    drive(speed=90, distanceInCM=95, target_angle=160, dontSlowDown=True)'''
    
# Drop water units
# Drop off energy units and innovation project
# Drop off energy units at rechargeable battery
# Take Oil truck to its final place.
def _run5():
    #global GLOBAL_LEVEL
    #GLOBAL_LEVEL = 5
    # Drive forward first. Drive at a slight angle to avoid hitting the power plant.
    drive(speed = 55, distanceInCM = 50, target_angle = -5)
    
    # Turn slightly to catch the n-s line in front of the power plant
    _turnToAngle(targetAngle = -20, speed = 20)
    _driveTillLine(speed=35, distanceInCM=70, target_angle=-20, colorSensorToUse="Right", blackOrWhite="Black")

    # Turn towards the power plant and then try to catch the black line running e-w line in front of the smart grid.
    _turnToAngle(targetAngle = -90, speed = 20)
    _driveTillLine(speed=35, distanceInCM=70, target_angle=-90, colorSensorToUse="Left", blackOrWhite="Black")

    # Turn towards the hydro-electric plant and then drop the water units. 
    # This also drops the energy units and the innovation project.
    _turnToAngle(targetAngle = 150, speed = 20)
    drive(speed = 25, distanceInCM = 25, target_angle = 150)

    # Drop the water units    
    moveArm(degrees = 3000, speed = -100, motor = motorF)

    # Backoff to leave the water reservoir
    gyroStraight(distance=10, speed = 20, backward = True, targetAngle = 150)

    # Bring upthe arm to backoff. 
    motorF.start_at_power(100)
    #moveArm(degrees = 3000, speed = 100, motor = motorF)
        
    # Now drive to the rechargeable battery and drop the oil factory
    _dropRechargeableBatteryAndOilTruck()

def _dropRechargeableBatteryAndOilTruck():
    # The robot gyro is consistently off by 10d, so we are adding this correction
    # to every angle in the code below. This is only done in this method, 
    # since for some reason this only kicks in after the initial set of methods.
    targetAngleAdjustment = 20

    # Turn towards the toy factory
    # This should be actually zero, however the robot seems to be off in its measure
    # consistently by 10d, so adjusting for that.
    _turnToAngle(targetAngle = 0 + targetAngleAdjustment, speed = 20)

    # Drive past the n-s line in front of the toy factory
    drive(speed = 35, distanceInCM = 25, target_angle = 0 + targetAngleAdjustment)

    # We expect the arm to have come up by now.
    motorF.stop()
    
    # Turn to catch e-w line in front of smartgrid
    _turnToAngle(targetAngle = -25, speed = 20)
    _driveTillLine(speed=25, distanceInCM=20, target_angle=-25, colorSensorToUse="Left", blackOrWhite="Black")

     # Turn to catch n-s line near toy factory
    _turnToAngle(targetAngle = 0 + targetAngleAdjustment, speed = 20)
    _driveTillLine(speed=25, distanceInCM=20, target_angle=0 + targetAngleAdjustment, colorSensorToUse="Right", blackOrWhite="Black")

    # Move and turn towards rechargeable battery
    drive(speed = 25, distanceInCM = 13, target_angle = 0 + targetAngleAdjustment)
    _turnToAngle(targetAngle = 55, speed = 20)
    drive(speed = 25, distanceInCM = 22, target_angle = 55)

    # Drop energy units in rechargeable battery
    moveArm(degrees = 3000, speed = 100, motor = motorD)
    
    # Start picking up the arm. Uncomment this for the competition. Instead of the waiting for bringing up the arm.
    #motorD.start_at_power(100)
    moveArm(degrees = 3000, speed = -100, motor = motorD)
    
    # Backup to the fuel station with the oil truck to finish
    _turnToAngle(targetAngle = 85, speed = 20)
    gyroStraight(distance=25, speed = 25, backward = True, targetAngle = 85)
    
    # Uncomment this for the competition.
    #motorD.stop()

#endregion Nami

#region Rishabh
def run1Old(moveArmDegrees, armSpeed):
    def full():
        drive(speed = 30, distanceInCM = 50, target_angle = 0) #Drive to Watch Tele
        wheels.move(amount = 15, unit = "cm", steering = 0, speed = -30) #Backup from Watch Television
        _turnToAngle(targetAngle = -45, speed = 25) #Turn towards Hybrid Car
        drive(speed = 30, distanceInCM = 45, target_angle = -45) #Drive towards Hybrid Car
        _turnToAngle(targetAngle = 45, speed = 25) #Turn towards Wind Turbine
        drive(speed = 30, distanceInCM = 29, target_angle = 45) #Drive towards Wind Turbine and push the lever once

        for i in range(2): #Push the lever the remaining two times
            wheels.move(amount = 7, unit = "cm", steering = 0, speed = -20)
            drive(speed = 20, distanceInCM = 7, target_angle = 35)
            time.sleep(0.5)
        wheels.move(amount = 11, unit = "cm", steering = 0, speed = -30) #Backup from Wind Turbine
        _turnToAngle(targetAngle = -50, speed = 25) #Turn towards Hybrid Car
        moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF) #Lower the Hybrid Car arm
        drive(speed = 30, distanceInCM = 20, target_angle = -50) # Drive towards the Hybrid Car
        moveArm(degrees = -1 * moveArmDegrees, speed = ceil(-0.75 * armSpeed), motor = motorF) #Raise the Hybrid Car arm and complete the mission
        wheels.move(amount = 30, unit = "cm", steering = 0, speed = -30) #Backup from the Hybrid Car
        _turnToAngle(targetAngle = -70, speed = 20) #Turn towards Rechargable Battery
        moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF)
        time.sleep(1)
        wheels.move(amount = 70, unit = "cm", steering = 0, speed = -50)
        _turnToAngle(targetAngle = -90, speed = 100)

    def watchTV():
        # Drive to Watch Television
        drive(speed = 25, distanceInCM = 52, target_angle = 0) 
        
        #Backup from Watch Television
        gyroStraight(distance=10, speed = 20, backward = True, targetAngle = 0)

    def getToWindTurbine():
        # Turn towards the hybrid car.
        _turnToAngle(targetAngle = -20, speed = 25) 

        # We should have turned such that we are able to find the black line in front of the wind turbine.
        _driveTillLine(speed=25, distanceInCM=45, target_angle=-20, colorSensorToUse="Right", blackOrWhite="Black")

        # Drive to align the arm with the wind turbine.

        # The Forward below used to be 10, we made it 8.
        drive(speed = 25, distanceInCM = 8, target_angle = -20) 
        _turnToAngle(targetAngle = 20, speed = 20) 
        gyroStraight(distance=12, speed = 20, backward = True, targetAngle = 20)
        _turnToAngle(targetAngle = 40, speed = 20)

        # Turn towards the wind turbine, using a force turn to avoid hitting the turbine.
        #_turnToAngle(targetAngle = 45, speed = 25, forceTurn="Left") 

    def windTurbine():
        drive(speed = 10, distanceInCM = 20, target_angle = 42) #Drive towards Wind Turbine and push the lever once
        time.sleep(1)
        gyroStraight(distance=10, speed = 20, backward = True, targetAngle = 42)
        # for i in range(2): #Push the lever the remaining two times
        #     wheels.move(amount = 10, unit = "cm", steering = 0, speed = -20) #Backup so the robot can push the Wind Turbine again
        #     drive(speed = 20, distanceInCM = 7, target_angle = 35) #Drive forward to push the Wind Turbine
        #     time.sleep(0.5) #Wait slightly
        # wheels.move(amount = 11, unit = "cm", steering = 0, speed = -30) #Backup from Wind Turbine

    def hybridCar(moveArmDegrees, armSpeed):
        _turnToAngle(targetAngle = -50, speed = 25) #Turn towards Hybrid Car
        moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF) #Lower the Hybrid Car arm
        drive(speed = 30, distanceInCM = 20, target_angle = -50) # Drive towards the Hybrid Car
        moveArm(degrees = -1 * moveArmDegrees, speed = ceil(-0.75 * armSpeed), motor = motorF) #Raise the Hybrid Car arm and complete the mission
        wheels.move(amount = 30, unit = "cm", steering = 0, speed = -30) #Backup from the Hybrid Car

    def rechargableBattery(moveArmDegrees, armSpeed):
        _turnToAngle(targetAngle = -70, speed = 20) #Turn towards Rechargable Battery
        moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF) #Lower arm for Rechargable Battery
        time.sleep(1)

    def goHome():
        wheels.move(amount = 70, unit = "cm", steering = 0, speed = -50)
        _turnToAngle(targetAngle = -90, speed = 100)
    
def run1(moveArmDegrees, armSpeed):
    def watchTV():
        # Drive to Watch Television
        drive(speed = 25, distanceInCM = 43, target_angle = 0) 
        
        #Backup from Watch Television
        gyroStraight(distance=19, speed = 20, backward = True, targetAngle = 0)

    def testHybridCarArm():
        # Bring arm down
        moveArm(degrees = 2800, speed = -100, motor = motorD)

        # Bring arm up
        moveArm(degrees = 2800, speed = 100, motor = motorD)

    def getToWindTurbine():
        # Turn towards the hybrid car.
        # URGENT: THE ROBOT SOMETIMES TURNS INFINITY ON THIS TURN
        _turnToAngle(targetAngle = -30, speed = 25) 

        # We should have turned such that we are able to find the black line in front of the wind turbine.
        _driveTillLine(speed=25, distanceInCM=45, target_angle=-30, colorSensorToUse="Right", blackOrWhite="Black")

        # Drive to align the arm with the wind turbine.

        # The Forward below used to be 10, we made it 8.
        drive(speed = 25, distanceInCM = 12, target_angle = -20) 
        _turnToAngle(targetAngle = 20, speed = 20) 
        gyroStraight(distance=3, speed = 20, backward = True, targetAngle = 20)
        _turnToAngle(targetAngle = 40, speed = 20)

    def windTurbine():
        drive(speed = 20, distanceInCM = 20, target_angle = 40) #Drive towards Wind Turbine and push the lever once
        time.sleep(1)
        #gyroStraight(distance=10, speed = 20, backward = True, targetAngle = 42)
        for i in range(2): #Push the lever the remaining two times
            wheels.move(amount = 10, unit = "cm", steering = 0, speed = -20) #Backup so the robot can push the Wind Turbine again
            drive(speed = 20, distanceInCM = 20, target_angle = 40) #Drive forward to push the Wind Turbine
            time.sleep(0.5) #Wait slightly
        wheels.move(amount = 16, unit = "cm", steering = 0, speed = -30) #Backup from Wind Turbine

    def hybridCar(moveArmDegrees, armSpeed):
        _turnToAngle(targetAngle = 120, speed = 25) #Turn towards Hybrid Car
        # drive(speed = 30, distanceInCM = 20, target_angle = 140) # Drive towards the Hybrid Car
        gyroStraight(distance=15, speed = 20, backward = True, targetAngle = 120)
        
        # Raise the Hybrid Car arm and complete the mission
        moveArm(degrees = -1700, speed = -100, motor = motorD)

        # Lower the Hybrid Car arm 
        moveArm(degrees = 1000, speed = 100, motor = motorD)

        # Drive forward
        drive(speed = 30, distanceInCM = 30, target_angle = 120)

        # Lower the Hybrid Car arm 
        moveArm(degrees = -1 * 500, speed = -100, motor = motorD)
        drive(speed = 30, distanceInCM = 36, target_angle = 0)
    
    def rechargableBattery(moveArmDegrees, armSpeed):
        _turnToAngle(targetAngle = 90, speed = 20) #Turn towards Rechargable Battery
        moveArm(degrees = ceil(-0.75 * moveArmDegrees), speed = -1 * armSpeed, motor = motorD) #Lower arm for Rechargable Battery
        _turnToAngle(targetAngle = 120, speed = 20)
        #time.sleep(1)

    def goHome():
        #wheels.move(amount = 70, unit = "cm", steering = 0, speed = -50)
        drive(speed = 30, distanceInCM = 25, target_angle = 100)
        #_turnToAngle(targetAngle = -90, speed = 100)
    
    watchTV()
    getToWindTurbine()
    windTurbine()
    hybridCar(moveArmDegrees, armSpeed)
    # rechargableBattery(moveArmDegrees, armSpeed)
    # goHome()
    #testHybridCarArm()

    
#endregion

#region Function Calls
initialize()
testLineSquaring()

# If we are comfortable with this code, then move it into the run1 code.
def doHybridCar():
    # Lower the Hybrid Car arm
    moveArm(degrees = -2500, speed = -100, motor = motorD)

    # # Raise the Hybrid Car arm and complete the mission
    moveArm(degrees = 2000, speed = 100, motor = motorD)

    # # Lower the Hybrid Car arm 
    moveArm(degrees = -1000, speed = -100, motor = motorD)

    # Drive forward
    drive(speed = 30, distanceInCM = 10, target_angle = 0)

    # # Lower the Hybrid Car arm 
    motorD.set_stop_action("hold")
    moveArm(degrees = -1100, speed = -100, motor = motorD)
    drive(speed = 30, distanceInCM = 50, target_angle = 0)
    motorD.set_stop_action("brake")

    # Remove in production code.
    moveArm(degrees = 2600, speed = 100, motor = motorD)

# doHybridCar()

#_run5()
#motorF.start_at_power(100)
#run1(moveArmDegrees = 2750, armSpeed = 100)
#_turnToAngle(speed = 20, targetAngle = 115)
#drive(speed = 20, distanceInCM = 30, target_angle = 0)

#doRunWithTiming(_run4)
#doRunWithTiming(_run5)
#runArisha()
raise SystemExit

#drive(speed=40,distanceInCM= 10, target_angle= 0)
#turnToAngle(targetAngle= -30 ,speed= 20)

#run4()

#doRunWithTiming(run5)
#runArisha()
#runAnya()
#t1_end = time.ticks_ms()
#print("Time taken timetakenfor this run " + 
#str( time.ticks_diff(t1_end,t1_start)) + " milliseconds")

#endregion
#GLOBAL_LEVEL = 5
#_turnToAngle(targetAngle = -90, speed = 25)
#drive(speed=25, distanceInCM=30, target_angle=-90, gain = 1)
#_run4WithSolarFarmWithLineFindingWithBigLeftArm()

## t1_start = time.ticks_ms()
# runArisha()
## _runAnya()
## t1_end = time.ticks_ms()
## print("Time taken time taken for this run " + 
## str( time.ticks_diff(t1_end,t1_start)) + " milliseconds")
