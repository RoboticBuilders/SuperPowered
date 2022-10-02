# LEGO type:standard slot:1 autostart
#from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike import PrimeHub, ColorSensor,  Motor, MotorPair
#from spike.control import wait_for_seconds, wait_until, Timer
from math import *
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
# Right large motor
motorE = Motor("E")

# The motor pair
motors = MotorPair('C', 'E')

# Right medium motor
motorD = Motor("D")
# Left medium motor
motorF = Motor("F")

#Right color sensor
colorB = ColorSensor("B")
#Left color sensor
colorA = ColorSensor("A")

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
    motors.set_stop_action("brake")
    motors.set_motor_rotation(2*3.14*WHEEL_RADIUS_CM, 'cm')
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
        

def _turnToAngle(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.4, correction=0.05):
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
    motors.stop()
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
    
    _turnRobotWithSlowDown(degreesToTurn, reducedTargetAngleIn179Space, speed, slowTurnRatio, direction)
    
    currentAngle = gyroAngleZeroTo360()
    #logMessage("TurnToAngle complete current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=4)
    logMessage("TurnToAngle complete current_angle: {}  targetAngle: {} ".format(str(currentAngle),str(targetAngle)), level=4)
    

def _turnRobotWithSlowDown(angleInDegrees, targetAngle, speed, slowTurnRatio, direction):
    """
    Turns the Robot using a fast turn loop at speed and for the slowTurnRatio
    turns the robot at SLOW_SPEED.

    angleInDegrees -- Angle in degrees to turn. Can be +ve or -ve.
    targetAngle -- targetAngle should be in the -179 to 179 space
    speed -- Fast turn speed. 
    slowTurnRatio -- This is the % of the turn that we want to slow turn.
                     For example 0.2 means that 20% of the turn we want
                     to slow turn.
    """
    SLOW_SPEED = 10
    
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    startAngle = currentAngle
    slowTurnSpeed = speed

    # First we will do a fast turn at speed. The amount to turn is 
    # controlled by the slowTurnRatio.
    if (direction == "Right"):
        motors.start_tank(speed, speed * -1)
    if (direction == "Left"):
        motors.start_tank(speed * -1, speed)

    fastTurnDegrees =  (1 - slowTurnRatio) * abs(angleInDegrees)
    while (abs(currentAngle - targetAngle) > fastTurnDegrees):
        time.sleep_ms(7)
       
        currentAngle = primeHub.motion_sensor.get_yaw_angle()    

    # After the initial fast turn that is done using speed, we are going to do a 
    # slow turn using the slow speed.
    if (direction == "Right"):
        motors.start_tank(SLOW_SPEED, SLOW_SPEED * -1)
    if (direction == "Left"):
        motors.start_tank(SLOW_SPEED * -1, SLOW_SPEED)

    while (abs(currentAngle - targetAngle) > 1):
        time.sleep_ms(7)
        
        currentAngle = primeHub.motion_sensor.get_yaw_angle()

    motors.stop()
   
    """
    motorCDiff = abs(motorC.get_degrees_counted()) - motorCInitialDeg
    motorEDiff = abs(motorE.get_degrees_counted()) - motorEInitialDeg
    avgDiff = (abs(motorCDiff) + abs(motorEDiff)) / 2
    robotTurn = 0.346*avgDiff
    logMessage("In SlowTurn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed) + " motorCDegDiff=" + 
        str(motorCDiff) + " motorEDegDiff=" + str(motorEDiff) + " expectedRobotTurn=" + str(robotTurn), level=5)  

    """
    
def gyroStraight(distance, speed = 20, backward = False, targetAngle = 0):
    initialDeg = abs(motorC.get_degrees_counted())

    logMessage("GYROSTRAIGHT START: targetAngle  is " + str(targetAngle), level=4)
    degreesToCover = (distance * 360)/(WHEEL_RADIUS_CM * 2 * 3.1416)
    position_start = motorE.get_degrees_counted()
    if (backward): 
        while ((motorE.get_degrees_counted() - position_start)  >= degreesToCover * -1):
           
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = getCorrectionForDrive(targetAngle) # - currentAngle
            motors.start(steering = -correction, speed=speed * -1)
    else:
         while ((motorE.get_degrees_counted() - position_start)  <= degreesToCover):
           
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = getCorrectionForDrive(targetAngle) # targetAngle - currentAngle
            motors.start(steering = correction, speed=speed)

    motors.stop()

    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distance-totalDistanceTravelled), level=4)
    logMessage("======== gyroStraight done for distance " + str(distance) + "==========", 4)
   

def getCorrectionForDrive(targetAngle):
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    logMessage("CurrentAngle: " + str(currentAngle) + " and targetAngle: " + str(targetAngle), 4)
    if( (currentAngle <= 0 and targetAngle <=0) or
            (currentAngle>0 and targetAngle > 0) or
            (abs(currentAngle) < 90 and abs(targetAngle)<90)):
        correction = targetAngle - currentAngle
    elif (currentAngle >= 90):
        correction = (360 - abs(currentAngle) - abs(targetAngle))
    else:
        correction = -1*(360 - abs(currentAngle) - abs(targetAngle))

    logMessage("Correction needed = " + str(correction), 4)
    correctionMultiplier = 2 # being off by more than 20 degrees will cause the robot to do a full turn to correct
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


def drive(speed, distanceInCM, target_angle, gain = 1):
    """
    Drive
    _____
    This function drives the robot FORWARD using the motion sensor and the 80-15-5 formula.
    80% of distance at speed
    15% of distance with linear slow down from speed to 10
    5% of distance with speed of FINAL_SLOW_SPEED.
    _____
    Speed - Speed the motors travel at. Integer from -100 to 100
    DistanceInCM - Distance to travel in centimeters. Integer greater than 0
    TargetAngle - The angle the robot should drive at. Integer from 0 to 360
    Gain - The multiplier off the error. Integer greater than 0
    """
    
    #motors.move_tank(distanceInCM, "cm", speed, speed)
    #return
    
    motors.stop()
    logMessage("driveStraight for distance: " + str(distanceInCM) + " and target angle: " + str(target_angle), level=2)
    initialDeg = abs(motorC.get_degrees_counted())
    remainingDistance = distanceInCM
    
    # If the distance is small, then just drive over that distance at speed.
    if (distanceInCM < 5):
        _driveStraightWithSlowDown(distanceInCM, speed, target_angle, gain, slowDown=False)    

    # First drive 80% of the distance at
    distance80 = distanceInCM * 0.6
    _driveStraightWithSlowDown(distance80, speed, target_angle, gain, slowDown=False)
    
    # This code is commented for now, the above code seems to be providing the best algorithm
    # We will reevaluate if needed.
    # Drive the next 16% of the distance at a speed that reduces from speed to speed=FINAL_SLOW_SPEED
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("Distance travelled after first part = "  + str(distanceTravelled) + " error=" + str(distance80-distanceTravelled), level=4)
    logMessage("remainingDistance after 1st travel=" + str(remainingDistance), level=1)

    # Only run this whole thing in case there is any remaining distance.
    # this check is meant to catch the case when for some reason
    # the currently travelled distance returned by motors are wrong for some
    # reason.
    if (remainingDistance > 0):
        distance16 = remainingDistance * 0.6
        _driveStraightWithSlowDown(distance16, speed, target_angle, gain, slowDown=True)
        
        distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
        remainingDistance = distanceInCM - distanceTravelled
        logMessage("Distance travelled after second part= "  + str(distanceTravelled) + " error=" + str((distance80+distance16)- distanceTravelled), level=4)
        logMessage("remainingDistance after 2nd travel=" + str(remainingDistance), level=1)
        
        # Drive the final 4% of the distance at speed 5
        # If we have already overshot the target, then dont correct for it.
        if remainingDistance > 1:
            FINAL_SLOW_SPEED = 15
            _driveStraightWithSlowDown(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False)
            #motors.move_tank(remainingDistance, "cm", FINAL_SLOW_SPEED, FINAL_SLOW_SPEED)
    
    motors.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distanceInCM-totalDistanceTravelled), level=2)

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
    motors.start(0, int(currentSpeed))
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
        #logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelledInCM = " + str(convertDegToCM(distanceInDegTravelled)) + " distanceInCM=" + str(distance) 
        #  + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceToTravelInDeg=" + str(distanceInDeg) 
        # + " target_angle= " + str(target_angle) + " current_yaw_angle = " + str(current_yaw_angle) +" correction= " + str(correction), level=5)
        if (abs(correction) > 2):
            motors.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
    logMessage("Drivestraight completed", level=5)


"""
Mistakes that Rishabh made:
1. The loop termination condition is like so
while  distanceInDegTravelled <= distanceInDeg and colorB.get_reflected_light() != rightColor and colorA.get_reflected_light() != leftColor:
doesnt this mean that the loop needs both the color sensors to hit the color? This seems tricky.

2. Also see the check for the color it is written as !=. This is typically a bad idea why? How would you change this?

"""

def _driveTillLine(speed, distanceInCM, target_angle, gain = 1, colorSensorToUse="Left", blackOrWhite="Black"):
    """
    Drive
    _____
    This function drives the robot FORWARD using the motion sensor and the 80-20 formula.
    80% of distance at speed
    20% of distance with FINAL_SLOW_SPEED
    _____
    Speed - Speed the motors travel at. Integer from -100 to 100
    DistanceInCM - Distance to travel in centimeters. Integer greater than 0
    TargetAngle - The angle the robot should drive at. Integer from 0 to 360
    Gain - The multiplier off the error. Integer greater than 0
    colorSensorToUse - "Left" or "Right". 
    blackOrWhite - "Black" or "White".
    """
    assert(colorSensorToUse == "Left" or colorSensorToUse == "Right")
    assert(blackOrWhite == "Black" or blackOrWhite == "White")

    motors.stop()
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
    else:
        stoppingCondition = lambda: colorSensor.get_reflected_light() >= WHITE_COLOR

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
    logMessage("Distance travelled after first part = "  + str(distanceTravelled) + " error=" + str(distanceTravelled-distance60), level=4)
    _driveStraightWithSlowDownTillLine(remainingDistance, FINAL_SLOW_SPEED, target_angle, gain, slowDown=False, reachedStoppingCondition=stoppingCondition)

    motors.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(totalDistanceTravelled-distanceInCM), level=2)
    
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
    motors.start(0, int(currentSpeed))
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
        #logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelledInCM = " + str(convertDegToCM(distanceInDegTravelled)) + " distanceInCM=" + str(distance) 
        #    + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceToTravelInDeg=" + str(distanceInDeg) 
        #    + " target_angle= " + str(target_angle) + " current_yaw_angle = " + str(current_yaw_angle) +" correction= " + str(correction), level=5)
        if (abs(correction) > 2):
            motors.start(turn_rate, int(currentSpeed))

        distanceInDegTravelled = abs(motorC.get_degrees_counted()) - startDistanceInDeg
    logMessage("Drivestraight completed", level=5)
    
    

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


 # ------------------------------------------------------------------- End Utilities --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 #endregion


#region Arisha
def runArisha():
    primeHub.motion_sensor.reset_yaw_angle()
    #active
    #getToOilPlatform_v2()
    #getToOilPlatform_v2Point1()
    #activeOilPlatform()
    #goBackHomeFromOilPlatform()
    #static
    #getToOilPlatform()
    #unloadEnergyUnits()
    #goBackHomeFromOilPlatform()
    # pullTruck()
    #pullTruckLeftArm()
    pullTruckGoStraight()


def getToOilPlatform_v2():
    print("Running now")
   
    #gyroStraight(distance=_CM_PER_INCH*11.5, speed=20, targetAngle=0)
    gyroStraight(distance=_CM_PER_INCH*9.5, speed=40, targetAngle=0)
    _turnToAngle(45)
    gyroStraight(distance=_CM_PER_INCH*10, speed=40, targetAngle=45)
    #_driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*10, target_angle = 45)
    _turnToAngle(0)
    motorD.run_for_degrees(degrees=900, speed=100)   
    time.sleep(10) 
    gyroStraight(distance=_CM_PER_INCH*10, speed=30, targetAngle=0)
    # time.sleep(5)


def getToOilPlatform_v2Point1():
    print("Running now")
    #gyroStraight(distance=_CM_PER_INCH*11.5, speed=20, targetAngle=0)
    gyroStraight(distance=_CM_PER_INCH*9.5, speed=40, targetAngle=0)
    _turnToAngle(90)
    #gyroStraight(distance=_CM_PER_INCH*10, speed=40, targetAngle=45)
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*10, target_angle = 90)
    _turnToAngle(0)
    motorD.run_for_degrees(degrees=900, speed=100)   
    time.sleep(10) 
    gyroStraight(distance=_CM_PER_INCH*10, speed=30, targetAngle=0)
    # time.sleep(5)

def activeOilPlatform():
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, speed=20)
    motorD.run_for_degrees(degrees=-900, speed=100)
    gyroStraight(distance=_CM_PER_INCH*1, speed=20, targetAngle=0, backward=True)
    for i in range(3):
        motorF.run_for_degrees(degrees=-1000, speed=100)
        motorF.run_for_degrees(degrees=1000, speed=100)
    motors.move(amount = 10, unit = "in", steering = 0, speed = -30)


def getToOilPlatform():
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

def unloadEnergyUnits():
    #motorF.run_for_degrees(degrees=600, speed=40)
    print('current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    for i in range(3):
        # drive(speed = 20, distanceInCM = _CM_PER_INCH*8, target_angle = 0)
        gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*8, speed=20)
        wiggleOilPlatform()
        gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward =True)
        _turnToAngle(2)

    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*8, speed=20)
    # drive(speed = 20, distanceInCM = _CM_PER_INCH*8, target_angle = 0)
    wiggleOilPlatform()
    motorD.run_for_degrees(degrees=-600, speed=40)
    #working 4th run
    #drive(speed = 10, distanceInCM = _CM_PER_INCH*4, target_angle = 0)
    #wiggleOilPlatform()
    #motorF.run_for_degrees(degrees=-600, speed=40)
    motors.move(amount = 10, unit = "in", steering = 0, speed = -30)

def wiggleOilPlatform():
    _turnToAngle(2)
    _turnToAngle(-1)
    _turnToAngle(2)
    # turnToAngle(-2)
    _turnToAngle(0)

def goBackHomeFromOilPlatform():
    _turnToAngle(60)
    motors.move(amount = 20, unit = "in", steering = 0, speed = -40) # Back home doesnt require accuracy
   #turnToAngle(30)
   #motors.move(amount = 11, unit = "in", steering = 0, speed = -30) # Back home doesnt require accuracy

def pullTruckLeftArm():
    motorF.run_for_degrees(degrees=400, speed=100)
    motors.move(amount = 18, unit = "in", steering = 0, speed = 30)

    motorF.run_for_degrees(degrees=-400, speed=100)
    motors.move(amount = 18, unit = "in", steering = 0, speed = -30)
def pullTruckGoStraight():
    # motorF.run_for_degrees(degrees=1000, speed=100)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH * 10, speed=30)
    motorF.run_for_degrees(degrees=-1000, speed=100)
    motors.move(amount = 10, unit = "in", steering = 0, speed = -30)

def pullTruck():
    #Move arm up
    motorD.run_for_degrees(degrees=800, speed=100)# actually it is 700
    #motorD.start(100)
    _turnToAngle(-55)
    # drive(speed = 30, distanceInCM = 12, target_angle = -55)
    gyroStraight(targetAngle = -55,  distance = 12, speed=30)
    #motorD.stop()
    time.sleep(5)
    _turnToAngle(-5)
    # drive(speed = 30, distanceInCM = 29.5, target_angle = 0)
    gyroStraight(targetAngle = -2,  distance = 29.5, speed=30)
    # _turnToAngle(-15)
    # drive(speed = 30, distanceInCM = _CM_PER_INCH*16, target_angle = -15)
    # _turnToAngle(-5)
    # drive(speed = 30, distanceInCM = _CM_PER_INCH*1, target_angle =-5)
    motorD.run_for_degrees(degrees=-800, speed=100)
    # # _turnToAngle(-5)
    # #gyroStraight(targetAngle = -15,  distance = _CM_PER_INCH*15, backward=True)
    motors.move(amount = 13, unit = "in", steering = 0, speed = -40)
#endregion Arisha 

#region Anya 
#powerPlant
def runAnya():
    primeHub.motion_sensor.reset_yaw_angle()
    # raiseEnergyUnitCollectingArm(200)# at least 245 But we don't know
    getToPowerPlantFromHome2()
    ReleaseEnergyUnitsLowerFirst()
    # ReleaseEnergyUnitsRaiseFirst()
    goToHome1()
    print("Battery voltage: " + str(hub.battery.voltage()))

def raiseEnergyUnitCollectingArm(deg = 90, raiseArm = True):
    multiplier = 1
    if raiseArm == True:
        multiplier = -1

    motorF.run_for_degrees(degrees=deg * 24 * multiplier, speed=40)

def goToHome1():
    motors.move(amount = 5, unit = "in", steering = 0, speed = -40)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105)#original value -90
    # motors.move(amount = 35, unit = "in", steering = 0, speed = 90)#original speed 40
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 105,  distance = _CM_PER_INCH*35, speed=90)

def getToPowerPlantFromHome2():

    # print('Starting getToPowerPlantFromHome2 function')
    # print('Going forward 9.5 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH + 0,  distance = _CM_PER_INCH*6)
    # print('Turning to angle: -90. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    # print('Going forward 36 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = _CM_PER_INCH*29, speed=60) # was 29 and then 2 more at 40 speed below
    drive(speed= 60,distanceInCM= _CM_PER_INCH*29, target_angle= ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    time.sleep(10)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = _CM_PER_INCH*2, speed=40)
    drive(speed= 40,distanceInCM= _CM_PER_INCH*2, target_angle= ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    time.sleep(10)

    ToyFactory()

    # print('Turning to angle: -135. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # turnToAngle(targetAngle = -135, speed = 25)
    # print('Going back 2 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # gyroStraight(targetAngle=-135, distance=_CM_PER_INCH*5, backward = True)
    # print('Turning to angle: -179. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))

    _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179, speed=25)
    # turnToAngle(targetAngle=170, speed=25)
    time.sleep(5)
    # print('Going forward 5 in. Current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179,  distance = _CM_PER_INCH*8.5)
    # gyroStraight(targetAngle = 170,  distance = _CM_PER_INCH*4)
    # _turnToAngle(targetAngle=ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179, speed = 25)
    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 179,  distance = _CM_PER_INCH*3)
    # print('current yaw angle ' +  str(primeHub.motion_sensor.get_yaw_angle()))
    # print('getToPowerPlantFromHome2 function Done')

def ToyFactory():
    # time.sleep(5)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 110,  distance = _CM_PER_INCH*3, backward=True)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 130)
    # time.sleep(10)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 130,  distance = _CM_PER_INCH*4, backward=True)
    # time.sleep(10)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 130,  distance = _CM_PER_INCH*3)
    _turnToAngle(ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    _driveTillLine(speed = 20, distanceInCM = _CM_PER_INCH*9, target_angle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90)
    gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90,  distance = 5) # Original was 8
    # time.sleep(5)

def ReleaseEnergyUnitsLowerFirst():
    motorD.run_for_degrees(degrees=-130, speed=100)
    #motorD.run_for_degrees(degrees=70, speed=100)
    time.sleep(0.5)
    #motorD.run_for_degrees(degrees=150, speed=100)
    motorD.run_for_degrees(degrees=250, speed=100)
    time.sleep(15)

def ReleaseEnergyUnitsRaiseFirst():
    # logMessage("lower arm", level=3)
    motorD.run_for_degrees(degrees=100, speed=100) #original values 150

    # gyroStraight(targetAngle = ANYA_RUN_START_OFFSET_TO_MAT_NORTH - 90, distance = _CM_PER_INCH*0.5, backward=True)


    # Lower arm after lifting the cover
    motorD.run_for_degrees(degrees=-150, speed=100)#original values -150 working value 100
    # time.sleep(0.2)

    # time.sleep(10)
    # Wait for the Arm to fall down so we can try raising again
    
    motorD.run_for_degrees(degrees=100, speed=100)
    #endregion Anya 

#region Nami
def _run4slider():
    # Drive to the first water unit
    drive(speed= 20,distanceInCM= 13, target_angle= 0)
    _turnToAngle(targetAngle= -30 ,speed= 20)
    drive(speed= 20, distanceInCM=16, target_angle= -30)
    moveArm(degrees= -1000, speed= 30, motor= motorF)
    _turnToAngle(targetAngle=-10, speed=20)
    drive(speed=20, distanceInCM=20, target_Angle= -10)

def _run4WithSolarFarmWithLineFindingWithBigLeftArm():
    # Drive till the hydro plant to pick up the first water
    # unit
    drive(speed=35, distanceInCM=12, target_angle=0)
    _turnToAngle(targetAngle=-37,speed=25)
    drive(speed=35, distanceInCM=17, target_angle=-37)
    _turnToAngle(targetAngle=-5,speed=25)
    
    # Turns towards the n-s black line in front of the power station.
    # drive to catch the line
    _driveTillLine(speed=35, distanceInCM=90, target_angle=-5, colorSensorToUse="Left", blackOrWhite="Black")

    # Turn towards the smart grid and drive forward.
    _turnToAngle(targetAngle=-90,speed=25)
    drive(speed=30, distanceInCM=22, target_angle=-90)
    
    # Turn to pick up the last two water units
    _turnToAngle(targetAngle=-120,speed=20)
    drive(speed=30, distanceInCM=12, target_angle=-120)

    # Turn to catch the e-w black line in front of the smart grid
    _turnToAngle(targetAngle=-90,speed=20)
    _driveTillLine(speed=30, distanceInCM=45, target_angle=-90, colorSensorToUse="Right", blackOrWhite="Black")

    # Drive back a little so we can turn to catch the solar energy units
    gyroStraight(distance=5, speed = 20, backward = True, targetAngle = -90)
    
    # Pick up the first solar energy unit
    _turnToAngle(targetAngle=-110,speed=20)
    drive(speed=25, distanceInCM=20, target_angle=-110)
    _turnToAngle(targetAngle=-160,speed=20)

    # Pick up the next two solar energy units
    drive(speed=25, distanceInCM=21, target_angle=-160)
    _turnToAngle(targetAngle=130,speed=20)
    drive(speed=50, distanceInCM=85, target_angle=130)
    
def _dropWaterUnitesWithSlider():
    drive(speed = 25, distanceInCM = 50, target_angle = 0)
    _turnToAngle(targetAngle = -45, speed = 20)
    drive(speed = 25, distanceInCM = 39, target_angle = -45)
    _turnToAngle(targetAngle = -90, speed = 20)
    drive(speed = 25, distanceInCM = 31, target_angle = -90)
    #motors.move(amount = 17, unit = "cm", steering = 0, speed = -20)
    _turnToAngle(targetAngle = 132, speed = 20)#was 132
    drive(speed = 10, distanceInCM = 11, target_angle = 132)
    moveArm(degrees=1000,speed=25,motor=motorF)
    motors.move(amount = 2, unit = "cm", steering = 0, speed = -20)
    _turnToAngle(targetAngle = -90, speed = 10)


def _run5():
    drive(speed = 25, distanceInCM = 55, target_angle = 0)
    _turnToAngle(targetAngle = -45, speed = 20)
    drive(speed = 25, distanceInCM = 55, target_angle = -45)
    _turnToAngle(targetAngle = -135, speed = 20),
    drive(speed = 25, distanceInCM = 35, target_angle = -135)
    
#endregion Nami

#region Rishabh
def _run1(moveArmDegrees, armSpeed):
    drive(speed = 30, distanceInCM = 50, target_angle = 0) #Drive to Watch Television
    motors.move(amount = 15, unit = "cm", steering = 0, speed = -30) #Backup from Watch Television
    _turnToAngle(targetAngle = -45, speed = 25) #Turn towards Hybrid Car
    drive(speed = 30, distanceInCM = 45, target_angle = -45) #Drive towards Hybrid Car
    _turnToAngle(targetAngle = 45, speed = 25) #Turn towards Wind Turbine
    drive(speed = 30, distanceInCM = 28, target_angle = 45) #Drive towards Wind Turbine and push the lever once

    for i in range(2): #Push the lever the remaining two times
        motors.move(amount = 5, unit = "cm", steering = 0, speed = -20)
        drive(speed = 30, distanceInCM = 5, target_angle = 35)
        time.sleep(0.5)
    motors.move(amount = 11, unit = "cm", steering = 0, speed = -30) #Backup from Wind Turbine
    _turnToAngle(targetAngle = -50, speed = 25) #Turn towards Hybrid Car
    moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF) #Lower the Hybrid Car arm
    drive(speed = 30, distanceInCM = 20, target_angle = -50) # Drive towards the Hybrid Car
    moveArm(degrees = -1 * moveArmDegrees, speed = ceil(-0.75 * armSpeed), motor = motorF) #Raise the Hybrid Car arm and complete the mission
    motors.move(amount = 30, unit = "cm", steering = 0, speed = -30) #Backup from the Hybrid Car
    _turnToAngle(targetAngle = -70, speed = 20) #Turn towards Rechargable Battery
    moveArm(degrees = moveArmDegrees, speed = armSpeed, motor = motorF)
    time.sleep(1)
    motors.move(amount = 70, unit = "cm", steering = 0, speed = -50)
    _turnToAngle(targetAngle = -90, speed = 100)
    
#endregion Rishabh
initialize()
#moveArm(degrees=1000,speed=25,motor=motorF)
#dropWaterUnitesWithSlider()
#moveArm(degrees = 1200, speed = 100, motor = motorF)
#time.sleep(5)
#moveArm(degrees = -1100, speed = 100, motor = motorF)
#motors.move(amount = 20, unit = "cm", steering = 0, speed = -20)
#testTurnToAngle()
#run1()

#run1(75, 75)

#drive(speed=40,distanceInCM= 10, target_angle= 0)
#turnToAngle(targetAngle= -30 ,speed= 20)

#run4()

#doRunWithTiming(run5)
#runArisha()
#GLOBAL_LEVEL = 5
#_turnToAngle(targetAngle = -90, speed = 25)
#drive(speed=25, distanceInCM=30, target_angle=-90, gain = 1)
#_run4WithSolarFarmWithLineFindingWithBigLeftArm()

#raise SystemExit
#raise SystemExit

#run1()


t1_start = time.ticks_ms()
#runArisha()
runAnya()
t1_end = time.ticks_ms()
print("Time taken time taken for this run " + 
str( time.ticks_diff(t1_end,t1_start)) + " milliseconds")
