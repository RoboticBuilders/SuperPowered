from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
# Note that the "hub" import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from math import *

AXLE_DIAMETER_CM = 12.7
WHEEL_RADIUS_CM = 4.5
GLOBAL_LEVEL = 0
primeHub = PrimeHub()

motorC = Motor("C")
motorE = Motor("E")

motors = MotorPair('C', 'E')

#right med motor
motorD = Motor("D")
#left med motor
motorF = Motor("F")
_CM_PER_INCH = 2.54

# -------------------------------------------------------------------  Classes --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class ContinousAngle:

    def __init__(self,startAngle,direction) :
        self.startAngle = startAngle
        self.direction = direction
        self.currentAngle = startAngle
    
    def getAngle(self):
         self.currentAngle = gyroAngleZeroTo360()
        
         if(self.direction == "Right" and self.startAngle > self.currentAngle):
           self.currentAngle = self.currentAngle + 360

         if(self.direction == "Left" and self.startAngle < self.currentAngle):
            self.currentAngle = self.currentAngle - 360
         return self.currentAngle


# -------------------------------------------------------------------  Utilities --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def initialize():
    print("___________________________________________________")
    primeHub.motion_sensor.reset_yaw_angle()
    motors.set_stop_action("brake")
    motors.set_motor_rotation(2*3.14*WHEEL_RADIUS_CM, 'cm')
    isBatteryGood()

def logMessage(message = "", level=1):
    """
    level: parameter between 1-5. 5 is the most detailed level.

    Prints the message that is passed to the function
    The printing is controlled by the level parameter.
    The function will only print if the passed level is higher than the global level.
    If the global level is set to zero nothing will print.
    """
    if level == 0:
        level = 1

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
    while abs(currentDegrees) - abs(startDegrees) < abs(degrees):
        currentDegrees = motor.get_degrees_counted()
        #print("MotorD smaller: " + str(abs(currentDegrees) - abs(startDegrees)))
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

def turnToAngle(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.2):
    """Turns the robot the specified angle.
    It calculates if the right or the left turn is the closest
    way to get to the target angle. Can handle both negative 
    targetAngle and negative gyro readings.
    targetAngle -- the final gyro angle to turn the robot to
    speed -- the speed to turn.
    forceTurn -- Can be "None", "Right" or "Left" strings, forcing
    the robot to turn left or right independent of the shortest 
    path.
    slowTurnRatio -- A number between 0.1 and 1.0. Controls the 
    amount of slow turn. If set to 1.0 the entire turn is a slow turn
    the default value is 0.2, or 20% of the turn is slow.
    """
    motors.stop()
    motors.set_stop_action("coast")
    currentAngle = gyroAngleZeroTo360()
    
    # Next make the targetAngle between 0-360
    #targetAngle = targetAngle * 0.922
    if (targetAngle < 0):
        targetAngle = targetAngle + 360

    logMessage("TurnToAngle current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=3)

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

    # Set the targetAngle to be in the continous space.
    # i.e. if we cross zero then the target Angle is set to be
    # continous. 
    # for example currentAngle = 10(starting turn angle)
    # and we are turning left, (i.e. that is crossing zero)
    # and targetAngle is 350. Then the targetAngle is set to 
    # -10. Since as the robot moves from 10 to 350, we would like the 
    # readings to go 10...7...3,2,1,0,-1,-2,-3,-4...-10
    # instead of 10...7...3,2,1,0,359,358,357....350.
    if(direction == "Right" and currentAngle > targetAngle):
        targetAngle = targetAngle + 360

    if(direction == "Left" and currentAngle < targetAngle):
        targetAngle = targetAngle - 360  

    logMessage("Continous target angle:" + str(targetAngle) + " direction:" + direction + " degreedToTurn:" + str(degreesToTurn), level=3)
    # _turnRobot(degreesToTurn, targetAngle, speed, slowTurnRatio, direction)
    _turnRobotWithSlowDown(degreesToTurn, targetAngle, speed, slowTurnRatio, direction)

    motors.stop()
    motors.set_stop_action("brake")
    currentAngle = gyroAngleZeroTo360()
    logMessage("TurnToAngle complete current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=3)
    
# This is an internal function, do not call this directly.
def _turnRobot(angleInDegrees, targetAngle, speed, slowTurnRatio, direction):
    """Turns the Robot accurately using robot.drive.
    This method first turns the robot with
    speed using robot.turn for a % of the angleInDegrees
    determined by slowTurnRatio. For the remainder of the turn
    i.e. slowturnRatio * angleInDegrees it turns using robot.drive
    with a speed of 30.
    angleInDegrees -- Angle in degrees to turn. Can be +ve or -ve.
    speed -- Fast turn speed. 
    slowTurnRatio -- This is the % of the turn that we want to slow turn.
                     For example 0.2 means that 20% of the turn we want
                     to slow turn.
    """
    fastTurnDegrees =  (1 - slowTurnRatio) * abs(angleInDegrees)    
    continuousAngle = ContinousAngle(gyroAngleZeroTo360(), direction)

    # Initial Fast turn.
    arcLengthInCM = (22*AXLE_DIAMETER_CM*fastTurnDegrees)/(7*360)
    fastTurnMotorRotationDegrees = converCMToDeg(arcLengthInCM)
    if (direction == "Left"):
        motors.move_tank(fastTurnMotorRotationDegrees, "degrees", speed * -1, speed)
    else:
        motors.move_tank(fastTurnMotorRotationDegrees, "degrees", speed, -1 * speed)
   
    currentAngle = continuousAngle.getAngle()
    logMessage("After fast turn current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=3)
    
    slowTurnSpeed = 5
    while (abs(currentAngle - targetAngle) > 1):
        if (currentAngle > targetAngle):
            motors.start_tank(slowTurnSpeed * -1, slowTurnSpeed)
            #motors.move_tank(2.8, "degrees", slowTurnSpeed * -1, slowTurnSpeed)
            logMessage("In SlowTurn left turn, current_angle:" + str(currentAngle), level=5)
        else:
            motors.start_tank(slowTurnSpeed, slowTurnSpeed * -1)
            #motors.move_tank(2.8, "degrees", slowTurnSpeed, -1 * slowTurnSpeed)
            logMessage("In SlowTurn right turn, current_angle:" + str(currentAngle), level=5)
        currentAngle = continuousAngle.getAngle()


def _turnRobotWithSlowDown(angleInDegrees, targetAngle, speed, slowTurnRatio, direction):
    """Turns the Robot accurately using robot.drive.
    This method first turns the robot with
    speed using robot.turn for a % of the angleInDegrees
    determined by slowTurnRatio. For the remainder of the turn
    i.e. slowturnRatio * angleInDegrees it turns using robot.drive
    with a speed of 30.
    angleInDegrees -- Angle in degrees to turn. Can be +ve or -ve.
    speed -- Fast turn speed. 
    slowTurnRatio -- This is the % of the turn that we want to slow turn.
                     For example 0.2 means that 20% of the turn we want
                     to slow turn.
    """
    fastTurnDegrees =  (1 - slowTurnRatio) * abs(angleInDegrees)    
    continuousAngle = ContinousAngle(gyroAngleZeroTo360(), direction)

    # Initial Fast turn.
    
   
    currentAngle = continuousAngle.getAngle()
    startAngle=currentAngle
    slowTurndegrees = slowTurnRatio * abs(angleInDegrees)
    logMessage("After fast turn current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=3)
    
    
    while (abs(currentAngle - targetAngle) > 1):
        slowTurnSpeed = int(speed+((10-speed)*(currentAngle-startAngle)/(angleInDegrees - slowTurndegrees)))
        if (currentAngle > targetAngle):
            motors.start_tank(slowTurnSpeed * -1, slowTurnSpeed)
            logMessage("In SlowTurn left turn, current_angle:" + str(currentAngle), level=5)
        else:
            motors.start_tank(slowTurnSpeed, slowTurnSpeed * -1)
            logMessage("In SlowTurn right turn, current_angle:" + str(currentAngle), level=5)
        currentAngle = continuousAngle.getAngle()        
   
def turnToAngleTrial(targetAngle = 0, speed = 25, forceTurn = "None", slowTurnRatio = 0.4):
    """
    Turns the robot using an algorithm of a fast turn followed by a slow turn.
    targetAngle: This should be an angle between -179 and +179.
    speed: Speed to turn at.
    forceTurn: use this to tell the robot to turn in a certain direction, otherwise this function figures that out.
    slowTurnRatio: This is the amount of turn that we want to slow turn. Balance between speed and accuracy.
    """
    # Initialize function
    motors.stop()
    currentAngle = gyroAngleZeroTo360()

    if targetAngle < 0:
        targetAngle = targetAngle + 360

    logMessage("TurnToAngle current angle is " + str(currentAngle) + " and target angle is " + str(targetAngle), level=3)
    
    # Detect which way is faster to turn
    degreesToTurnRight = 0
    degreesToTurnLeft = 0
    
    if targetAngle > currentAngle:
        degreesToTurnRight = targetAngle - currentAngle
        degreesToTurnLeft = 360 + currentAngle - targetAngle

    else:
        degreesToTurnRight = 360 - currentAngle + targetAngle
        degreesToTurnLeft = currentAngle - targetAngle

    logMessage("degreesToTurnLeft is " + str(degreesToTurnLeft) + " degreesToTurnRight is: " + str(degreesToTurnRight), level=4)
    isZeroCrossing = False
    if forceTurn == "None":
        if degreesToTurnLeft < degreesToTurnRight:
            logMessage("Turning left", level=4)
            if (targetAngle > currentAngle):
                isZeroCrossing = True
            _turnRobotForPrime(currentAngle, targetAngle, degreesToTurnLeft, "Left", speed, slowTurnRatio, isZeroCrossing)
        else:
            logMessage("Turning right", level=4)
            if (targetAngle < currentAngle):
                isZeroCrossing = True
            _turnRobotForPrime(currentAngle, targetAngle, degreesToTurnRight, "Right", speed, slowTurnRatio, isZeroCrossing)

    motors.stop()
    currentAngle = gyroAngleZeroTo360()
    logMessage("Turn finished. currentAngle=" + str(currentAngle) + " targetAngle="+str(targetAngle), level=4)

    """
    elif forceTurn == "Right":
        logMessage("Turning right", level=4)
        _turnRobotForPrime(targetAngle, degreesToTurnRight, "Right", speed, slowTurnRatio)
    
    elif forceTurn == "Left":
        logMessage("Turning left", level=4)
        _turnRobotForPrime(targetAngle, degreesToTurnLeft, "Left", speed, slowTurnRatio)
    """        

def _turnRobotForPrime(startAngle, targetAngle, angleInDegrees, direction, speed, slowTurnRatio, isZeroCrossing):
    """
    INTERNAL FUNCTION. 

    This is an internal function do not call this directly. Use the turnToAngle
    instead.

    This function turns the robot. It operates in the space of 0-360. Note that the
    spike prime gyro operates in the -179 to +179d space. Expects all inputs in the
    0-360 space and also reads the gyro in the 0-360 space.

    startAngle: The angle where we started.
    targetAngle: The final target Angle. Should be between 0 - 360
    angleInDegrees: The angle to turn should be between 0 - 360
    direction: String should be one of "Left" "Right"
    speed: The speed to turn at
    slowTurnRatio: The slow turn ratio to use.
    isZeroCrossing: If the turn will result in the robot crossing the zero angle.
    """
    # Direction should be one of "Left" or "right"
    if (direction != "Left" and direction != "Right"):
        raise AssertionError

    # targetAngle should be between 0-360
    if (targetAngle < 0 or targetAngle > 360):
        raise AssertionError

    # angleInDegrees should be between 0-360
    if (angleInDegrees < 0 or angleInDegrees > 360):
        raise AssertionError
    
    motors.stop()
    motors.set_stop_action("coast")
    
    slowTurnSpeed = 10
    if (isZeroCrossing == False):   
        # Calculate the initial turn distance
        slowTurnDegrees = slowTurnRatio * abs(angleInDegrees)
        fastTurnDegrees = angleInDegrees - slowTurnDegrees

        logMessage("Non Zero Crossing. Doing a fast turn followed by a slow turn", level=4) 
        _nonZeroCrossingTurn(direction, fastTurnDegrees, targetAngle, slowTurnSpeed, speed)
    else:
        logMessage("Zero Crossing. Turning: " + direction + " . Turning to zero", level=4)
        currentAngle = gyroAngleZeroTo360()
        degreesToTurnToZero = 0
        if (direction == "right"):
            degreesToTurnToZero = 360 - currentAngle
        else:
            degreesToTurnToZero = currentAngle

        # First turning to zero
        arcLengthInCM = (22*AXLE_DIAMETER_CM*degreesToTurnToZero)/(7*360)
        turnToZeroMotorRotationDegrees = converCMToDeg(arcLengthInCM)
        if (direction == "Left"):
            motors.move_tank(turnToZeroMotorRotationDegrees, "degrees", speed * -1, speed)
        else:
            motors.move_tank(turnToZeroMotorRotationDegrees, "degrees", speed, -1 * speed)    

        # Make sure we are past zero
        currentAngle = gyroAngleZeroTo360()
        if (direction == "Left"):
            # This means we are not past zero.
            while (currentAngle < startAngle):
                logMessage("Ensuring zero crossing. Left turn, currentAngle:" + str(currentAngle), level=4) 
                motors.move_tank(turnToZeroMotorRotationDegrees, "degrees", slowTurnSpeed * -1, slowTurnSpeed)
                currentAngle = gyroAngleZeroTo360()
        else:
            # This means we are not past zero.
            while (currentAngle > targetAngle):
                logMessage("Ensuring zero crossing. Right turn, currentAngle:" + str(currentAngle), level=4) 
                motors.move_tank(turnToZeroMotorRotationDegrees, "degrees", slowTurnSpeed, -1 * slowTurnSpeed)
                currentAngle = gyroAngleZeroTo360()

        # Now turn the non-zero crossing turn for the remaining turn
        logMessage("Past zero crossing. Doing a fast turn followed by a slow turn", level=4) 
        currentAngle = gyroAngleZeroTo360()
        degreesToTurn = abs(currentAngle - targetAngle)
        _nonZeroCrossingTurn(direction, degreesToTurn, targetAngle, slowTurnSpeed, speed)

def _nonZeroCrossingTurn(direction, fastTurnDegrees, targetAngle, slowTurnSpeed, speed):
    """
    INTERNAL FUNCTION DO NOT CALL DIRECTLY
    Current and target angles are all always between 0-360
    """
    logMessage("First doing a fast turn. currentAngle=" + str(gyroAngleZeroTo360()) + " targetAngle:" + str(targetAngle), level=4)   

    # Turn the initial amount at a fast rate.
    # Figure out the amount ot degrees to run using the axle diameter.
    arcLengthInCM = (22*AXLE_DIAMETER_CM*fastTurnDegrees)/(7*360)
    fastTurnMotorRotationDegrees = converCMToDeg(arcLengthInCM)
    if (direction == "Left"):
        motors.move_tank(fastTurnMotorRotationDegrees, "degrees", speed * -1, speed)
    else:
        motors.move_tank(fastTurnMotorRotationDegrees, "degrees", speed, -1 * speed)

    logMessage("Starting slow turn. currentAngle=" + str(gyroAngleZeroTo360()) + " targetAngle:" + str(targetAngle), level=4)   
    _simpleTurn(targetAngle=targetAngle, speed=slowTurnSpeed)
    
def _simpleTurn(targetAngle, speed):
    """
    INTERNAL FUNCTION DO NOT CALL DIRECTLY
    Current and target angles are all always between 0-360
    """
    currentAngle = gyroAngleZeroTo360()
    while abs(currentAngle - targetAngle) > 1:
        if currentAngle > targetAngle:
            logMessage("Turning left. CurrentAngle: " + str(currentAngle) + "  targetAngle: " + str(targetAngle), level=5)
            motors.start_tank(speed * -1, speed)
        else:
            logMessage("Turning right. CurrentAngle: " + str(currentAngle) + "  targetAngle: " + str(targetAngle), level=5)
            motors.start_tank(speed, speed * -1)
        currentAngle = gyroAngleZeroTo360()
    
def _turnRobotOld(angleInDegrees, speed, slowTurnRatio):
    # Initialize the function
    motors.stop()
    slowTurnDegrees = slowTurnRatio * abs(angleInDegrees)
    initialAngle = primeHub.motion_sensor.get_yaw_angle()
    finalAngle = initialAngle + angleInDegrees
    #print("TurnToAngle initial gyro angle is " + str(initialAngle) + " and target angle is " + str(finalAngle))
    initialTurn = 0

    # Calculate the initial turn distance
    if angleInDegrees > 0:
        initialTurn = angleInDegrees - slowTurnDegrees
    else:
        initialTurn = angleInDegrees + slowTurnDegrees

    # Turn the initial amount
    motors.move_tank(initialTurn, "degrees", speed, -1 * speed)
    #motors.stop() #Commented out to stop jerking between 2 parts of turn

    # Initialize slow turn
    initialSlowSpeed = 5

    robotAngle = primeHub.motion_sensor.get_yaw_angle()
    #print("TurnToAngle gyro angle before slow turn is " + str(robotAngle))

    # Turn slowly
    while abs(robotAngle - finalAngle) > 1:
        if robotAngle > finalAngle:
            logMessage("in robotAngle > finalAngle, RobotAngle is: " + str(robotAngle) + "  finalAngle is: " + str(finalAngle), level=5)
            motors.start_tank(initialSlowSpeed * -1, initialSlowSpeed)
        else:
            logMessage("in robotAngle <= finalAngle, RobotAngle is: " + str(robotAngle) + "  finalAngle is: " + str(finalAngle), level=5)
            motors.start_tank(initialSlowSpeed, initialSlowSpeed * -1)
        robotAngle = primeHub.motion_sensor.get_yaw_angle()
        
    motors.stop()

def testTurnToAngle():
    # TurnToAngle Testing
    #primeHub.motion_sensor.reset_yaw_angle()

    turnToAngle(targetAngle = 90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    #time.sleep(1)
    #turnToAngle(targetAngle = -90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    #time.sleep(1)
    #turnToAngle(targetAngle = -90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    #time.sleep(1)
    #turnToAngle(targetAngle = -90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    #time.sleep(1)
    """
    # Right turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Left turn zero crossing.
    turnToAngle(targetAngle = -45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Right turn zero crossing.
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)
    """
    
    """
    # Right turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Right turn non zero crossing
    turnToAngle(targetAngle = 135, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Right turn non zero crossing
    turnToAngle(targetAngle = -135, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Right turn non zero crossing
    turnToAngle(targetAngle = -45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Left turn non zero crossing
    turnToAngle(targetAngle = -135, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Left turn non zero crossing
    turnToAngle(targetAngle = 135, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)

    # Left turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.2)
    """
    
def gyroStraight(distance, speed = 20, backward = False, targetAngle = 0):
    initialDeg = abs(motorC.get_degrees_counted())

    print("GYROSTRAIGHT START: targetAngle  is " + str(targetAngle))
    degreesToCover = (distance * 360)/(WHEEL_RADIUS_CM * 2 * 3.14)
    position_start = motorE.get_degrees_counted()
    if (backward): 
        while ((motorE.get_degrees_counted() - position_start)  >= degreesToCover * -1):
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = targetAngle - currentAngle
            motors.start(correction, speed * -1)
    else:
         while ((motorE.get_degrees_counted() - position_start)  <= degreesToCover):
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = targetAngle - currentAngle
            motors.start(correction, speed)

    motors.stop()

    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distance-totalDistanceTravelled), level=4)

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
    motors.stop()
    logMessage("driveStraight for distance: " + str(distanceInCM) + " and target angle: " + str(target_angle), level=4)
    initialDeg = abs(motorC.get_degrees_counted())

    # If the distance is small, then just drive over that distance at speed.
    if (distanceInCM < 5):
        _driveStraightWithSlowDown(distanceInCM, speed, target_angle, gain, slowDown=False)    

    # First drive 80% of the distance at
    distance80 = distanceInCM * 0.8
    _driveStraightWithSlowDown(distance80, speed, target_angle, gain, slowDown=False)

    # This code is commented for now, the above code seems to be providing the best algorithm
    # We will reevaluate if needed.
    # Drive the next 16% of the distance at a speed that reduces from speed to speed=FINAL_SLOW_SPEED
    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("Distance travelled after first part = "  + str(distanceTravelled) + " error=" + str(distanceTravelled-distance80), level=4)

    distance16 = remainingDistance * 0.8
    _driveStraightWithSlowDown(distance16, speed, target_angle, gain, slowDown=True)

    distanceTravelled = convertDegToCM(abs(motorC.get_degrees_counted()) - initialDeg)
    remainingDistance = distanceInCM - distanceTravelled
    logMessage("Distance travelled after second part= "  + str(distanceTravelled) + " error=" + str(distanceTravelled - (distance80+distance16)), level=4)

    # Drive the final 4% of the distance at speed 5
    FINAL_SLOW_SPEED = 5
    motors.move_tank(remainingDistance, "cm", FINAL_SLOW_SPEED, FINAL_SLOW_SPEED)

    motors.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(totalDistanceTravelled-distanceInCM), level=4)

def _driveStraightWithSlowDown(distance, speed, target_angle, gain, slowDown):
    """
    Drive Straight
    ______________
    This is a internal function do not call directly. Call drive instead.

    The algorithm goes from speed to speed 10 until distance is travelled.
    slowDown: True if you want to slow down over the distance. 
    """
    startDistanceInDeg = abs(motorC.get_degrees_counted())
    logMessage("startDistanceInDeg = " + str(int(startDistanceInDeg)), level=5)
    distanceInDeg = converCMToDeg(distance)
    currentSpeed = speed

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0
    while  distanceInDegTravelled <= distanceInDeg:
        if (slowDown == True):
            currentSpeed = (distanceInDegTravelled * (10 - speed) / (distanceInDeg)) + speed
        current_yaw_angle = primeHub.motion_sensor.get_yaw_angle()
        correction = target_angle - current_yaw_angle
        turn_rate = correction * gain
        logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceInDeg=" + str(distanceInDeg) + " target_angle= " + str(target_angle) + " current_yaw_angle = " + str(current_yaw_angle) +" correction= " + str(correction), level=5)
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
 
 # ------------------------------------------------------------------- End Utilities --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 

 # ------------------------------------------------------------------- Arisha OIL platform --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def runArisha():
    unloadEnergyUnits()

def unloadEnergyUnits():
    #motorF.run_for_degrees(degrees=-400, speed=40)
    #motorF.run_for_degrees(degrees=-400, speed=40)
    # motor_pair.move(_CM_PER_INCH*26, 'cm',0,50)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    #hub.motion_sensor.reset_yaw_angle()
    print('current yaw angle ' +  str(hub.motion_sensor.get_yaw_angle()))
    drive(speed=20, distanceInCM=_CM_PER_INCH*10, targetAngle = 0)
    #turnToAngle(0)
    print('current yaw angle ' +  str(hub.motion_sensor.get_yaw_angle()))
    #turnToAngle(0)
    
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward =True)
    #turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    #turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward=True)
    #turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2)
    turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3, backward=True)
    turnToAngle(0)
    #motorF.run_for_degrees(degrees=400, speed=40)

# def pullTruck():
#     # motor_pair.move(_CM_PER_INCH*26, 'cm',0,50)
#     # left_med_motor.run_for_degrees(degrees=800, speed=40)
#     # motor_pair.move(_CM_PER_INCH*5,'cm',0,20)
#     # time.sleep(0.5)
#     # left_med_motor.run_for_degrees(degrees=-800, speed=40)
#     # time.sleep(0.5)
#     # motor_pair.move(_CM_PER_INCH*-5,'cm',0,20)
#     hub.motion_sensor.reset_yaw_angle()
#     #motor_pair.start(0, -20  )

#     left_med_motor.run_for_degrees(degrees=900, speed=40)
#     gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*7)
#     time.sleep(0.5)
#     left_med_motor.run_for_degrees(degrees=-850, speed=40)
#     time.sleep(0.5)
#     gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*5, backward=True)
#     #motor_pair.move(_CM_PER_INCH*-5,'cm',0,20)

# ------------------------------------------------------------------- END Arisha OIL platform --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def testAxleLogic(degrees,speed,direction):
    print("Start Test")
    startmotorCDegrees =  motorC.get_degrees_counted()
    startmotorEDegrees = motorE.get_degrees_counted()
    print("MotorC degrees counted: " + str(startmotorCDegrees) + " motorE degrees counted: " + str(startmotorEDegrees))
    arcLengthInCM = (22*AXLE_DIAMETER_CM*degrees)/(7*360)
    toturndegrees = converCMToDeg(arcLengthInCM)

    print("Arc Length in CM: " + str(arcLengthInCM) + " toturndegrees: " + str(toturndegrees))
    if (direction == "Left"):
        motors.move_tank(toturndegrees, "degrees", speed * -1, speed)
    else:
        motors.move_tank(toturndegrees, "degrees", speed, -1 * speed)
    endmotorCDegrees =  motorC.get_degrees_counted()
    endmotorEDegrees = motorE.get_degrees_counted()
    print("MotorC degrees counted: " + str(endmotorCDegrees) + " motorE degrees counted: " + str(endmotorEDegrees))
    print("motorCDiff: " + str(endmotorCDegrees -startmotorCDegrees) + " motor E Diff: " + str(endmotorEDegrees - startmotorEDegrees))
    


def squareTest():
    # Run on a square of 30cm
    angle = 0
    for i in range(4):
        drive(speed = 30, distanceInCM = 30, target_angle = angle)
        angle += 90
        turnToAngle(targetAngle = angle, speed = 30)

def run4():
    drive(speed = 30, distanceInCM = 50, target_angle = 0)
    #gyroStraight(speed = 20, targetAngle = 0,  distance = 20, backward = False)

def testGyroTurn(targetAngle,speed):
    
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    startAngle = currentAngle
    degreesToTurn = targetAngle - currentAngle
    slowTurndegrees = degreesToTurn * 0.4

    while (abs(currentAngle - targetAngle) > 1):
        slowTurnSpeed = int(speed+((10-speed)*(currentAngle-startAngle)/(degreesToTurn - slowTurndegrees)))
        if (currentAngle > targetAngle):
            motors.start_tank(slowTurnSpeed * -1, slowTurnSpeed)
            #motors.move_tank(2.8, "degrees", slowTurnSpeed * -1, slowTurnSpeed)
            #logMessage("In SlowTurn left turn, current_angle:" + str(currentAngle) + " speed= " + str(slowTurnSpeed) + " targetAngle=" + str(targetAngle), level=5)
        else:
            motors.start_tank(slowTurnSpeed, slowTurnSpeed * -1)
            #motors.move_tank(2.8, "degrees", slowTurnSpeed, -1 * slowTurnSpeed)
            #logMessage("In SlowTurn right turn, current_angle:" + str(currentAngle)  + " speed= " + str(slowTurnSpeed) + " targetAngle=" + str(targetAngle), level=5)
        currentAngle =  primeHub.motion_sensor.get_yaw_angle()


initialize()
#run4()
#squareTest()
#testGyroTurn(90,25)
testTurnToAngle()
#testAxleLogic(90,20,"Right")

raise SystemExit

#runArisha()
