from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
# Note that the "hub" import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from math import *

AXLE_DIAMETER_CM = 12.7
WHEEL_RADIUS_CM = 4.4
GLOBAL_LEVEL = 0
primeHub = PrimeHub()

# Motor C is the left motor and Motor E is the right motor.
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
    """
    WARNING: This is a very specialed class and only to be used for turn to angle.

    This class produces continous gyro readings. The spike prime
    gyro produces angles between -179 and +179. This class will avoid the 
    jump of values between -179 and +179. Here are examples of the values the
    class produces
    Example 1:
    Gyro readings : -10,-5,-4,-1,0,1,5,8
    Class Output  : -10,-5,-4,-1,0,1,5,8

    Example 2: (note there are two discontinous readings in the series below one from 179 to -179
    and another from -179 to 179)
    Gyro readings : 170,175,179,-179,-178,-170,-175,-179,179,175
    Class Output  : 170,175,179,181,182,190,185,181,179,175

    The way the class works is by converting all the values in the -179 to +179
    range to 0-360. Thus the discontinuity is around the zero angle.
    It then uses the precision term to determine if it is in a condition that 
    is a zero crossing and going to produce a non-continous value.
    It it is in that condition then it converts it into a contunous value.

    WARNING: This class will not work if between two calls to getAngle the
    gyro reading changes more than precision. Precision is a input value
    and can be adjusted. Since it is expected that the class is used in 
    only a special case in turnToAngle.
    """
    
    def __init__(self,startAngle,direction,precision=20) :
        self.startAngle = startAngle
        self.direction = direction
        self.currentAngle = startAngle
        self.precision = precision
        
        self.zeroCrossing = False

    def getAngle(self):
        previousAngle = self.currentAngle
        newAngle = gyroAngleZeroTo360()

        if abs(newAngle - previousAngle) > self.precision:
            # This means we are going to produce a non-continous value.
            if(self.direction == "Right" ):
                # We are supposed to be turning right, however for some reason
                # we read got gyro readings that indicate we are going left
                # This means we got something like 0,1,359... readings.
                # this we should not return the 359, reading, but return -1
                if (newAngle > previousAngle):
                    self.currentAngle = newAngle - 360
                else:
                    self.currentAngle = newAngle + 360
            elif(self.direction == "Left" ):
                self.currentAngle = newAngle - 360 
        else:
            self.currentAngle = newAngle

        return self.currentAngle

    def OLDgetAngle(self):
        previousAngle = self.currentAngle
        newAngle = gyroAngleZeroTo360()
        
        # Check if we are in zero crossing stage
        if(self.direction == "Left" and ((previousAngle >= 0 and previousAngle < self.startAngle) and (newAngle > 350 and newAngle < 360) or (self.startAngle == 0))):
            self.zeroCrossing = True
        if(self.direction == "Right" and (previousAngle < 360 and previousAngle > self.startAngle) and (newAngle >= 0 and newAngle < 10) and self.startAngle != 0):
            self.zeroCrossing = True   

        if(self.zeroCrossing == True):
            if(self.direction == "Right" ):
                self.currentAngle = newAngle + 360

            if(self.direction == "Left" ):
                if (newAngle != 0):
                    self.currentAngle = newAngle - 360 
                else:
                    self.currentAngle = newAngle
        else:
            self.currentAngle = newAngle
        
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

def turnRobotRelative(degreesToTurn, speed=20, slowTurnRatio=0.2, correction=0):
    """Turns the robot the specified angle.

    degreesToTurn: The degrees to turn, use +ve numbers to turn right and -ve numbers to turn left.
    speed: The speed to turn at.
    slowTurnRation: The slow turn ratio.
    correction: The correction to use.
    """
    if (degreesToTurn > 179 or degreesToTurn < -179):
        raise ValueError("degreesToTurn should be between +179 and -179")

    degreesToTurn = degreesToTurn * (1 - correction)

    motors.stop()
    primeHub.motion_sensor.reset_yaw_angle()
    currentAngle = gyroAngleZeroTo360()
    targetAngle = currentAngle + degreesToTurn
    
    logMessage("Current Angle:" + str(currentAngle) + " Target angle:" + str(targetAngle) + " degreesToTurn:" + str(degreesToTurn), level=4)

    fastTurnDegrees =  (1 - slowTurnRatio) * abs(degreesToTurn)    
    #motorCInitialDeg = abs(motorC.get_degrees_counted())
    #motorEInitialDeg = abs(motorE.get_degrees_counted())

    SLOW_TURN_SPEED = 10
    startAngle=currentAngle
    slowTurndegrees = slowTurnRatio * abs(degreesToTurn)
    while (abs(currentAngle - targetAngle) > 1):
        degreestravelled = abs(currentAngle-startAngle)
        if(degreestravelled<fastTurnDegrees):
            slowTurnSpeed = int(speed + ((SLOW_TURN_SPEED-speed) * degreestravelled/fastTurnDegrees))
        else:
            slowTurnSpeed = SLOW_TURN_SPEED

        if (currentAngle > targetAngle):
            motors.start_tank(slowTurnSpeed * -1, slowTurnSpeed)
            logMessage("In SlowTurn left turn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed), level=5)
        else:
            motors.start_tank(slowTurnSpeed, slowTurnSpeed * -1)
            logMessage("In SlowTurn right turn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed), level=5)
        currentAngle = gyroAngleZeroTo360()      

        """
        motorCDiff = abs(motorC.get_degrees_counted()) - motorCInitialDeg
        motorEDiff = abs(motorE.get_degrees_counted()) - motorEInitialDeg
        avgDiff = (abs(motorCDiff) + abs(motorEDiff)) / 2
        robotTurn = 0.346*avgDiff
        logMessage("In SlowTurn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed) + " motorCDegDiff=" + 
            str(motorCDiff) + " motorEDegDiff=" + str(motorEDiff) + " expectedRobotTurn=" + str(robotTurn), level=5)
        """         

    motors.stop()
    currentAngle = gyroAngleZeroTo360()
    logMessage("TurnToAngle complete current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=4)
    

def turnToAngle(targetAngle, speed=20, forceTurn="None", slowTurnRatio=0.2, correction=0.05):
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
    correction -- The correction value in ratio. If its set to 0.05, we are going to 
    addjust the turnAngle by 5%, if you dont want any correction set it to 0
    """
    motors.stop()
    #motors.set_stop_action("coast")
    currentAngle = gyroAngleZeroTo360()
    
    # Next make the targetAngle between 0-360
    targetAngle = targetAngle * (1 - correction)
    if (targetAngle < 0):
        targetAngle = targetAngle + 360

    logMessage("TurnToAngle current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=4)

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

    logMessage("Continous target angle:" + str(targetAngle) + " direction:" + direction + " degreedToTurn:" + str(degreesToTurn), level=4)
    _turnRobotWithSlowDown(degreesToTurn, targetAngle, speed, slowTurnRatio, direction)

    motors.stop()
    #motors.set_stop_action("brake")
    currentAngle = gyroAngleZeroTo360()
    logMessage("TurnToAngle complete current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=4)

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
    motorCInitialDeg = abs(motorC.get_degrees_counted())
    motorEInitialDeg = abs(motorE.get_degrees_counted())

    currentAngle = gyroAngleZeroTo360()
    startAngle=currentAngle
    slowTurndegrees = slowTurnRatio * abs(angleInDegrees)
    continuousAngle = ContinousAngle(currentAngle, direction)
    logMessage("After fast turn current_angle:" + str(currentAngle) + " targetAngle:" + str(targetAngle), level=3)
    
    while (abs(currentAngle - targetAngle) > 1):
        degreestravelled = abs(currentAngle-startAngle)
        if(degreestravelled<fastTurnDegrees):
            slowTurnSpeed = int(speed + ((10-speed) * degreestravelled/fastTurnDegrees))
        else:
            slowTurnSpeed = 10
        if (currentAngle > targetAngle):
            motors.start_tank(slowTurnSpeed * -1, slowTurnSpeed)
            logMessage("In SlowTurn left turn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed), level=5)
        else:
            motors.start_tank(slowTurnSpeed, slowTurnSpeed * -1)
            logMessage("In SlowTurn right turn, current_angle:" + str(currentAngle) + " speed=" + str(slowTurnSpeed), level=5)
        currentAngle = continuousAngle.getAngle()      

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

    if (target_angle == -180):
        target_angle = 180

    # Drop the speed from speed to five in distanceInDeg.
    distanceInDegTravelled = 0
    while  distanceInDegTravelled <= distanceInDeg:
        if (slowDown == True):
            currentSpeed = (distanceInDegTravelled * (10 - speed) / (distanceInDeg)) + speed
        current_yaw_angle = primeHub.motion_sensor.get_yaw_angle()

        # This hackery is needed to handle 180 or -180 straight run.
        if (target_angle == 180 and current_yaw_angle < 0):
            current_yaw_angle = (360 + current_yaw_angle)

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
 

def testTurnToAngle():
    # TurnToAngle Testing
    
    #turnRobotRelative(degreesToTurn=90, speed=25, slowTurnRatio = 0.2, correction = 0.05)
    turnToAngle(targetAngle = 90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4, correction = 0.05)
    time.sleep(1)
    turnToAngle(targetAngle = 180, speed = 25, forceTurn = "None", slowTurnRatio = 0.4, correction = 0.05)
    time.sleep(1)
    turnToAngle(targetAngle = -90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4, correction = 0.05)
    time.sleep(1)
    turnToAngle(targetAngle = 0, speed = 25, forceTurn = "None", slowTurnRatio = 0.4, correction = 0.05)
    time.sleep(1)
    turnToAngle(targetAngle = 90, speed = 25, forceTurn = "None", slowTurnRatio = 0.4, correction = 0.05)

    """
    # Right turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Left turn zero crossing.
    turnToAngle(targetAngle = -45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Right turn zero crossing.
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    """
    
    """
    # Right turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Right turn non zero crossing
    turnToAngle(targetAngle = 135, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Right turn non zero crossing
    turnToAngle(targetAngle = -135, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Right turn non zero crossing
    turnToAngle(targetAngle = -45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Left turn non zero crossing
    turnToAngle(targetAngle = -135, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Left turn non zero crossing
    turnToAngle(targetAngle = 135, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)

    # Left turn non zero crossing
    turnToAngle(targetAngle = 45, speed = 25, forceTurn = "None", slowTurnRatio = 0.4)
    """
 # ------------------------------------------------------------------- End Utilities --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 # ------------------------------------------------------------------- Arisha OIL platform --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def runArisha():
    hub.motion_sensor.reset_yaw_angle()
    unloadEnergyUnits()
    # unLoadEnergyStorage()

def unLoadEnergyStorage():
    #Move the arm up
    #motorF.run_for_degrees(degrees=800, speed=40)
    #turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*10)
    turnToAngle(0)


def unloadEnergyUnits():
    #motorF.run_for_degrees(degrees=-400, speed=40)
    #Move the arm down
    motorF.run_for_degrees(degrees=-200, speed=40)

    # motor_pair.move(_CM_PER_INCH*26, 'cm',0,50)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*2,'cm',0,20)
    # motor_pair.move(_CM_PER_INCH*-2,'cm',0,20)
    
    print('current yaw angle ' +  str(hub.motion_sensor.get_yaw_angle()))
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*10)
    print('current yaw angle ' +  str(hub.motion_sensor.get_yaw_angle()))
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward =True)
    turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*2, backward=True)
    turnToAngle(0)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3)
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*3, backward=True)
    # turnToAngle(45)
    # gyroStraight(targetAngle = 45,  distance = _CM_PER_INCH*7)

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


# endregion Arisha 

# ------------------------------------------------------------------- END Arisha OIL platform --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
def run4():
    drive(speed = 30, distanceInCM = 3, target_angle = 0)
    turnToAngle(targetAngle=55, speed=20)
    drive(speed= 30, distanceInCM= 25, target_angle= 45)

def run1():
    drive(speed = 30, distanceInCM = 50, target_angle = 0)
    logMessage("Gyro after 1st move" + str(gyroAngleZeroTo360()), 3)
    motors.move(amount = 15, unit = "cm", steering = 0, speed = -30)
    logMessage("Gyro after 2nd move" + str(gyroAngleZeroTo360()), 3)
    turnToAngle(targetAngle = -45, speed = 25)
    logMessage("Gyro after 3rd move" + str(gyroAngleZeroTo360()), 3)
    drive(speed = 30, distanceInCM = 45, target_angle = -45)
    logMessage("Gyro after 4th move" + str(gyroAngleZeroTo360()), 3)
    turnToAngle(targetAngle = 45, speed = 25)
    logMessage("Gyro after 5th move" + str(gyroAngleZeroTo360()), 3)
    drive(speed = 30, distanceInCM = 23, target_angle = 45)
    logMessage("Gyro after 6th move" + str(gyroAngleZeroTo360()), 3)

    for i in range(2):
        motors.move(amount = 5, unit = "cm", steering = 0, speed = -20)
        logMessage("Gyro after 7th move" + str(gyroAngleZeroTo360()), 3)
        drive(speed = 30, distanceInCM = 5, target_angle = 35)
        logMessage("Gyro after 8th move" + str(gyroAngleZeroTo360()), 3)
        time.sleep(0.5)
    motors.move(amount = 25, unit = "cm", steering = 0, speed = -30)
    logMessage("Gyro after 9th move" + str(gyroAngleZeroTo360()), 3)
    # turnToAngle(targetAngle = -45, speed = 25)
    # logMessage("Gyro after 10th move" + str(gyroAngleZeroTo360()), 3)
    # drive(speed = 30, distanceInCM = 30, target_angle = -45)
    # logMessage("Gyro after 11th move" + str(gyroAngleZeroTo360()), 3)

initialize()
#testTurnToAngle()

run1()
#run4()
#raise SystemExit

#runArisha()
