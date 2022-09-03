from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
# Note that this import is needed, this is different from the PrimeHub import above, this is the way to access the battery.
import time, hub
from math import *

WHEEL_RADIUS_CM = 4.3
GLOBAL_LEVEL = 4
primeHub = PrimeHub()

motorC = Motor("C")
motorE = Motor("E")

motors = MotorPair('C', 'E')

#right med motor
motorD = Motor("D")
#left med motor
motorF = Motor("F")
_CM_PER_INCH = 2.54

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

def turnToAngle(targetAngle = 0, speed = 25, forceTurn = "None", slowTurnRatio = 0.4):
    # Initialize function
    motors.stop()
    currentAngle = primeHub.motion_sensor.get_yaw_angle()
    #print("TurnToAngle current angle is " + str(currentAngle) + " and target angle is" + str(targetAngle))
    
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

    #print("TurnToAngle degrees to turn is " + str(degreesToTurn))

    # Call the internal function that actually turns the robot
    _turnRobot(degreesToTurn, speed, slowTurnRatio)

def _turnRobot(angleInDegrees, speed, slowTurnRatio):
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
            motors.start_tank(initialSlowSpeed * -1, initialSlowSpeed)
        else:
            motors.start_tank(initialSlowSpeed, initialSlowSpeed * -1)
        robotAngle = primeHub.motion_sensor.get_yaw_angle()
        

    motors.stop()

def testTurnToAngle():
    # TurnToAngle Testing
    primeHub.motion_sensor.reset_yaw_angle()
    multiplier = 1
    start = time.ticks_us()
    while multiplier < 5:
        turnToAngle(90 * multiplier, 25)
        multiplier = multiplier + 1
        time.sleep(1)
    time.sleep(1)        
    print("Final Angle is " + str(primeHub.motion_sensor.get_yaw_angle()))
    end = time.ticks_us()
    print("Total time = " + str(end - start))

def gyroStraight(distance, speed = 20, backward = False, targetAngle = 0):
    print("GYROSTRAIGHT START: targetAngle  is " + str(targetAngle))
    degreesToCover = (distance * 360)/(WHEEL_RADIUS_CM * 2 * 3.14)
    position_start = motorE.get_degrees_counted()
    if (backward): 
        while ((motorE.get_degrees_counted() - position_start)  > degreesToCover * -1):
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
            correction = targetAngle - currentAngle
            motors.start(correction, speed * -1)
    else:
         while ((motorE.get_degrees_counted() - position_start)  < degreesToCover):
            currentAngle = primeHub.motion_sensor.get_yaw_angle()
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
    logMessage("driveStraight for distance: " + str(distanceInCM) + " and target angle" + str(target_angle), level=4)
    initialDeg = abs(motorC.get_degrees_counted())

    # If the distance is small, then just drive over that distance at speed.
    if (distanceInCM < 5):
        _driveStraightWithSlowDown(distanceInCM, speed, target_angle, gain, slowDown=False)    

    # First drive 80% of the distance at
    distance80 = distanceInCM * 1.0
    _driveStraightWithSlowDown(distance80, speed, target_angle, gain, slowDown=False)

    # This code is commented for now, the above code seems to be providing the best algorithm
    # We will reevaluate if needed.
    '''
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
    '''
    motors.stop()
    finalDeg = abs(motorC.get_degrees_counted())

    totalDistanceTravelled = convertDegToCM(finalDeg - initialDeg)
    logMessage("Total distance travelled = " + str(totalDistanceTravelled) + " error=" + str(distanceInCM-totalDistanceTravelled), level=4)

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
        
        correction = target_angle - primeHub.motion_sensor.get_yaw_angle()
        turn_rate = correction * gain
        logMessage("currentSpeed = " + str(int(currentSpeed)) + " distanceInDegTravelled = " + str(distanceInDegTravelled) + " distanceInDeg=" + str(distanceInDeg), level=5)
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
    logMessage("Battery voldate: " + str(hub.battery.voltage()), level=1)
    
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
    hub.motion_sensor.reset_yaw_angle()
    print('current yaw angle ' +  str(hub.motion_sensor.get_yaw_angle()))
    gyroStraight(targetAngle = 0,  distance = _CM_PER_INCH*10)
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

def run4():
    drive(speed = 30, distanceInCM = 20, target_angle = 0)

initialize()
run4()
raise SystemExit

#runArisha()
