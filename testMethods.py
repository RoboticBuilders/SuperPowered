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
    motors.stop()
    time.sleep(2)
    endmotorCDegrees =  motorC.get_degrees_counted()
    endmotorEDegrees = motorE.get_degrees_counted()
    print("MotorC degrees counted: " + str(endmotorCDegrees) + " motorE degrees counted: " + str(endmotorEDegrees))
    print("motorCDiff: " + str(endmotorCDegrees -startmotorCDegrees) + " motor E Diff: " + str(endmotorEDegrees - startmotorEDegrees))
    currentAngle = gyroAngleZeroTo360()
    logMessage("Axle:" + str(currentAngle) , level=3)
  
def squareTest():
    
    drive(speed = 30, distanceInCM = 30, target_angle = 0)

    turnToAngle(targetAngle = -90, speed = 30)
    drive(speed = 30, distanceInCM = 30, target_angle = -90)

    turnToAngle(targetAngle = -180, speed = 30)
    drive(speed = 30, distanceInCM = 30, target_angle = -180)