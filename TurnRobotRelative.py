def turnRobotRelative(degreesToTurn, speed=20, slowTurnRatio=0.2, correction=0):
    """Turns the robot the specified angle.

    degreesToTurn: The degrees to turn, use +ve numbers to turn right and -ve numbers to turn left.
    speed: The speed to turn at.
    slowTurnRation: The slow turn ratio.
    correction: The correction to use.
    """
    global absolute_angle
    if (degreesToTurn > 179 or degreesToTurn < -179):
        raise ValueError("degreesToTurn should be between +179 and -179")

    logMessage("Absolute Angle=" + str(absolute_angle), level=4)
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
    absolute_angle = absolute_angle + degreesToTurn
