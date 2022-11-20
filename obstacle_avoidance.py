import csv, math

class robot:
    currentLocationX = 0
    currentLocationY = 0
    currentRobotAngle = 0
    angle = 0
    slope = 0
    distance = 0
    quadrant2 = 0

    def __init__(self,x,y):
        self.currentLocationX = x
        self.currentLocationY = y
        self.angle = 0
        self.slope = 0
        self.distance = 0
        self.quadrant2 = 0

    def addActions(self, speed, robotActions):
        robotActions.append("_turnToAngle(targetAngle=={}, speed={}".format(self.angle, speed))
        robotActions.append("drive(speed={}, distanceInCM = {}, target_angle = {})".format(speed, self.distance, self.angle))

    def goto(self,x2,y2,endAngle,speed):
        self.angle = 0
        self.slope = 0
        self.quadrant2 = 0
        self.distance = 0
        x1 = self.currentLocationX
        y1 = self.currentLocationY
        a1 = self.currentRobotAngle

        def _calculateSlope(x1,y1,x2,y2):
            lineSlope = (y1 - y2)/(x1-x2)
            self.slope = lineSlope

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

            self.quadrant2 = endQuadrant
            
        def _calculateAngle(slope):
            lineSlope = slope
            if x1 == 0 and x2 == 0:
                self.angle = 0
            angleRadians = math.atan(lineSlope)
            angleDegrees = math.degrees(angleRadians)
            self.angle = round(angleDegrees)

        def _fixAngle(endQuadrant, rAngle):
            turnAngle = rAngle

            if endQuadrant == 3:
                turnAngle = -1 * rAngle - 90

            if endQuadrant == 4:
                turnAngle = -1 * rAngle + 90

            self.angle = round(turnAngle)

        def _findDistance(x1,y1,x2,y2):
            distanceToDrive = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
            self.distance = round(distanceToDrive)

        #def _move(speed):
        #    _turnToAngle(targetAngle = self.angle, speed = speed)
        #    drive(speed = speed, distanceInCM = self.distance, target_angle = self.angle)

        _findQuadrant(x1,y1,x2,y2)
        _calculateSlope(x1,y1,x2,y2)
        _calculateAngle(self.slope)
        _fixAngle(self.quadrant2, self.angle)
        _findDistance(x1,y1,x2,y2)
        #_move(speed)
        #print(str(self.angle))
        #print(str(self.distance))
        self.currentLocationX = x2
        self.currentLocationY = y2
        #_turnToAngle(targetAngle = endAngle, speed = speed)

class Mission(object):
    def __init__(self, bottomLeft, topRight):
        self.bottomLeft = bottomLeft
        self.topRight = topRight

    def findClosestIntersectionPoint(self, robotLine):
        # 1. form all the four lines that the mission is made of.
        # 2. intersect the four lines with the input line
        # 3. Return the intersection point or None.

        lines = []
        end = Point(self.topRight.getX(), self.bottomLeft.getY())
        bottomHorizontalLine = Line(self.bottomLeft, end)
        lines.append(bottomHorizontalLine)

        start = Point(self.bottomLeft.getX(), self.topRight.getY())
        topHorizontalLine = Line(start, self.topRight)
        lines.append(topHorizontalLine)

        end = Point(self.bottomLeft.getX(), self.topRight.getY())
        leftVerticalLine = Line(self.bottomLeft, end)
        lines.append(leftVerticalLine)

        start = Point(self.topRight.getX(), self.bottomLeft.getY())
        rightVerticalLine = Line(start, self.topRight)
        lines.append(rightVerticalLine)

        minDistance = 10000000
        nearestIntersection = None
        for line in lines:
            intersection = line.doesIntersect(robotLine, )
            if (intersection != None):
                distance = intersection.distance(start)
                if (distance < minDistance):
                    minDistance = distance
                    nearestIntersection = intersection

        return nearestIntersection

class Point(object):
    def __init__(self, x, y):
        '''Defines x and y variables'''
        self.X = x
        self.Y = y

    def getX(self):
        return self.X

    def getY(self):
        return self.Y

    def distance(self, point):
        return math.sqrt(math.pow((self.x - point.getX())) + math.pow((self.y - point.getY())))

class Line(object):
    def __init__(self, start, end):
        '''Defines start and end points'''
        self.start = start
        self.end = end

    def getStart(self):
        return self.start

    def getEnd(self):
        return self.end

    def isPointOnLine(self, point):
        '''TODO Returns true if the point is on the line and between start and end points'''
    
    def doesIntersect(self, anotherLine, intersectionPoint):
        ''' TODO: Rishabh write this code.
            returns the intersection point else returns None.
        '''
        # Calculate Slope & Y Intercept of both points
        
        robotLineSlope = (end.getY() - start.getY()) / (end.getX() - start.getX())
        missionLineSlope = (l2y2 - l2y1) / (l2x2 - l2x1)
        robotLineYIntercept = end.getY() - robotLineSlope * end.getX()
        missionLineYIntercept = l2y1 - missionLineSlope * l2x1
        # Solve Both Equations
        intersectionX = (robotLineYIntercept - missionLineYIntercept) / (missionLineSlope - robotLineSlope)
        intersectionY = (robotLineSlope * robotLineYIntercept - robotLineSlope * missionLineYIntercept) / (missionLineSlope - robotLineSlope)
        # Check If Point Is On Lines
        if end.getX() < start.getX():
            lowestX = end.getX()
            highestX = start.getX()

        if end.getX() > start.getX():
            lowestX = start.getX()
            highestX = end.getX()

        if end.getY() < start.getY():
            lowestY = end.getY()
            highestY = start.getY()

        if end.getY() > start.getY():
            lowestY = start.getY()
            highestY = end.getY()

        if intersectionX > lowestX and intersectionX < highestX and intersectionY > lowestY and intersectionY < highestY:
            return intersectionX, intersectionY

        else:
            return None

        return Point(10, 10)

def findAngleToTurnAfterIntersection(mission, robotLine, intersectionPoint):
    
    # TODO Rishabh
    return 90

def findDistanceToTravelAfterIntersection(mission, robotLine, intersectionPoint):
    # TODO Rishabh
    return 10

def calculateNewStartPointAfterInstersection(intersectionPoint, distance, angle):
    # TODO Rishabh
    return Point(100, 100)

def intersectLineWithAllMissions(reader, robotLine):
    # Intersects the robotline with all missions and returns the mission
    # and the intersection point that is closest to the start point of the
    # robot line.

    # select a large value.
    minDistance = 1000000
    nearestIntersection = None
    nearestIntersectionMission = None
    for row in reader:
        bottomLeft = Point(row[1], row[2])
        topRight = Point(row[3], row[4])

        # Form mission.    
        mission = Mission(bottomLeft, topRight)
        intersectionPoint = mission.findClosestIntersectionPoint(robotLine)

        # Calculate distance of the intersection point from the start point.
        distance = intersectionPoint.distance(robotLine.getStart())
        if (distance < minDistance):
            minDistance = distance
            nearestIntersection = intersectionPoint
            nearestIntersectionMission = mission

    if nearestIntersection != None:
        return nearestIntersection, nearestIntersectionMission
    else:
        return None, None
            
def readMissionFile():
    csvfile = open("C:\\Users\\rishabh\Documents\\FLL\\SuperPowered\\mission_coordinates.txt", newline='\n')
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        print(row)
    return reader

def addActions(speed, angle, distance, actions):
    actions.append("_turnToAngle(targetAngle=={}, speed={}".format(angle, speed))
    actions.append("drive(speed={}, distanceInCM = {}, target_angle = {})".format(speed, distance, angle))

def findPath(start, end, reader, speed, actions):
    if (start == end):
        return True

    robotLine = Line(start, end)
    intersectionPoint, mission = intersectLineWithAllMissions(reader, robotLine)
    if (intersectionPoint == None):
        # Call Rishabh's code and return.
        robot = robot(start.getX(), start.getY())
        robot.goto(end.getX(), end.getY(), endAngle=0, speed=speed)
        robot.addActions(speed=speed, robotActions=actions)
        return True
    else:
        angle = findAngleToTurnAfterIntersection(mission, robotLine, intersectionPoint)
        distance = findDistanceToTravelAfterIntersection(mission, robotLine, intersectionPoint)
        startPoint = calculateNewStartPointAfterInstersection(intersectionPoint, distance, angle)
        addActions(speed, angle, distance, actions)
        if (findPath(startPoint, end, reader, speed, actions) == True):
            return True

start = Point(0,0)
end = Point(100,100)
reader = readMissionFile()
actions = []
findPath(start, end, reader, actions)
print(actions)


