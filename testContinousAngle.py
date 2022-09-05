

class Gyro:
    def __init__(self, gyroReadings):
        self.gyroReadings = gyroReadings
        self.index = 0

    def getReading(self):
        value = self.gyroReadings[self.index]
        self.index = self.index + 1
        return int(value)

class ContinousAngle:

    def __init__(self,startAngle,direction, gyro) :
        self.gyro = gyro
        self.startAngle = startAngle
        self.direction = direction
        self.currentAngle = startAngle
          
        
        self.zeroCrossing = False

    def getAngle(self):
        previousAngle = self.currentAngle
        newAngle = self.gyro.getReading()
        print("previousAngle: " + str(previousAngle) + " newAngle:" + str(newAngle) + " direction:" + self.direction + " startAngle:" + str(self.startAngle))

        # Check if we are in zero crossing stage
        if(self.direction == "Left" and ((previousAngle >= 0 and previousAngle < self.startAngle) and (newAngle > 350 and newAngle < 360) or (self.startAngle == 0))):
            print("Setting zero Crossing for left")
            self.zeroCrossing = True
        if(self.direction == "Right" and (previousAngle < 360 and previousAngle > self.startAngle) and (newAngle >= 0 and newAngle < 10) and self.startAngle != 0):
            print("Setting zero Crossing for right")
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

def readAndExecuteOneTest(file, testCaseName, direction, startAngle, gyroReadings, expectedReadings, emptyLine):
    print("Running test case: " + testCaseName, end =" ")
    
    gyro = Gyro(gyroReadings.split(','))
    testAngle = ContinousAngle(int(startAngle), direction, gyro)
    expectedReadingAry = expectedReadings.split(',')
    readingIndex = 0
    for expectedReading in expectedReadingAry:
        readingIndex = readingIndex +1
        continousReading = testAngle.getAngle()
        if (int(expectedReading) != continousReading):
            print("FAILED")
            print("expectedReading:" + expectedReading + " continousReading:" + str(continousReading) + " at index:" + str(readingIndex))
            return False
    
    print("SUCESS")
    return True

def runTest(testCaseToRun):
    f = open("C:\\Users\\rishabh\\Documents\\FLL\\SuperPowered\\testData.txt", "r")
    testCaseIndex = 1
    while(True):
        testCaseName = f.readline().strip()
        if not testCaseName:
            break
        
        direction = f.readline().strip()
        startAngle = int(f.readline().strip())
        gyroReadings = f.readline().strip()
        expectedReadings = f.readline().strip()
        emptyLine = f.readline().strip()

        if (testCaseToRun == -1 or testCaseToRun == testCaseIndex):
            readAndExecuteOneTest(f, testCaseName, direction, startAngle, gyroReadings, expectedReadings, emptyLine)
        testCaseIndex = testCaseIndex + 1
            
    f.close()

runTest(11)
