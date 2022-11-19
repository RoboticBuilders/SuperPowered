import csv

class Point(object):
    def __init__(self, x, y):
        '''Defines x and y variables'''
        self.X = x
        self.Y = y

    def getX(self):
        return self.X

    def getY(self):
        return self.Y

class Line(object):
    def __init__(self, start, end):
        '''Defines start and end points'''
        self.start = start
        self.end = end

    def isPointOnLine(point):
        '''Returns true if the point is on the line and between start and end points'''
    
    def doesIntersect(anotherLine):
        ''' TODO: Rishabh write this code.
        '''

def readMissionFile():
    with open("C:\\Users\\rishabh\Documents\\FLL\\SuperPowered\\mission_coordinates.txt", newline='\n') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in spamreader:
            print(', '.join(row))


readMissionFile()