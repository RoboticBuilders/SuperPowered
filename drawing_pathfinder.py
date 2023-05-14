from turtle import Turtle, Screen
from obstacle_avoidance import *
import obstacle_avoidance
from time import *
import time

turtle = Turtle()
screen = Screen()
screen.setup(780,460, 0, 0)
screen.setworldcoordinates(0,0,200,200)
screen.mode('world')
#screensize(1500,1500)
screen.bgpic(".\\superpowered_darker_ewireframe.GIF")
canvas = screen.getcanvas()
canvas.itemconfig(screen._bgpic, anchor="sw")
screen.update()

"""
# Add code now to use the output of obstacle avoidance.
counter = 0
while True:
    if counter % 2 == 0:
        turtle.pen(fillcolor="black", pencolor="red", pensize=5)
    else:
        turtle.pen(fillcolor="black", pencolor="blue", pensize=5)
    turtle.setposition(30,10)
    turtle.goto(86, 13.5)
    turtle.goto(86, 43.5)
    turtle.goto(190, 20)
    time.sleep(5000)
    turtle.clear()
    counter = counter + 1
    screen.update()
"""

#repeatlyShowThePaths()

# while(True):
#         findAndShowAllPaths()
#         sleep(1)
#         screen.reset()
#         turtle.pen(fillcolor="black", pencolor="blue", pensize=5)

obstacle_avoidance.repeatlyShowThePaths()

screen.mainloop()