from turtle import Turtle, Screen
from obstacle_avoidance import *
from time import *

turtle = Turtle()
screen = Screen()
screen.setup(780,460, 0, 0)
screen.setworldcoordinates(0,0,200,200)
screen.mode('world')
#screensize(1500,1500)
screen.bgpic(".\\superpowered_wireframe.GIF")
canvas = screen.getcanvas()
canvas.itemconfig(screen._bgpic, anchor="sw")
screen.update()

# Add code now to use the output of obstacle avoidance.

turtle.setposition(30,10)
turtle.goto(86,13.5)
turtle.goto(86,43.5)
turtle.goto(190, 20)

screen.mainloop()