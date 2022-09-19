# LEGO type:standard slot:0 autostart
import os, sys







def RoboticBuildersimportFile(slotid=0):

    
   
    with open("/projects/.slots","rt") as f:
        slots = eval(str(f.read()))
    #print("0: " + str(slots[0]))
    #print("1: " + str(slots[1]))
    #print("2: " + str(slots[2]))

    # Getting the current work directory (cwd)
    #thisdir = os.getcwd()

    # r=root, d=directories, f = files
    #for x in os.listdir("/projects/37016"):
    #    print(x)

    with open("/projects/"+str(slots[slotid]["id"])+"/__init__.py","rb") as f:
        program = f.read()
    #print("program: " + str(program))
    try:
     os.remove("/RoboticBuildersimportFile.py")
    except:
      pass
    with open("/RoboticBuildersimportFile.py","w+") as f:
        f.write(program)
    if ("RoboticBuildersimportFile" in sys.modules):
        del sys.modules["RoboticBuildersimportFile"]
    exec("from RoboticBuildersimportFile import *")

# Import the files
RoboticBuildersimportFile(1)
RoboticBuildersimportFile(2)

#Call the files

Utils()
Run1()