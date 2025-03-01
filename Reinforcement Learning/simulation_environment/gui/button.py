import tkinter
from tkinter import messagebox as tkMessageBox
import os

top = tkinter.Tk()
top.geometry("200x100")

def callback():
   os.system("ign service -s /world/car_world/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 300 --req \'name: \"Start_Box\" type: MODEL\'")
   top.destroy()
   
def callback1():
   os.system("ros2 run ros_ign_gazebo create -world 'car_world' -file ~/bridge_ws/src/simulation/start_box/start_box.sdf -allow_renaming true -Y 3.14 -z 0.09 -x 1.0 -y 0.4")
  
remove = tkinter.Button(top, text = "Start", command = callback)
appear = tkinter.Button(top, text = "Show Box", command = callback1)

appear.pack()
remove.pack()
top.mainloop()
