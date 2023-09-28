#!/usr/bin/python3

# XILIBOT PC CONTROL python script
# Exampe 3: Graphic User Interface to control XILIBOT (using Tkinter python UI)
# Before running the script you need to connect the PC to the XILIBOT wifi
# Remember, default password for Wifi network JJROBOTS_XX is 87654321

# author: JJROBOTS 2016
# version: 1.01 (28/10/2016)
# Licence: Open Source (GNU LGPLv3)

import os
import socket
import time
import struct
import tkinter as tk
from tkinter import IntVar
from XILIBOT_Class import XILIBOT # Import CLASS to control XILIBOT


# GRAPHIC USER INTERFACE
class MainApplication(tk.Frame):
  throttle = 0
  steering = 0
  mode = 0
  myRobot = XILIBOT()
  def draw(self):
    print("DRAW")
  def __init__(self,parent,*args,**kwargs):
    tk.Frame.__init__(self,parent,*args,**kwargs)
    self.parent = parent
    # GUI INIT: 2 sliders and 1 button
    self.t = tk.Scale(troughcolor="light yellow",from_=100,to=-100,width=40,length=200)
    self.t.pack(side=tk.LEFT,fill=tk.BOTH,expand=1)
    self.s = tk.Scale(troughcolor="light cyan",from_=-100,to=100,width=40,length=250,orient=tk.HORIZONTAL)
    self.s.pack(side=tk.LEFT,fill=tk.X)
    self.after(50,self.timer)
    self.b = tk.Button(bg="light green",repeatdelay=1,repeatinterval=50,width=10,height=2,text="SERVO",command=self.buttonCallback)
    self.b.pack(side=tk.LEFT)
    self.mode_control = IntVar()
    self.m = tk.Checkbutton(text="PRO MODE",variable=self.mode_control)
    self.m.pack()

    # Handle cursor keys
    self.parent.bind("<KeyPress>", self.key_pressed)
    self.parent.bind("<KeyRelease>", self.key_released)

  def key_pressed(self, event):
    if event.keysym == 'Up':
      t = self.throttle + 100
      if t > 100:
        t = 100 
      self.t.set(t)
    elif event.keysym == 'Down':
      t = self.throttle - 100
      if t < -100:
        t = -100 
      self.t.set(t)
    elif event.keysym == 'Right':
      s = self.steering + 50
      if s > 100:
        s = 100 
      self.s.set(s)
    elif event.keysym == 'Left':
      s = self.steering - 50
      if s < -100:
        s = -100 
      self.s.set(s)
    #print("Pressed", vars(event))
  def buttonCallback(self):
    self.myRobot.servo(1)
    time.sleep(0.2)
    self.myRobot.servo(0)    
  def timer(self): # Timer at 50ms interval to read slider values
    if (self.t.get()!=self.throttle):
      self.throttle = self.t.get()
      self.myRobot.throttle(self.throttle/100.0)
      #print "THROTTLE:",self.throttle
    if (self.s.get()!=self.steering):
      self.steering = self.s.get()
      self.myRobot.steering(self.steering/100.0)
      #print "STEERING:",self.steering    
    if (self.mode_control.get()!=self.mode):
      self.mode = self.mode_control.get()
      self.myRobot.mode(self.mode)
      #print "MODE:",self.mode
    if abs(self.throttle) > 0.001:
      self.throttle *= 0.25
      self.t.set(round(self.throttle))
    if abs(self.steering) > 0.001:
      self.steering *= 0.25
      self.s.set(round(self.steering))
    self.after(50,self.timer)
    

# START APPLICATION
if __name__=="__main__":
    root = tk.Tk()
    root.wm_title("XILIBOT CONTROL")
    MainApplication(root).pack(side="top",fill="both",expand=True)
    root.mainloop()


