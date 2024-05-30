from tkinter import *
import numpy as np
import math as mt
import time
import winsound
import socket

def connectVehicle(event):
    ip = get_ip_address()+":14553"
    ipAddr_var.set(ip)
    
def get_ip_address():
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    return ip_address

##### Main Program #####

gui = Tk()

Label(gui, text="IP Address").grid(row=0)
Label(gui, text="Desired Latitude").grid(row=1)
Label(gui, text="Desired Longitude").grid(row=2)
Label(gui, text="Desired Altitude").grid(row=3)

ipAddr_var = StringVar()
lat_var = DoubleVar()
long_var = DoubleVar()
alt_var = DoubleVar()

Label(gui, textvariable = ipAddr_var).grid(row=0, column=1)
Label(gui, textvariable = lat_var).grid(row=1, column=1)
Label(gui, textvariable = long_var).grid(row=2, column=1)
Label(gui, textvariable = alt_var).grid(row=3, column=1)

#Connect Button
connectButton = Button(gui, text="Connect")
connectButton.grid(row=4, column=0)
connectButton.bind('<ButtonPress-1>',connectVehicle)

gui.mainloop()