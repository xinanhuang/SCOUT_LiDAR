##!/usr/bin/env python

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np

import serial

com_port = "/dev/cu.usbmodem38A2377D30391" # This is on mac os x. For Windows/Linux: 5 == "COM6" == "/dev/tty5"
baudrate = 1440000








def main():
    f = open("demofile5.xyz", "a")
    raw = serial.Serial(com_port, baudrate)

    ## read until raw recieve an end string
    while True:
        line = raw.readline()
        f.write(line.decode('utf-8'))



if __name__ == "__main__":

    main()
