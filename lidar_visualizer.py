##!/usr/bin/env python

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np

import serial

com_port = "/dev/cu.usbmodem38A2377D30391" # This is on mac os x. For Windows/Linux: 5 == "COM6" == "/dev/tty5"
baudrate = 1440000






class MyWidget(pg.GraphicsWindow):

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)

        self.timer.start()
        self.timer.timeout.connect(self.onNewData)
        self.raw = serial.Serial(com_port, baudrate)
        self.plotItem = self.addPlot(title="Lidar points")
        self.plotItem.enableAutoRange(pg.ViewBox.XYAxes,False)
        self.plotItem.setXRange(-1000,1000)
        self.plotItem.setYRange(-1000,1000)
        self.plotItem.setAspectLocked(True,1)
        self.plotItem.showGrid(x=True,y=True)
        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255,0,0), symbolSize=5, symbolPen=None,)

        self.f = open("demofile2.xyz", "a")


    def setData(self, x, y):
        self.plotDataItem.setData(x, y)


    def onNewData(self):
        ## read lidar data and update 
        ## read 1000 data and draw the loop
        ## can be further optimized by parse the data myself (duh i mean using python byitself is the defiantion of not being optimized)
        x = np.empty(7400)
        y = np.empty(7400)
        for idx in range (7400):

            line = self.raw.readline()
            line_split = line.decode().split(' ')
            x[idx] = int(line_split[0])
            y[idx] = int(line_split[1])
            z = int(line_split[2])
    
            ##self.f.write(str(line))
       


        
        ## parse input with space and carriage return 

        self.setData(x, y)


def main():

    app = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=False) # True seems to work as well

    win = MyWidget()
    win.show()
    win.resize(800,600) 
    win.raise_()
    app.exec_()

if __name__ == "__main__":

    main()