##!/usr/bin/env python


## python is stupid as  we need numpy to define 16 bit
## the additiaonl perforamnce might not be worth it 

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


        self.x_h0 = 0;
        self.x_h1 = 0;
        self.x_h2 = 0;
        self.x_h3 = 0;


        self.y_h0 = 0;
        self.y_h1 = 0;
        self.y_h2 = 0;
        self.y_h3 = 0;

        self.z_h0 = 0;
        self.z_h1 = 0;
        self.z_h2 = 0;
        self.z_h3 = 0;

        self.f = open("demofile9.xyz", "a")


    def setData(self, x, y):
        self.plotDataItem.setData(x, y)


    def onNewData(self):
        ## read lidar data and update 
        ## read 1000 data and draw the loop
        ## can be further optimized by parse the data myself (duh i mean using python byitself is the defiantion of not being optimized)
        x = np.empty(7400)
        y = np.empty(7400)

        for idx in range (7400):

            line = self.raw.read(12)


            x_num = 0;
            y_num = 0;
           
            for idx in range (12):
                if (line[idx] & 0xF0) == 0x00:
                    x_h0 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x10:
                    x_h1 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x20:
                    x_h2 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x30:
                    x_h3 = line[idx] & 0xF

                elif (line[idx] & 0xF0) == 0x40:
                    y_h0 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x50:
                    y_h1 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x60:
                    y_h2 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x70:
                    y_h3 = line[idx] & 0xF

                elif (line[idx] & 0xF0) == 0x80:
                    z_h0 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0x90:
                    z_h1 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0xA0:
                    z_h2 = line[idx] & 0xF
                elif (line[idx] & 0xF0) == 0xB0:
                    z_h3 = line[idx] & 0xF
            

            x[idx] = np.int16((x_h0 <<12 & 0xF000) | (x_h1 << 8 & 0xF00) |(x_h2 <<4 & 0xF0) | (x_h3 & 0xF))
            y[idx] = np.int16((y_h0 <<12 & 0xF000) | (y_h1 << 8 & 0xF00) |(y_h2 <<4 & 0xF0) | (y_h3 & 0xF))
            z = np.int16((z_h0 <<12 & 0xF000) | (z_h1 << 8 & 0xF00) |(z_h2 <<4 & 0xF0) | (z_h3 & 0xF))
   
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