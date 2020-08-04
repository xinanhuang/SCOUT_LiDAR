##!/usr/bin/env python

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np

import serial

com_port = "/dev/cu.usbmodem38A2377D30391" # This is on mac os x. For Windows/Linux: 5 == "COM6" == "/dev/tty5"
baudrate = 1440000








def main():

	x = 0
	y = 0
	z = 0

	x_h0 = 0
	x_h1 = 0
	x_h2 = 0
	x_h3 = 0

	y_h0 = 0
	y_h1 = 0
	y_h2 = 0
	y_h3 = 0

	z_h0 = 0
	z_h1 = 0
	z_h2 = 0
	z_h3 = 0

	end_flag = False;

	f = open("demofile10.xyz", "a")
	raw = serial.Serial(com_port, baudrate)

	## read until raw recieve an end string
	while end_flag is False:

		line = raw.read(12)

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

			elif (line[idx] & 0xF0) == 0xC0: 
				end_flag = True
				print('end')


		if end_flag is True:
			break

		x = np.int16((x_h0 <<12 & 0xF000) | (x_h1 << 8 & 0xF00) |(x_h2 <<4 & 0xF0) | (x_h3 & 0xF))
		y = np.int16((y_h0 <<12 & 0xF000) | (y_h1 << 8 & 0xF00) |(y_h2 <<4 & 0xF0) | (y_h3 & 0xF))
		z = np.int16((z_h0 <<12 & 0xF000) | (z_h1 << 8 & 0xF00) |(z_h2 <<4 & 0xF0) | (z_h3 & 0xF))

		f.write(str(x)+' '+str(y)+' '+ str(z)+ '\n')

	raw.close()

if __name__ == "__main__":

    main()
