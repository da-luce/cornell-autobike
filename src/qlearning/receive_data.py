import settings
from machine import I2C

def init():
        #dummy
	settings.gps += 1

	i2c = I2C(freq=400000)          # create I2C peripheral at frequency of 400kHz
	                                # depending on the port, extra parameters may be required
	                                # to select the peripheral and/or pins to use
	print(i2c.scan())               #check that the peripherals are connected

	settings.gps = i2c.readfrom(42, 4)             # read 4 bytes from peripheral with 7-bit address 42

	#an idea: write to a file and have the other file read the data from there
	#https://stackoverflow.com/questions/13034496/using-global-variables-between-files"""

