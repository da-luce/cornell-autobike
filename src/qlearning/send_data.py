from machine import I2C
import settings

def init():
     #dummy
    settings.angle -= 1



    i2c = I2C(freq=400000)          # create I2C peripheral at frequency of 400kHz
	                                # depending on the port, extra parameters may be required
	                                # to select the peripheral and/or pins to use

    print(i2c.scan())               #check that the peripherals are connected

	#send floating point value to controls for the angle
    i2c.writeto(42, settings.angle)         # write 3 bytes to peripheral with 7-bit address 42
