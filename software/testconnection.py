from curses import baudrate
import serial
import struct



ser=serial.Serial('/dev/ttyACM1', baudrate=9600)
ser.close()
speed1=10
speed2=10
speed3=10
ser.open()

data=struct.pack('<hhhHBH', speed1, speed2, speed3, 0, 0, 0xAAAA)
ser.write(data)
ser.close()
