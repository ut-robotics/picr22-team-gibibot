from curses import baudrate
import serial
import struct

ser=serial.Serial('/dev/ttyUSB1', baudrate=9600)
speed1=0
speed2=0
speed3=0
ser.open()

data=struct.pack('<hhhHBH', speed1, speed2, speed3, 0, 0, 0xAAAA)
incinfo = ser.read()
actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', incinfo)

print("Incoming data: " + actual_speed1, actual_speed2, actual_speed3, feedback_delimiter)
ser.close()
