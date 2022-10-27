import serial
import struct

#ports=serial.tools.list_ports.comports()
#print([port.name for port in ports])

class Communication:
    def __init__(self):
        self.ser = serial.Serial()
        self.ser.port='/dev/ttyACM0'
        self.ser.baudrate=9600
        self.ser.open()
        print("Serial opened")

        #  self.port='dev/ttyACM0'
        #self.ser = serial.Serial(self.port, baudrate=115200, timeout=2)
        #self.ser.open()
        #print("Serial opened")
        #self.ser.write('<hhhHBH', 0, 0, 0, 0, 0xAAAA)

    def send_inf(self, speed1, speedr, speed3, thrower_speed, disable_failsafe):
        inf_out=struct.pack('<hhhHBH', speed1, speedr, speed3, thrower_speed, disable_failsafe, 0xAAAA)
        self.ser.write(inf_out)
        #print("Data sent")
        inf_in=self.ser.read(8)
        actual_speed1, actual_speedr, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', inf_in)
        print("Real Data: " , actual_speed1, actual_speedr, actual_speed3, feedback_delimiter)
        
    def close(self):
        self.ser.close()
        print("Serial closed")
        
    def communication_test(self):
        self.send_inf(0,0,0,0,0)

