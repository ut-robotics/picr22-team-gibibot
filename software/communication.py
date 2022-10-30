import serial
import struct
import serial.tools.list_ports



class Communication:
    def __init__(self):
        self.ports=serial.tools.list_ports.comports()
        for port in self.ports:
            try:
                self.testPort="/dev/"+port.name
                self.testSerial=serial.Serial(self.testport, baudrate=9600, timeout=2)
                self.testSerial.write(struct.pack('<hhhHBH', 0,0,0,0,0,0xAAAA))
                self.testRecvData=self.testSerial.read(8)
                self.testData=struct.unpack('<hhhH', self.testRecvData)
                self.testSerial.close()
                self.port=self.testPort
            except:
                continue
            finally:
                self.Serial=serial.Serial(self.port, baudrate=9600, timeout=2)
            
        print("Serial opened on port: ", self.port)

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

