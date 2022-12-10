import serial
import struct
import serial.tools.list_ports



class Communication:
    def __init__(self):
        self.ports=serial.tools.list_ports.comports()
        for port in self.ports:
            try:
                self.port="/dev/"+port.name
                self.Serial=serial.Serial(self.port, baudrate=9600, timeout=2)
                self.Serial.write(struct.pack('<hhhHHHH', 0,0,0,0,4800,4800,0xAAAA))
                self.RecvData=self.Serial.read(8)
                self.ball_in_grabber = 0
                self.Data=struct.unpack('<hhhH', self.RecvData)
                #self.testSerial.close()
                #self.port=self.testPort
            except:
                continue
            #finally:
                #self.Serial=serial.Serial(self.port, baudrate=9600, timeout=2)
            
        print("Serial opened on port: ", self.port)

    def send_inf(self, speed1, speedr, speed3, thrower_speed, thrower_angle, grabber):
        try:
            inf_out=struct.pack('<hhhHHHH', speed1, speedr, speed3, thrower_speed, thrower_angle, grabber, 0xAAAA)
            self.Serial.write(inf_out)
            print("Data sent: ", speed1, speedr, speed3, thrower_speed, thrower_angle, grabber, 0xAAAA)
            inf_in=self.Serial.read(8)
            ball_detected, actual_speedr, actual_speed3,feedback_delimiter = struct.unpack('<hhhH', inf_in)
            self.ball_in_grabber = ball_detected
        except Exception as e:
            print(e)

    def close(self):
        self.Serial.close()
        print("Serial closed")
        
    def communication_test(self):
        self.send_inf(0,0,0,0,4800,4800)

