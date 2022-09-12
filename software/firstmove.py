import motion
import time
import serial

robot=motion.OmniRobotMotion()



robot.open()
robot.sendinfo(0, 0, 0, 0, 0)
robot.getinfo()
time.sleep(1)
robot.sendinfo(0, 0, 0, 0, 0)
robot.getinfo()
time.sleep(1)
robot.sendinfo(0, 0, 0, 0, 0)
robot.close()