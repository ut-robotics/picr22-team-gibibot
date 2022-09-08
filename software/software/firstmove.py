import motion
import time

motion.open()
motion.sendinfo(20, 20, 20, 0, 0)
motion.getinfo()
time.sleep(2)
motion.sendinfo(-20, -20, -20, 0, 0)
motion.getinfo()
time.sleep(2)
motion.sendinfo(0, 0, 0, 0, 0)
motion.close()