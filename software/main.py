import image_processor
import camera
import motion
import cv2
import time
from math import *
import numpy as np

def main_loop():
    debug = True
    
    motion_sim = motion.TurtleRobot()
    motion_sim2 = motion.TurtleOmniRobot()
    robot = motion.OmniRobotMotion()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug, color_config = "colors/colors.pkl")

    processor.start()
    motion_sim.open()
    motion_sim2.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]
                if len(processedData.balls)!=0:
                    processedData.balls.sort(key= lambda x: x.size)
                    print(processedData.balls[-1])
                    xcord=processedData.balls[-1].x
                    
                    ycord=processedData.balls[-1].y
                    print("X: " , xcord , "Y: " , ycord)
                #848 laius
                speedx, speedy = 20,100
                speedr=0.003*(xcord-424)
                robotspeed=sqrt(speedx * speedx + speedy * speedy)
                robotdirangle = atan2(speedy, speedx)
                ratas1=robotspeed * cos(robotdirangle - 120) + 0.11 * speedr
                ratas2=robotspeed * cos(robotdirangle - 0) + 0.11 * speedr
                ratas3=robotspeed * cos(robotdirangle - 240) + 0.11 * speedr

                motion_sim.move(0, speedy, speedr)
                
                #speed1 - x, speed2 - y, speed3 - rotation
                """
                if xcord<930:
                    robot.sendinfo(0, 0, 15, 0, 0)
                    robot.getinfo()
                elif ycord>990:
                    robot.sendinfo(0, 0, -15, 0, 0)
                    robot.getinfo()
                else:
                    print("Pall on keskel")
                    robot.sendinfo(20,20,0,0,0)
                    robot.getinfo()"""
                
                
                    
                
                #if (frame_cnt > 1000):
                #    break

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()

main_loop()
