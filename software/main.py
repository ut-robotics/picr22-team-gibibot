from ast import Break
from asyncio import constants
from concurrent.futures import process
from pickle import TRUE
import image_processor
import camera
#import motion
import cv2
import time
from math import *
import numpy as np
import movement

spin = 10





def main_loop():
    debug=True
    
    cam=camera.RealsenseCamera(exposure=100)
    processor = image_processor.ImageProcessor(cam, debug=debug, color_config = "colors/colors.pkl")
    global robot
    robot=movement.OmniRobot()
    
    processor.start()
    start=time.time()
    fps=0
    frame=0
    frame_cnt=0
    
    try:
        while True:
            processedData= processor.process_frame(aligned_depth=False)
            frame_cnt+=1
            frame+=1
            #robot.try_motors()
            #break
            if frame%30==0:
                frame = 0
                end = time.time()
                fps = 30/(end-start)
                start = end
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]  
                print(processedData.balls)  
        
                if (len(processedData.balls)!=0):
                    processedData.balls.sort(key= lambda x: x.size)
                    xcord=processedData.balls[-1].x
                    ycord=processedData.balls[-1].y
                    dist=processedData.balls[-1].distance

                    if dist > 400:
                        robot.center_ball(xcord)
                    else:
                        #robot.center_ball(spin, xcord)
                        wheel1=robot.omni(xcord,1,dist)
                        wheel2=robot.omni(xcord,2,dist)
                        wheel3=robot.omni(xcord,3,dist)

                    #print("Wheel speeds-wheel1: ", wheel1, "wheel2:", wheel2, "wheel3:", wheel3)
                    
                        robot.send_inf(wheel1,wheel2,wheel3, 0, 1)

                else:
                    print("find ball")
                    robot.find_ball(spin, len(processedData.balls))
                        
                    
            if debug:
                    debug_frame = processedData.debug_frame

                    cv2.imshow('debug', debug_frame)

                    k = cv2.waitKey(1) & 0xff
                    if k == ord('q'):
                        robot.stop()
                        break
        
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot.close()
        
#def Drive1m():
#    start_time = time.time()
#    while True:
#        robot.send_inf(-20,0,20,0,1)
#        if robot.timer(3,start_time):
#            robot.stop()
#            break
    
main_loop()
