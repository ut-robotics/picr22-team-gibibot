from ast import Break
from asyncio import constants
import image_processor
import camera
#import motion
import cv2
import time
from math import *
import numpy as np
import movement

spin = 10

speedY=0.5



def main_loop():
    debug=True
    
    cam=camera.RealsenseCamera(exposure=100)
    processor = image_processor.ImageProcessor(cam, debug=debug, color_config = "colors/colors.pkl")
    global robot
    robot=movement.OmniRobot()
    
    processor.start()
    #robot.try_motors()
    start=time.time()
    fps=0
    frame=0
    frame_cnt=0
    #Drive1m()
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
                #print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                #print("ball_count: {}".format(len(processedData.balls)))
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]
                if (len(processedData.balls)==0):
                    robot.find_ball(spin)

                else:
                    processedData.balls.sort(key= lambda x: x.size)
                    print(processedData.balls[-1])
                    xcord=processedData.balls[-1].x
                    ycord=processedData.balls[-1].y
                    print("X: " , xcord , "Y: " , ycord)
                    #Framei laius 848
                    

                    #robot.center_ball(spin, xcord)
                    wheel1=robot.omni(speedY,xcord,1)
                    wheel2=robot.omni(speedY,xcord,2)
                    wheel3=robot.omni(speedY,xcord,3)

                    #print("Wheel speeds-wheel1: ", wheel1, "wheel2:", wheel2, "wheel3:", wheel3)
                    
                    robot.send_inf(wheel1,wheel2,wheel3, 0, 1)
                        
                    
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
