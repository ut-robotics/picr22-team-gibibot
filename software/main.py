import image_processor
import camera
import cv2
import time
from math import *
import numpy as np
import movement
import calculations
import communication

def Orbit(radius, xcord, reso_x_mid,dist):
    speedY = 0
    speedX = 0.5
    speedR = 1000*speedX/(640-radius)

    if radius > (dist-5):
        speedY += (radius-dist)/100
    elif radius > (dist+5):
        speedY += (radius-dist)/100
    
    if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
        speedR += (reso_x_mid- xcord) / 100

    return [speedX, speedR, speedY]


def main_loop():
    state="find_ball"
    spin = 10
    debug=True
    basket_color="magenta"
    basket_height = 0.5
    cam=camera.RealsenseCamera(exposure=100) #defaulti peal on depth_enabled = True
    processor = image_processor.ImageProcessor(cam, debug=debug, color_config = "colors/colors.pkl")
    robot=movement.OmniRobot()
    reso_x_mid=424
    calculator = calculations.Calculations()
    comms=communication.Communication()
    processor.start()
    start=time.time()
    fps=0
    frame=0
    frame_cnt=0
    radius=250
    ball_side=0
    calibration_speed=0.15
    
    try:
        while True:
            if state == "throw":
                processedData = processor.process_frame(aligned_depth=True)
            else:
                processedData = processor.process_frame(aligned_depth=False)

            frame_cnt+=1
            frame+=1
            if state == "try":
                robot.try_motors()
                break
            if state == "tmotor":
                robot.test_thrower(1400)
                break
            if state=="cameratest":
                processedData = processor.process_frame(aligned_depth=True)
                processedData.balls.sort(key= lambda x: x.size)
                targeted_ball=processedData.balls[-1]
                xcord=targeted_ball.x
                ycord=targeted_ball.y
                dist=targeted_ball.distance
                print("DEPTH_FRAME: ", processedData.depth_frame)
                continue
            
            if frame%30 == 0:
                frame = 0
                end = time.time()
                fps = 30/(end-start)
                start = end
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]  
                print(fps)  

            if state == "find_ball":
                print("--------Searching ball--------")
                if len(processedData.balls)!=0:
                    state="move"
                else:
                    if ball_side==1:
                        robot.find_ball(spin)
                    elif ball_side==0:
                        robot.find_ball((-1)*spin)
                        
            elif state == "move":
                print("--------Moving to ball--------")
                if (len(processedData.balls)!=0):
                    
                    processedData.balls.sort(key= lambda x: x.size)
                    targeted_ball=processedData.balls[-1]
                    xcord=targeted_ball.x
                    ycord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)
                    print(dist)
                        #Follows where is ball so find_ball function is more efficient
                    if delta<0:
                        ball_side = 0
                    else:
                        ball_side = 1
                    
                        #SpeedY based on ball distance
                    if dist > 175:
                        speedY=0.1
                    else:
                        speedY=0.5
                        #Controlls that balls location is ready for robot's orbit function
                    if xcord < (reso_x_mid + 15) and xcord >(reso_x_mid -15) and dist > 250:
                        #robot.center_ball(xcord)
                        state="orbit"
                    else:
                        #Moves to ball
                        speedR = delta*(-1.75)
                        speedX = delta*0.25
                        robot.move(speedX, speedR, speedY)
                else:
                    #If there is no ball
                    state="find_ball"
                        
            elif state=="orbit":
                print("--------Centering ball and basket--------")
                
                
                #If there is ball
                if len(processedData.balls)>0:
                    processedData.balls.sort(key= lambda x: x.size)
                    targeted_ball=processedData.balls[-1]
                    xcord=targeted_ball.x
                    ycord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)

                    if (xcord > 434):
                        rotate_dir = -2
                    elif (xcord < 414):
                        rotate_dir = 2
                    else:
                        rotate_dir = 0

                    robot.move(rotate_dir, rotate_dir, rotate_dir)
                    #controlls basket colour

                    if basket_color == "magenta":
                        basket = processedData.basket_m
                    elif basket_color == "blue":
                        basket = processedData.basket_b

                    #if that kind of basket is in our list
                    if basket.exists:
                        #deltabasket=((basket.x-reso_x_mid)/reso_x_mid)
                        if(abs(basket.x-reso_x_mid)<10):
                            robot.stop()
                            state="throw"
                            print("Robot is ready to throw")
                            #state="find_ball"
                            
                        elif abs(basket.x-reso_x_mid)>30:
                            #orbit_speeds = Orbit(radius, reso_x_mid, xcord, dist)
                             
                            speedY=0
                            speedX=0.5
                            speedR=1000*speedX/(640-radius)

                            if radius > (dist-5):
                                speedY += (radius-dist)/100
                            elif radius > (dist+5):
                                speedY += (radius-dist)/100
                            
                            
                            if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                                speedR += (reso_x_mid- xcord) / 100 
                                                   
                            #robot.move(orbit_speeds[0],orbit_speeds[1],orbit_speeds[2])
                            robot.move(speedX, speedR, speedY)                        
                    else:
                        #Orbit
                        #orbit_speeds = Orbit(radius, reso_x_mid, xcord, dist)

                        speedY=0
                        speedX=0.5
                        speedR=1000*speedX/(640-radius)

                        if radius > (dist-5):
                            speedY += (radius-dist)/100
                        elif radius > (dist+5):
                            speedY += (radius-dist)/100
                        
                        
                        if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                            speedR += (reso_x_mid- xcord) / 100 
                                                
                        #robot.move(orbit_speeds[0],orbit_speeds[1],orbit_speeds[2])
                        robot.move(speedX, speedR, speedY)
                        
                        #robot.move(orbit_speeds[0],orbit_speeds[1],orbit_speeds[2])
            
                            
                    

            elif state == "throw":
                #calc speed in m/s

                processedData.balls.sort(key= lambda x: x.size)
                targeted_ball=processedData.balls[-1]
                dist=targeted_ball.distance
                delta=((xcord-reso_x_mid)/reso_x_mid)
                

                if dist > 300:
                    basket_distance = 2
                    launch_speed = 2*basket_distance / sqrt((2*basket_distance*sqrt(3) - 2*basket_height)/9.81)
                    
                    
                    #convert to rpm
                    ang_speed=(60/(2*pi*0.015))*launch_speed
                    speedT = ang_speed
                #Moves to ball
                speedY = 0.3
                if dist > 400:
                    speedR = 0
                    speedX = 0
                    speedT=950
                    robot.throw(speedX, speedR, speedY, speedT)
                    time.sleep(3)
                    state = "find_ball"
                    
                else:
                    speedR = delta*(-1.75)
                    speedX = delta*0.25
                    robot.move(speedX, speedR, speedY)

                
                        
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
        comms.close()


    
main_loop()
