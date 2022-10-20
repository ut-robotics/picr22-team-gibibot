import image_processor
import camera
import cv2
import time
from math import *
import numpy as np
import movement
import calculations
import communication


def main_loop():
    state="tmotor"
    spin = 10
    debug=True
    basket_color="magenta"
    cam=camera.RealsenseCamera(exposure=100)
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
    
    try:
        while True:
            processedData= processor.process_frame(aligned_depth=False)
            frame_cnt+=1
            frame+=1
            if state=="try":
                robot.try_motors()
                break
            if state=="tmotor":
                comms.send_inf(0,0,0,300,1)
                time.sleep(1)
                comms.send_inf(0,0,0,0,1)
                break
            
            if frame%1==0:
                frame = 0
                end = time.time()
                fps = 30/(end-start)
                start = end
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]  
                print(processedData.balls)  

                if state=="find_ball":
                    print("--------Searching ball--------")
                    if len(processedData.balls)!=0:
                        state="move"
                    else:
                        if ball_side==1:
                            robot.find_ball(spin)
                        elif ball_side==0:
                            robot.find_ball((-1)*spin)
                            
                elif state=="move":
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
                            ball_side=0
                        else:
                            ball_side=1
                        
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
                            wheel1=calculator.calc_speed(speedX, speedY, speedR, 1)
                            wheelr=calculator.calc_speed(speedX, speedY, speedR, 2)
                            wheel3=calculator.calc_speed(speedX, speedY, speedR, 3)
                            comms.send_inf(wheel1,wheelr,wheel3, 0, 1)
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
                        robot.center_ball(xcord)
                        #controlls basket colour

                        if basket_color == "magenta":
                            basket = processedData.basket_m
                        elif basket_color == "blue":
                            basket = processedData.basket_b
                        #if that kind of basket is in our list
                        
                        if basket.exists:
                            deltabasket=((basket.x-reso_x_mid)/reso_x_mid)
                            print(basket.x, basket.y, basket.distance)
                            print("basketx-resomid: ", basket.x-reso_x_mid)
                            if(abs(basket.x-reso_x_mid)<10):
                                robot.stop() 
                                #input("Robot is ready to throw")
                                #state="find_ball"
                                
                            elif abs(basket.x-reso_x_mid)>30:
                                speedY=0
                                speedX=0.5
                                speedR=1000*speedX/(640-radius)

                                if radius > (dist-5):
                                    speedY += (radius-dist)/100
                                elif radius > (dist+5):
                                    speedY += (radius-dist)/100
                                
                                
                                if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                                    speedR += (reso_x_mid- xcord) / 100
                                
                                wheely=calculator.calc_speed(speedX, speedY, speedR, 1)
                                wheelr=calculator.calc_speed(speedX, speedY, speedR, 2)
                                wheelx=calculator.calc_speed(speedX, speedY, speedR, 3)
                                
                                robot.orbit(wheely,wheelr,wheelx)                        
                        else:
                            #Orbit
                            print("ORBITING RIGHT NOW!!!!")

                            speedY=0
                            speedX=0.5
                            speedR=1000*speedX/(640-radius)
                            if radius > (dist-5):
                                speedY += (radius-dist)/100
                            elif radius > (dist+5):
                                speedY += (radius-dist)/100
                            if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                                speedR += (reso_x_mid - xcord) / 100
                            
                            print("SPEED R JA X: ", speedR, " ", speedX)
                            wheely=calculator.calc_speed(speedX, speedY, speedR, 1)
                            wheelr=calculator.calc_speed(speedX, speedY, speedR, 2)
                            wheelx=calculator.calc_speed(speedX, speedY, speedR, 3)
                            print("WHEELS: 1.", wheely, " 2.", wheelr, " 3.", wheelx)
                            robot.orbit(wheely,wheelr,wheelx)
                    else:
                        state="find_ball"
                        
                    
                    
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
