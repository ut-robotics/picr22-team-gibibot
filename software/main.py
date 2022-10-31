import image_processor
import camera
import cv2
import time
from math import *
import numpy as np
import movement
import calculations
import communication
from enum import Enum

class State(Enum):
    FIND_BALL=0
    MOVE=1
    ORBIT=2
    THROW=3
    TRYMOTORS=4
    TESTCAMERA=5
    TMOTOR=6
    
class Color(Enum):
    MAGENTA=0
    BLUE=1




def main_loop():
    state=State.FIND_BALL
    spin = 12
    debug=True
    basket_color=Color.MAGENTA
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
    radius=300
    #Which side ball is on
    ball_side=0
    MaxOrbit_YSpeed=0.2
    MaxOrbit_RSpeed=2
    try:
        while True:
            speedT=0
            if state == State.THROW:
                processedData = processor.process_frame(aligned_depth=True)
            else:
                processedData = processor.process_frame(aligned_depth=False)
            
            frame_cnt+=1
            frame+=1
            #Just Testing states for own comfort
            if state == State.TRYMOTORS:
                
                robot.try_motors()
                break
            if state == State.TMOTOR:
                try:
                    processedData = processor.process_frame(aligned_depth=True)
                    spt=int(input("ANNA TSPEED: "))
                    robot.test_thrower(spt)
                    
                    continue
                except:
                    print("KORVI POLE")
            if state==State.TESTCAMERA:
                try:
                    processedData = processor.process_frame(aligned_depth=True)
                    print("LEGIT DISTANCE: ", processedData.basket_m.distance)
                    #targeted_ball=processedData.balls[-1]
                    #print("Palli y kord   ", targeted_ball.y)
                    #print("DEPTH:",processedData.depth_frame)
                    #print(processedData.basket_m)
                except:
                    continue
                    
                
            
            if frame%30 == 0:
                frame = 0
                end = time.time()
                fps = 30/(end-start)
                start = end
                #[Object: x=7; y=212; size=15.0; distance=212; exists=True]  
                print(fps)  

            if state == State.FIND_BALL:
                print("--------Searching ball--------")
                if len(processedData.balls)!=0:
                    state=State.MOVE
                else:
                    if ball_side==1:
                        robot.find_ball(spin)
                    elif ball_side==0:
                        robot.find_ball((-1)*spin)
                        
            elif state == State.MOVE:
                print("--------Moving to ball--------")
                if (len(processedData.balls)!=0):
                    
                    targeted_ball=processedData.balls[-1]
                    xcord=targeted_ball.x
                    ycord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)
                        #Follows where is ball so find_ball function is more efficient
                    if delta<0:
                        ball_side = 0
                    else:
                        ball_side = 1
                    
                        #SpeedY based on ball distance
                    if dist > 175:
                        speedY=0.4
                    else:
                        speedY=1
                        #Controlls that balls location is ready for robot's orbit function
                    if xcord < (reso_x_mid + 15) and xcord >(reso_x_mid -15) and dist > 340:
                        #robot.center_ball(xcord)
                        state=State.ORBIT
                    else:
                        #Moves to ball
                        speedR = delta*(-3.5)
                        speedX = delta*0.5
                        robot.move(speedX, speedR, speedY, speedT)
                else:
                    #If there is no ball
                    state=State.FIND_BALL
                        
            elif state==State.ORBIT:
                print("--------Centering ball and basket--------")
                #If there is ball
                if len(processedData.balls)>0:
                    targeted_ball=processedData.balls[-1]
                    xcord=targeted_ball.x
                    ycord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)

                    #controlls basket colour

                    if basket_color == Color.MAGENTA:
                        basket = processedData.basket_m
                    elif basket_color == Color.BLUE:
                        basket = processedData.basket_b

                    #if that kind of basket is in our list
                    if basket.exists:
                        if(basket.x>reso_x_mid-2 and basket.x<(reso_x_mid+2)) and (xcord>(reso_x_mid-2) and xcord<(reso_x_mid+2)):
                            robot.stop()
                            state=State.THROW
                            print("Robot is centering")
                            continue
                            #break

                        if basket.x<reso_x_mid:
                            basket_side=1
                        elif basket.x > reso_x_mid:
                            basket_side=-1
                            
                        speedX=0.13*basket_side

                        
                        speedY=0
                        speedR=1000*speedX/(640-radius)
                        if radius > (dist-5) or radius > (dist+5):
                            speedY += (radius-dist)/100
                               
                        if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                            speedR += (reso_x_mid- xcord) / 100
                                                   
                        robot.move(speedX, speedR, speedY, speedT)    
                    else:
                        #Orbit
                        speedY=0
                        speedX=0.4
                        speedR=1000*speedX/(640-radius)

                        if radius > (dist-5) or radius > (dist+5):
                            speedY += (radius-dist)/100
                        
                        if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                            speedR += (reso_x_mid- xcord) / 100 

                        if speedR > MaxOrbit_RSpeed:
                            speedR = MaxOrbit_RSpeed
                        if speedY > MaxOrbit_YSpeed:
                            speedY = MaxOrbit_YSpeed
                                                
                        robot.move(speedX, speedR, speedY, speedT)        
            
                            

            elif state == State.THROW:
                try:
                    targeted_ball=processedData.balls[-1]
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)

                    #Moves to ball
                    speedY = 0.3
                    speedR = 0
                    speedX=0
                    speedT=0
                    if dist > 440:
                        basketdist=processedData.basket_m.distance*100
                        speedT=calculator.calc_throwingSpeed(basketdist)
                        print("Throwing with speed: ", speedT)
                        #Throws
                        robot.move(speedX, speedR, speedY, int(speedT))
                        time.sleep(2)
                        #Gonna change the time sleep so it can aim til the throw, but works for now  
                        state = State.FIND_BALL
                        continue
                    
                    elif (xcord<(reso_x_mid-2) or xcord>(reso_x_mid+2)) and dist <=440:
                        speedR+=delta/100
                        robot.move(speedX, speedR, speedY, speedT)
                    elif (dist <=400):
                        robot.move(speedX,speedR,speedY,speedT)
                
                   
                except:
                    state=State.FIND_BALL

                
                        
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
