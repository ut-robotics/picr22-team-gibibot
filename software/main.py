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
import Color

class State(Enum):
    FIND_BALL=0
    MOVE=1
    ORBIT=2
    THROW=3
    TRYMOTORS=4
    TESTCAMERA=5
    TMOTOR=6
    CALIBRATION=7
    



def main_loop():
    state=State.FIND_BALL
    spin = 12
    debug=True
    color=Color.Color()
    basket_color=color.MAGENTA
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
    radius=400
    #Which side ball is on
    ball_right_side=0
    max_orbit_Yspeed=0.2
    max_orbit_Rspeed=2
    try:
        while True:
            speed_T=0
            if state == State.THROW:
                processed_Data = processor.process_frame(aligned_depth=True)
            else:
                processed_Data = processor.process_frame(aligned_depth=False)
            
            frame_cnt+=1
            frame+=1
            #Just Testing states for own comfort
            if state == State.TRYMOTORS:
                
                robot.try_motors()
                break
            if state == State.TMOTOR:
                try:
                    processed_Data = processor.process_frame(aligned_depth=True)
                    spt=int(input("ANNA TSPEED: "))
                    robot.test_thrower(spt)
                    
                    continue
                except:
                    print("KORVI POLE")
            if state==State.TESTCAMERA:
                try:
                    processed_Data = processor.process_frame(aligned_depth=True)
                    print("MUST JOON DIST: ", processed_Data.lines_b)
                    print("VALGE JOON DIST: ", processed_Data.lines_w)
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
                if len(processed_Data.balls)!=0:
                    state=State.MOVE
                else:
                    if ball_side==1:
                        robot.find_ball(spin)
                    elif ball_side==0:
                        robot.find_ball((-1)*spin)
                
                
                        
            elif state == State.MOVE:
                print("--------Moving to ball--------")
                if (len(processed_Data.balls)!=0):
                    # if pall seespool valget ja musta, siis edasi, kui on väljaspool, siis find_ball
                    
                    targeted_ball=processed_Data.balls[-1]
                    x_cord=targeted_ball.x
                    y_cord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((x_cord-reso_x_mid)/reso_x_mid)
                        #Follows where is ball so find_ball function is more efficient
                    if delta<0:
                        ball_right_side = 0
                    else:
                        ball_right_side = 1
                    
                        #SpeedY based on ball distance
                    if dist > 175:
                        speed_Y=0.4
                    else:
                        speed_Y=1
                        #Controlls that balls location is ready for robot's orbit function
                    if x_cord < (reso_x_mid + 15) and x_cord >(reso_x_mid -15) and dist > 275:
                        #robot.center_ball(xcord)
                        state=State.ORBIT
                    else:
                        #Moves to ball
                        speed_R = delta*(-3.5)
                        speed_X = delta*0.5
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                else:
                    #If there is no ball
                    state=State.FIND_BALL
                        
            elif state==State.ORBIT:
                print("--------Centering ball and basket--------")
                #If there is ball
                if len(processed_Data.balls)>0:
                    targeted_ball=processed_Data.balls[-1]
                    x_cord=targeted_ball.x
                    y_cord=targeted_ball.y
                    dist=targeted_ball.distance
                    delta=((x_cord-reso_x_mid)/reso_x_mid)

                    #controlls basket colour
                    if dist<250:
                        State.FIND_BALL
                    
                    if basket_color == color.MAGENTA:
                        basket = processed_Data.basket_m
                    elif basket_color == color.BLUE:
                        basket = processed_Data.basket_b

                    #if that kind of basket is in our list
                    if basket.exists:
                        print("BASKET ON OLEMAS")
                        if(basket.x>reso_x_mid-2 and basket.x<(reso_x_mid+2)): #and (xcord>(reso_x_mid-5) and xcord<(reso_x_mid+5)):
                            robot.stop()
                            state=State.THROW
                            print("Robot is centering")
                            continue

                        speed_Y=0
                        basket_right_side = 1
                        speed_R=1000*speed_X/(640-radius)
                        if basket.x<reso_x_mid-2:
                           basket_right_side=1
                        elif basket.x > reso_x_mid+2:
                           basket_right_side=-1
                         
                        speedX=0.15*basket_right_side
                      
                        if radius > (dist-5) or radius > (dist+5):
                            speed_Y += (radius-dist)/100
        
                        if  x_cord > (reso_x_mid + 1) or x_cord < (reso_x_mid - 1):
                            speed_R += (reso_x_mid- x_cord) / 100
                            
                        print("SAADAME MOOTORILE SPEEDID XRYT: ", speed_X, " ", speed_R," " ,speed_Y, " " ,speed_T)                        
                        robot.move(speed_X, speed_R, speed_Y, speed_T)    
                    else:
                        #Orbit
                        speed_Y=0
                        speed_X=0.4
                        speed_R=1000*speed_X/(640-radius)

                        if radius > (dist-5) or radius > (dist+5):
                            speed_Y += (radius-dist)/100
                        
                        if x_cord > (reso_x_mid + 1) or x_cord < (reso_x_mid - 1):
                            speedR += (reso_x_mid- x_cord) / 100 

                        if speed_R > max_orbit_Rspeed:
                            speed_R = max_orbit_Rspeed
                        if speed_Y > max_orbit_Yspeed:
                            speed_Y = max_orbit_Yspeed
                                                
                        robot.move(speed_X, speed_R, speed_Y, speed_T)  
                else:
                    State.FIND_BALL      
            
            elif state==State.CALIBRATION:
                
                try:
                    print("----CALIBRATION----")
                    targeted_ball=processed_Data.balls[-1]
                    dist=targeted_ball.distance
                    delta=((x_cord-reso_x_mid)/reso_x_mid)

                    #Moves to ball
                    speed_Y = 0.3
                    speed_R = 0
                    speed_X=0
                    if dist > 440:
                        state=State.THROW
                        continue
                    elif dist<=440 and dist >=radius-10:
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                    else:
                        State.FIND_BALL
                    
                    # if (xcord<(reso_x_mid)) or xcord>(reso_x_mid):
                    #     print("KALIBREERIN")
                    #     speedR+=delta*10
                    #     if(abs(xcord-basket.x))>=1:
                    #         speedX+=xcord-basket.x/1000
                    #     if speedX>0.5 or speedR>4 or speedX<-0.5 or speedR<-4:
                    #         state=State.FIND_BALL
                    #     print("PEEDX; ", speedX, "speedy: ", speedY)
                    #     robot.move(speedX, speedR, speedY, int(speedT))
                    # else:
                    #     print("SOIDAN EDASI ET VISATA")
                    #     robot.move(speedX, speedR, speedY, int(speedT))
                    
                except:
                    state=State.FIND_BALL        

            elif state == State.THROW:
                basket_dist=processed_Data.basket_m.distance*100
                speed_T=calculator.calc_throwingSpeed(basket_dist)
                print("Throwing with speed: ", speed_T)
                #Throws
                speed_X = 0
                speed_Y = 0.3
                speed_R =0
                robot.move(speed_X, speed_R, speed_Y, int(speed_T))
                time.sleep(2)
                #Gonna change the time sleep so it can aim til the throw, but works for now  
                state = State.FIND_BALL
                continue
                
                        
            if debug:
                    debug_frame = processed_Data.debug_frame

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
