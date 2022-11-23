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
import Color as c
import client

class State(Enum):
    FIND_BALL=0
    MOVE=1
    ORBIT=2
    THROW=3
    TRYMOTORS=4
    TESTCAMERA=5
    TMOTOR=6
    CALIBRATION=7
    WAITING=8
    

class BasketColor(Enum):
    BLUE=0
    MAGENTA=1





def main_loop():
    state=State.TMOTOR
    debug=True
    ref_cmds=False
    basket_color = BasketColor.BLUE

    First_Ref=1
    ref=client.Client()
    if ref_cmds==True:
        ref.start()
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
    
    spin = 13
    radius=375
    ball_right_side=0
    basket_right_side = 1
    max_orbit_Yspeed=0.2
    max_orbit_Rspeed=3
    max_move_Yspeed=1.2
    orbit_to_cali_buffer=17
    try:
        while True:
            speed_T=0
            if state == State.THROW:
                processed_Data = processor.process_frame(aligned_depth=True)
            else:
                processed_Data = processor.process_frame(aligned_depth=False)
            
            frame_cnt+=1
            frame+=1

            #State for testing the wheel motors
            if state == State.TRYMOTORS:
                robot.try_motors()
                robot.straight_movement(0.3)
                time.sleep(2)
                robot.straight_movement(-0.3)
                time.sleep(2)
                robot.side_movement(0.3)
                time.sleep(2)
                robot.side_movement(-0.3)
                time.sleep(2)
                robot.stop()
                break
            
            #State for calibrating the thrower motor speed with distance
            if state == State.TMOTOR:
                try:
                    processed_Data = processor.process_frame(aligned_depth=True)
                    if basket_color == c.Color.MAGENTA:
                        basket = processed_Data.basket_m
                    elif basket_color == c.Color.BLUE:
                        basket = processed_Data.basket_b
                    print("Measured basket distance: ", basket.distance)
                    
                    speed_T=int(input("Give motor speed: "))
                    if speed_T == "stop":
                        break
                    robot.test_thrower(speed_T, 0)
                    continue
                except:
                    print("KORVI POLE")
                    continue

            #State for camera testing
            if state==State.TESTCAMERA:
                try:
                    processed_Data = processor.process_frame(aligned_depth=True)
                except:
                    continue
                    
                
            
            if frame%30 == 0:
                frame = 0
                end = time.time()
                fps = 30/(end-start)
                start = end 
                print(fps)  

            
            try:
            
                if ref_cmds==True:
                    run, blue=ref.get_current_referee_command()
                    print(run, blue )
                    if run ==True and First_Ref==1:
                        state=State.FIND_BALL
                        if blue==True:
                            basket_color=BasketColor.BLUE
                        else:
                            basket_color=BasketColor.MAGENTA
                        First_Ref=0
                        continue
                    elif run==False and First_Ref==0:
                        state=State.WAITING
                        First_Ref=1
                elif ref_cmds==False and First_Ref==1:
                    state=State.FIND_BALL
                    First_Ref=0
        
            except:
                print("Server client communication failed.")
            

            if state==State.WAITING:
                robot.stop()
                print("WAITIN")
                



            if state == State.FIND_BALL:
                
                print("--------Searching ball--------")
                print(processed_Data.balls)
                if len(processed_Data.balls)!=0:
                    state=State.MOVE
                else:
                    if ball_right_side==1:
                        robot.find_ball(spin)
                    elif ball_right_side==0:
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
                    if dist > 250:
                        speed_Y=0.25
                    elif dist > 325:
                        speed_Y=0.15
                    elif dist > 385:
                        speed_Y=-0.15
                        
                    else:
                        speed_Y=max_move_Yspeed
                        
                    #Controlls that balls location is ready for robot's orbit function
                    if x_cord < (reso_x_mid + 5) and x_cord >(reso_x_mid -5) and dist > 368:
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
                    if dist<220:
                        print("PALL LIIGA KAUGEL LAHEME OTSIME PALLI")
                        state=State.FIND_BALL
                        continue
                    
                    if basket_color == BasketColor.MAGENTA:
                        basket = processed_Data.basket_m
                    elif basket_color == BasketColor.BLUE:
                        basket = processed_Data.basket_b

                    #if that kind of basket is in our list
                    if basket.exists:
                        print("BASKET ON OLEMAS")
                        
                        if basket.x<reso_x_mid-2:
                           basket_right_side=1
                        elif basket.x > reso_x_mid+2:
                           basket_right_side=-1
                        speed_Y=0
                        speed_X=0.25*basket_right_side
                        speed_R=1000*(speed_X)/(640-radius)

                      
                        if radius > (dist-1) or radius > (dist+1):
                            speed_Y += (radius-dist)/ 25 
        
                        if  (x_cord > (reso_x_mid + 1) or x_cord < (reso_x_mid - 1)): #and (x_cord > (reso_x_mid - 10) and x_cord < (reso_x_mid +10)):
                            speed_R += (reso_x_mid- x_cord) / 200
                        
                        if(basket.x>reso_x_mid-orbit_to_cali_buffer and basket.x<(reso_x_mid+orbit_to_cali_buffer)):# and (x_cord>(reso_x_mid-5) and x_cord<(reso_x_mid+5)):
                            robot.stop()
                            state=State.CALIBRATION
                            print("Robot is centering")
                            continue
                        
                        
                        if speed_R > max_orbit_Rspeed:
                            speed_R = max_orbit_Rspeed
                        elif speed_R < -1*max_orbit_Rspeed:
                            speed_R = -1*max_orbit_Rspeed
                        if speed_Y > max_orbit_Yspeed:
                            speed_Y = max_orbit_Yspeed
                            
                        print("SAADAME MOOTORILE SPEEDID XRYT: ", speed_X, " ", speed_R," " ,speed_Y, " " ,speed_T)                        
                        
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                            
                    else:
                        #Orbit
                        speed_Y=0
                        speed_X=0.5
                        speed_R=1000*speed_X/(640-radius)

                        if radius > (dist-3) or radius > (dist+3):
                            speed_Y += (radius-dist)/25
                        
                        if x_cord > (reso_x_mid + 1) or x_cord < (reso_x_mid - 1):
                            speed_R += (reso_x_mid- x_cord) / 200

                        if speed_R > max_orbit_Rspeed:
                            speed_R = max_orbit_Rspeed
                        if speed_R < -1*max_orbit_Rspeed:
                            speed_R = -1*max_orbit_Rspeed
                        if speed_Y > max_orbit_Yspeed:
                            speed_Y = max_orbit_Yspeed
                                                
                        robot.move(speed_X, speed_R, speed_Y, speed_T)  
                else:
                    state=State.FIND_BALL      
            
            elif state==State.CALIBRATION:
                
                try:
                    print("----CALIBRATION----")
                    targeted_ball=processed_Data.balls[-1]
                    dist=targeted_ball.distance
                    delta=((x_cord-reso_x_mid)/reso_x_mid)

                    #Moves to ball
                    speed_Y = 0.2
                    speed_R = 0
                    speed_X=0
                    if dist > 420:
                        state=State.THROW
                        continue
                    
                    if (x_cord<(reso_x_mid-1) or x_cord>(reso_x_mid+1)) and x_cord>(reso_x_mid-24) and x_cord<(reso_x_mid+24):
                        speed_X+=delta*2
                    else:
                        state=State.FIND_BALL
                        
                    if (basket.x<(reso_x_mid-1) or basket.x>(reso_x_mid+1)):
                        speed_R+=(x_cord-basket.x)/(reso_x_mid)
                        
                    elif dist<=420 and dist >=radius-41:
                        speed_Y = 0.2
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                    else:
                        state=State.FIND_BALL
                   
                    robot.move(speed_X, speed_R, speed_Y, speed_T)
                    
                except:
                    state=State.FIND_BALL         

            elif state == State.THROW:
                basket_dist=basket.distance
                print("BASKET DISTANCE: ", basket_dist)
                speed_T=calculator.calc_throwingSpeed(basket_dist)
                print("Throwing with speed: ", speed_T)
                #Throws
                speed_X = 0
                speed_Y = 0.2
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
