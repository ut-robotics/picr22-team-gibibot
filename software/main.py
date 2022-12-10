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
    NO_BALLS = 9
    

class BasketColor(Enum):
    BLUE=0
    MAGENTA=1





def main_loop():
    state=State.WAITING
    debug=True
    ref_cmds=False
    basket_color = BasketColor.MAGENTA

    First_Ref=1
    ref=client.Client()
    if ref_cmds==True:
        ref.start()
    cam=camera.RealsenseCamera(exposure=100) #defaulti peal on depth_enabled = True
    processor = image_processor.ImageProcessor(cam, debug=debug, color_config = "colors/colors.pkl")
    robot=movement.OmniRobot()
    calculator = calculations.Calculations()
    comms=communication.Communication()
    processor.start()
    start=time.time()
    
    fps = 0
    frame = 0
    frame_cnt = 0
    
    mid_offset = 19
    reso_x_mid = cam.rgb_width/2 + mid_offset #443
    
    spin = 15
    radius = 365
    ball_right_side = 0
    basket_right_side = 1
    basket_exists=False
    basket=""
    basket_edge_buffer = 75

    #orbit state constants
    max_orbit_Yspeed = 0.15
    max_orbit_Rspeed = 3
    orbit_to_cali_buffer = 15
    change_orbit_Y = 0.1

    #move state constants
    max_move_Xspeed = 1.0
    max_move_Yspeed = 1.3
    max_move_Rspeed = 4
    change_move_X = 0.01
    change_move_Y = 0.01
    change_move_R = 0.01

    #Timer
    timer = False
    elapsed_time = 0
    start_time = 0

    try:
        while True:
            print("------------------------- Current state: ", state, "-------------------------")
            speed_T=0
            if state == State.THROW or state == State.TMOTOR or state == State.TESTCAMERA:
                processed_Data = processor.process_frame(aligned_depth=True)
            else:
                processed_Data = processor.process_frame(aligned_depth=False)
            
            if(processed_Data.basket_b.exists or processed_Data.basket_m.exists):
                basket_exists=True
                try:
                    if basket_color == BasketColor.MAGENTA:
                        basket = processed_Data.basket_m
                    elif basket_color == BasketColor.BLUE:
                        basket = processed_Data.basket_b
                except:
                    basket=""
            else:
                basket_exists=False
            
            frame_cnt += 1
            frame += 1

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
            
                if ref_cmds == True:
                    run, blue = ref.get_current_referee_command()
                    print(run, blue )
                    if run == True and First_Ref==1:
                        state = State.FIND_BALL
                        if blue == True:
                            basket_color = BasketColor.BLUE
                        else:
                            basket_color = BasketColor.MAGENTA
                        First_Ref = 0
                        continue
                    elif run == False and First_Ref==0:
                        state = State.WAITING
                        First_Ref = 1
                elif ref_cmds == False and First_Ref==1:
                    state = State.FIND_BALL
                    First_Ref = 0
        
            except:
                print("Server client communication failed.")
                continue
            

            if state == State.WAITING:
                robot.stop()
                print("WAITIN")
                
            
            


            if state == State.FIND_BALL:
                
                print("--------Searching ball--------")
                print(processed_Data.balls)
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True

                if elapsed_time > 5 basket_exists:
                    state = State.NO_BALLS

                if len(processed_Data.balls) != 0:
                    state = State.MOVE
                else:
                    if ball_right_side == 1:
                        robot.find_ball(-spin)
                    elif ball_right_side == 0:
                        robot.find_ball(spin)

            elif state == State.NO_BALLS:
                if (basket_exists):
                    
                    if basket.distance > 100:
                        speed_Y = calculator.sig_approach(target.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(target.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(target.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T)

                    else:
                        state = State.FIND_BALL
                
                        
            elif state == State.MOVE:
                timer = False
                elapsed_time = 0
                print("--------Moving to ball--------")
                
                if (len(processed_Data.balls) != 0):
                    # if pall seespool valget ja musta, siis edasi, kui on väljaspool, siis find_ball
                    
                    targeted_ball = processed_Data.balls[-1]
                    x_cord = targeted_ball.x
                    y_cord = targeted_ball.y
                    dist = targeted_ball.distance
                    delta = ((x_cord-reso_x_mid)/reso_x_mid)
                    
                    
                    #Follows where was the ball last so find_ball function is more efficient
                    if delta<0:
                        ball_right_side = 0
                    else:
                        ball_right_side = 1
                    
                    #SpeedY based on ball distance

                    speed_Y = calculator.sig_approach(y_cord,max_move_Yspeed, change_move_Y)
                        
                    #Controlls that balls location is ready for robot's orbit function
                    if x_cord < (reso_x_mid + 150) and x_cord >(reso_x_mid -150) and dist > 355:
                        state=State.ORBIT
                          
                    else:
                        #Moves to ball

                        speed_R = -(calculator.sig_correction_move(x_cord,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(x_cord,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                        print("x speed on - ", speed_X)
                        print("y speed on - ", speed_Y)
                        print("r speed on - ", speed_R)

                else:
                    #If there is no ball
                    state = State.FIND_BALL
                        
            elif state==State.ORBIT:
                print("--------Centering ball and basket--------")
                #If there is ball
                if len(processed_Data.balls) > 0:
                    targeted_ball=processed_Data.balls[-1]
                    x_cord=targeted_ball.x
                    y_cord=targeted_ball.y
                    dist=targeted_ball.distance

                    #controlls basket colour
                    if dist < 340:
                        print("PALL LIIGA KAUGEL LAHEME OTSIME PALLI")
                        state = State.FIND_BALL
                        continue
                    

                    #if that kind of basket is in our list
                    if basket.exists and (basket.x > basket_edge_buffer and basket.x < cam.rgb_width - basket_edge_buffer):
                        print("BASKET ON OLEMAS")
       
                        speed_X = calculator.sig_correction_move(x_cord, 0.5, 0.01)
                        
                      
                        speed_Y = calculator.sig_correction_orbit(y_cord, max_orbit_Yspeed, change_orbit_Y)


                        speed_R = -(calculator.sig_correction_move(basket.x, max_orbit_Rspeed, 0.008))
                        
                        
                        if(basket.x > reso_x_mid-orbit_to_cali_buffer and basket.x < (reso_x_mid + orbit_to_cali_buffer)) and (x_cord > (reso_x_mid-15) and x_cord < (reso_x_mid+15)):
                            robot.stop()
                            state = State.CALIBRATION
                            print("Robot is centering")
                            continue
                        
                        
                        if speed_R > max_orbit_Rspeed:
                            speed_R = max_orbit_Rspeed
                        elif speed_R < -1*max_orbit_Rspeed:
                            speed_R = -1*max_orbit_Rspeed
                        
                            
                        print("SAADAME MOOTORILE SPEEDID XRYT: ", speed_X, " ", speed_R," " ,speed_Y, " " ,speed_T)                        
                        
                        robot.move(speed_X, speed_R, speed_Y, speed_T)
                            
                    else:
                        #Orbit
                        speed_Y = 0
                        speed_X = 0.65
                        speed_R = 1000*speed_X/(640-radius)

                        speed_Y = calculator.sig_correction_orbit(y_cord, max_orbit_Yspeed, change_orbit_Y)
                        
                        speed_R += -(calculator.sig_correction_move(x_cord, 0.5, 0.01))

                        if speed_R > max_orbit_Rspeed:
                            speed_R = max_orbit_Rspeed
                        if speed_R < -1*max_orbit_Rspeed:
                            speed_R = -1*max_orbit_Rspeed
                        
                                                
                        robot.move(speed_X, speed_R, speed_Y, speed_T)  
                else:
                    state = State.FIND_BALL      
            
            elif state == State.CALIBRATION:
                
                try:
                    print("----CALIBRATION----")
                    speed_T = int(calculator.calc_throwingSpeed(basket.distance))
                    speed_R = -(calculator.sig_correction_move(basket.x, 0.5, 0.008))
                    speed_Y = 0.2
                    if len(processed_Data.balls) > 0:
                        targeted_ball = processed_Data.balls[-1]
                        dist = targeted_ball.distance
                    #Moves to ball
                        speed_X = calculator.sig_correction_move(x_cord, 0.5, 0.01)
                    else:
                        speed_X = 0
                    
                    
                    if len(processed_Data.balls) > 0 and dist > radius:
                        speed_X = 0
                    
                     
                        
                     
                    if timer:
                        elapsed_time = time.time() - start_time
                    else:
                        start_time = time.time()
                        timer = True
                    print('Elapsed time: ', elapsed_time)
                    if elapsed_time > 2:
                        state = State.FIND_BALL
                        elapsed_time = 0
                        timer = False
                        continue
                    robot.move(speed_X, speed_R, speed_Y, speed_T)
                    
                except Exception as e:
                    print(e)
                    state = State.FIND_BALL
                    elapsed_time = 0
                    timer = False
                
                        
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
