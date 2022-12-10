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
    GRAB_BALL=2
    THROW=3
    TRYMOTORS=4
    TESTCAMERA=5
    TMOTOR=6
    CALIBRATION=7
    WAITING=8
    NO_BALLS = 9
    MOVE_BASKET = 10
    FIND_BASKET = 11
    

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
    
    mid_offset = 11
    reso_x_mid = cam.rgb_width/2 + mid_offset #443
    
    spin = 26
    radius = 365
    ball_right_side = 0
    basket_right_side = 1
    basket_exists = False
    
    basket_edge_buffer = 75

    #orbit state constants
    max_orbit_Yspeed = 0.15
    max_orbit_Rspeed = 6
    orbit_to_cali_buffer = 15
    change_orbit_Y = 0.2
    orbit_thresh = 340

    #move state constants
    max_move_Xspeed = 0.8
    max_move_Yspeed = 2.5
    max_move_Rspeed = 4
    change_move_X = 0.01
    change_move_Y = 0.005
    change_move_R = 0.01

    #Timer
    timer = False
    elapsed_time = 0
    start_time = 0

    try:
        while True:
            #print("------------------------- Current state: ", state, "-------------------------")
            speed_T = 0
            grabber = 6000
            thrower_angle = 6900
            if state == State.THROW or state == State.TMOTOR or state == State.TESTCAMERA:
                processed_Data = processor.process_frame(aligned_depth=True)
            else:
                processed_Data = processor.process_frame(aligned_depth=False)
            
            if(processed_Data.basket_b.exists or processed_Data.basket_m.exists):
                basket_exists = True
                
                if basket_color == BasketColor.MAGENTA:
                    basket = processed_Data.basket_m
                elif basket_color == BasketColor.BLUE:
                    basket = processed_Data.basket_b
                
            else:
                basket_exists=False
                if basket_color == BasketColor.MAGENTA:
                    basket = processed_Data.basket_m
                elif basket_color == BasketColor.BLUE:
                    basket = processed_Data.basket_b
                
                
            
            frame_cnt += 1
            frame += 1

            #State for testing the wheel motors
            if state == State.TRYMOTORS:
                #robot.try_motors()
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
                    robot.move(0,0,0,0,6900,4800)
                    print('kas pall on grabberis - ', robot.ball_in_grabber)
                    continue
                except Exception as e:
                    print(e)
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
                    print(run, blue)
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
                if robot.ball_in_grabber>0:
                    grabber = 4800
                    state = State.FIND_BASKET
                    continue
                
                print("--------Searching ball--------")
                print(processed_Data.balls)
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
                if elapsed_time > 2.5 and basket_exists:
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
                    
                    if processed_Data.basket_b.distance > 150:
                        basket_no_ball = processed_Data.basket_b
                        speed_Y = calculator.sig_approach(basket_no_ball.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket_no_ball.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket_no_ball.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)

                    elif processed_Data.basket_m.distance > 150:
                        basket_no_ball = processed_Data.basket_m
                        speed_Y = calculator.sig_approach(basket_no_ball.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket_no_ball.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket_no_ball.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)

                    else:
                        timer=False
                        elapsed_time=0
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
                    if x_cord < (reso_x_mid + 150) and x_cord >(reso_x_mid -150) and dist > 365:
                        state=State.GRAB_BALL
                          
                    else:
                        #Moves to ball

                        speed_R = -(calculator.sig_correction_move(x_cord,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(x_cord,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                        print("x speed on - ", speed_X)
                        print("y speed on - ", speed_Y)
                        print("r speed on - ", speed_R)

                else:
                    #If there is no ball
                    state = State.FIND_BALL
                        
            elif state==State.GRAB_BALL:
                print("--------Grab ball--------")
                #If there is ball
                grabber=6800
                speed_R=0
                speed_X=0
                speed_Y=0.4
                if len(processed_Data.balls) > 0:
                    targeted_ball=processed_Data.balls[-1]
                    x_cord=targeted_ball.x
                    y_cord=targeted_ball.y
                    dist=targeted_ball.distance
                    if dist < 365:
                        speed_R=0
                        speed_X=0
                        speed_Y=0.4
                    else:
                        speed_Y = calculator.sig_approach(y_cord,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(x_cord,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(x_cord,max_move_Xspeed, change_move_X)


            
                robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)  
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
            
                if robot.ball_in_grabber>0:
                    grabber=4800
                    state = State.FIND_BASKET   
                    elapsed_time = 0
                    timer = False
                    robot.stop()
                    continue
                elif elapsed_time>1.5:
                    state=State.FIND_BALL
                    elapsed_time=0
                    timer=False
                    grabber=4800
                                     
                    
                elif len(processed_Data.balls)==0 and timer==False:
                    State.FIND_BALL
                

            elif state==State.FIND_BASKET:
                print("--------Searching BASKET--------")
                #<


                if basket.exists and basket.distance<=150:
                    state = State.CALIBRATION
                    
                elif basket.exists and basket.distance>150:
                    state = State.MOVE_BASKET
                    
                
                else:
                    if basket_right_side == 1:
                        robot.find_ball(-spin)
                    elif basket_right_side == 0:
                        robot.find_ball(spin)

            elif state==State.MOVE_BASKET:
                grabber=4800
                if (basket.exists):
                    
                    if basket.distance >=250:
                        speed_Y = calculator.sig_approach(basket.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                    
                    elif basket.distance >250:
                        state=State.CALIBRATION

                    else:
                        state = State.FIND_BALL
                else:
                    state=State.FIND_BASKET
            
            
            elif state == State.CALIBRATION:
                
                try:
                
                    print("----CALIBRATION----")
                    speed_T = int(calculator.calc_throwingSpeed(basket.distance))
                    speed_R = -(calculator.sig_correction_move(basket.x, 3, 0.008))
                    speed_Y = 0
                    grabber = 3000
                    speed_X = 0
                    
                    
                    if timer:
                        elapsed_time = time.time() - start_time
                    else:
                        start_time = time.time()
                        timer = True
                    print('Elapsed time: ', elapsed_time)
                    if elapsed_time > 0.2:
                        grabber=4800
                    if elapsed_time > 1:
                        state = State.THROW
                        elapsed_time = 0
                        timer = False
                        continue
                    print('saadan mootoritele ajad')
                    robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                    
                except Exception as e:
                    print(e)
                    state = State.FIND_BASKET
                    elapsed_time = 0
                    timer = False

            elif state==State.THROW:
                speed_T = int(calculator.calc_throwingSpeed(basket.distance))
                speed_R = 0
                speed_Y = 0
                grabber = 6900
                speed_X = 0
                robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
                    print('Elapsed time: ', elapsed_time)
                if elapsed_time > 1:
                    state = State.FIND_BALL
                    elapsed_time = 0
                    timer = False
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
