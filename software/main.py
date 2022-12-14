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
from random import choices

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
    
    mid_offset = 10
    reso_x_mid = cam.rgb_width/2 + mid_offset #443
    
    spin = 40
    radius = 365
    ball_right_side = 0
    
    basket_exists = False

    find_basket_side = 1
    
    basket_edge_buffer = 75

    #orbit state constants
    max_orbit_Yspeed = 0.15
    max_orbit_Rspeed = 6
    orbit_to_cali_buffer = 15
    change_orbit_Y = 0.2
    orbit_thresh = 340

    #move state constants
    max_move_Xspeed = 1
    max_move_Yspeed = 2.5
    max_move_Rspeed = 5
    change_move_X = 0.01
    change_move_Y = 0.005
    change_move_R = 0.01
    

    #Timer
    timer = False
    find_timer = False
    grab_timer = False
    elapsed_time = 0
    grab_elapsed_time = 0
    grab_start_time = 0
    elapsed_time_search = 0
    start_time = 0
    start_time_search = 0

    try:
        while True:
            speed_T = 0
            grabber = 6900
            thrower_angle = 6900
            if state == State.THROW or state == State.TMOTOR or state == State.TESTCAMERA or state==State.FIND_BASKET or state==State.CALIBRATION or state==State.NO_BALLS:
                processed_Data = processor.process_frame(aligned_depth=True)
            else:
                processed_Data = processor.process_frame(aligned_depth=False)
            
            if(processed_Data.basket_b.exists or processed_Data.basket_m.exists):
                basket_exists = True
                
                
                if basket_color == BasketColor.MAGENTA:
                    basket = processed_Data.basket_m
                    
                    
                elif basket_color == BasketColor.BLUE:
                    basket = processed_Data.basket_b
                if basket.x > reso_x_mid:
                    find_basket_side = 1
                else:
                    find_basket_side = 1

                
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
                    continue
                except Exception as e:
                    print(e)
                    continue
                    
                
            
            if frame%30 == 0:
                print('-------------- hetke state -', state, '----------------')
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
                    elapsed_time = 0
                    continue
                
                print("--------Searching ball--------")
                print("elapsed time", elapsed_time)
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
                if elapsed_time > 2.2 and basket_exists:
                    state = State.NO_BALLS

                if find_timer:
                    elapsed_time_search = time.time() - start_time_search
                else:
                    start_time_search = time.time()
                    find_timer = True

                if elapsed_time_search > 0.2:
                    if spin != 0:
                        spin = 0
                    else:
                        spin = 40
                    elapsed_time_search = 0
                    find_timer = False

                if len(processed_Data.balls) != 0:
                    state = State.MOVE
                    timer = False
                    find_timer = False
                    elapsed_time = 0
                    elapsed_time_search = 0
                else:
                    if ball_right_side == 1:
                        robot.find_ball(-spin)
                    elif ball_right_side == 0:
                        robot.find_ball(spin)

            elif state == State.NO_BALLS:
                if (basket_exists):
                    
                    if processed_Data.basket_b.distance > 110:
                        basket_no_ball = processed_Data.basket_b
                        speed_Y = calculator.sig_approach(basket_no_ball.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket_no_ball.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket_no_ball.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                        timer=False
                        elapsed_time=0
                    


                    elif processed_Data.basket_m.distance > 110:
                        basket_no_ball = processed_Data.basket_m
                        speed_Y = calculator.sig_approach(basket_no_ball.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket_no_ball.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket_no_ball.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                        timer=False
                        elapsed_time=0
                    else:
                        state = State.FIND_BALL
                else:
                    if ball_right_side == 0:
                        robot.find_ball(-20)
                    elif ball_right_side == 0:
                        robot.find_ball(20)
                
                        
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
                    if x_cord < (reso_x_mid + 80) and x_cord >(reso_x_mid -80) and dist > 310:
                        state=State.GRAB_BALL
                          
                    else:
                        #Moves to ball

                        speed_R = -(calculator.sig_correction_move(x_cord,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(x_cord,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)

                else:
                    #If there is no ball
                    state = State.FIND_BALL
                        
            elif state==State.GRAB_BALL:
                print("--------Grab ball--------")
                print('elapsed time grav', elapsed_time)
                print('timer - ', timer)
                print('palli list - ', len(processed_Data.balls))
                if grab_timer:
                    grab_elapsed_time = time.time() - grab_start_time
                elif not grab_timer:
                    grab_start_time = time.time()
                    grab_timer = True
                if grab_elapsed_time > 3:
                    state=State.FIND_BALL
                    grab_elapsed_time=0
                    grab_timer=False
                #If there is ball
                grabber=6800
                speed_R=0
                speed_X=0
                speed_Y=0.45
                if len(processed_Data.balls) > 0:
                    targeted_ball=processed_Data.balls[-1]
                    x_cord=targeted_ball.x
                    y_cord=targeted_ball.y
                    dist=targeted_ball.distance
                if timer==True:
                    speed_R=0
                    speed_X=0
                else:
                    #speed_Y = calculator.sig_approach(y_cord,max_move_Yspeed, change_move_Y)
                    speed_R = -(calculator.sig_correction_move(x_cord, max_move_Rspeed-1.5, change_move_R))
                    speed_X = calculator.sig_correction_move(x_cord,max_move_Xspeed-0.5, change_move_X)


            
                robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)

                if timer:
                    elapsed_time = time.time() - start_time
                elif not timer and dist >465 and x_cord>reso_x_mid+80 and x_cord<reso_x_mid-80:
                    start_time = time.time()
                    timer = True
            
                if robot.ball_in_grabber>0:
                    
                    grabber=4800
                    state = State.FIND_BASKET   
                    elapsed_time = 0
                    timer = False
                    robot.stop()
                    continue
                # elif len(processed_Data.balls)==0 and timer==False:
                #     state=State.FIND_BALL
                elif elapsed_time>0.8:
                    state=State.FIND_BALL
                    elapsed_time=0
                    timer=False
                    grabber=4800
                                     
                    
                
                

            elif state==State.FIND_BASKET:
                print("--------Searching BASKET--------")
                #<
                
                if find_timer:
                    elapsed_time_search = time.time() - start_time_search
                else:
                    start_time_search = time.time()
                    find_timer = True

                if elapsed_time_search > 0.2:
                    if spin != 0:
                        spin = 0
                    else:
                        spin = 40
                    elapsed_time_search = 0
                    find_timer = False

                if basket.exists and (basket.distance<=250 and basket.distance>=65):
                    state = State.CALIBRATION
                    elapsed_time = 0
                    elapsed_time_search = 0
                    find_timer = False
                    timer = False
                    
                elif basket.exists and (basket.distance>250 or basket.distance<65):
                    state = State.MOVE_BASKET
                    elapsed_time = 0
                    elapsed_time_search = 0
                    find_timer = False
                    timer = False
                

                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
                # if elapsed_time > 3:
                #     state = State.NO_BALLS
                #     elapsed_time = 0
                #     timer = False
                #     continue
                print('Elapsed time search basket: ', elapsed_time)
                print('elapsed time search - ', elapsed_time_search )
                print('basket side - ',find_basket_side)
                print('spin value - ',spin)

                robot.find_ball(spin*find_basket_side)

            elif state==State.MOVE_BASKET:
                grabber=4800
                if (basket.exists):
                    
                    
                    if basket.distance<=65:
                        speed_Y = -calculator.sig_approach(basket.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket.x,max_move_Rspeed, change_move_R))
                        speed_X = -calculator.sig_correction_move(basket.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)


                    elif basket.distance >=250:
                        speed_Y = calculator.sig_approach(basket.y,max_move_Yspeed, change_move_Y)
                        speed_R = -(calculator.sig_correction_move(basket.x,max_move_Rspeed, change_move_R))
                        speed_X = calculator.sig_correction_move(basket.x,max_move_Xspeed, change_move_X)
                        robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                    
                    
                    elif basket.distance <250 and basket.distance>65:
                        state=State.CALIBRATION
                        timer = False
                        elapsed_time = 0

                    else:
                        state = State.FIND_BALL
                else:
                    state=State.FIND_BASKET
            
            
            elif state == State.CALIBRATION:
                
                try:
                
                    print("----CALIBRATION----")
                    speed_T = 0
                    speed_R = -(calculator.sig_correction_move(basket.x, 4, 0.011))
                    speed_Y = 0
                    grabber = 3000
                    speed_X = 0
                    
                    
                    if timer:
                        elapsed_time = time.time() - start_time
                    else:
                        start_time = time.time()
                        timer = True
                    if elapsed_time > 0.3:
                        grabber=4800
                        speed_T = int(calculator.calc_throwingSpeed(basket.distance))
                    if elapsed_time > 1: #and basket.x>reso_x_mid-10 and basket.x<reso_x_mid+10:
                        state = State.THROW
                        elapsed_time = 0
                        timer = False
                        continue
                    robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                    
                except Exception as e:
                    print(e)
                    state = State.FIND_BASKET
                    elapsed_time = 0
                    timer = False

            elif state==State.THROW:
                speed_T = int(calculator.calc_throwingSpeed(basket.distance))
                speed_R = -(calculator.sig_correction_move(basket.x, 4, 0.008))
                speed_Y = 0
                grabber = 6900
                speed_X = 0
                robot.move(speed_X, speed_R, speed_Y, speed_T, thrower_angle, grabber)
                if timer:
                    elapsed_time = time.time() - start_time
                else:
                    start_time = time.time()
                    timer = True
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
