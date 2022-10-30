from asyncio import base_tasks
from turtle import st, xcor
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
    tdist=[60,68,85,100,125,150,175,200,225,250,275,300,325,350,375,400,425]
    tspeeds=[472,472,500,520,550,620,670,730,780,810,865,900,930,985,1048,1115,1180]
    state="find_ball"
    spin = 12
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
    radius=385
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
                continue
            if state == "tmotor":
                try:
                    
                    processedData = processor.process_frame(aligned_depth=True)
                    #print("basket y kord   ", processedData.basket_m.y)
                    spt=int(input("NNA TSPEED: "))
                    robot.test_thrower(spt)
                    #print("DEPTH:", processedData.basket_m.distance*100)
                    continue
                except:
                    print("KORVI POLE")
            if state=="cameratest":
                try:
                    processedData = processor.process_frame(aligned_depth=True)
                    print("LEGIT DISTANCE: ", processedData.basket_m.distance)
                    #processedData.balls.sort(key= lambda x: x.size)
                    #targeted_ball=processedData.balls[-1]
                    #print("Palli y kord   ", targeted_ball.y)
                    #print("DEPTH:",processedData.depth_frame)
                    #print(processedData.basket_m)

                    
                    
                except:
                    continue
                    print("Ei leidnud")
                    
                
            
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
                    #print(dist)
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
                    if xcord < (reso_x_mid + 15) and xcord >(reso_x_mid -15) and dist > 250:
                        #robot.center_ball(xcord)
                        state="orbit"
                    else:
                        #Moves to ball
                        speedR = delta*(-3.5)
                        speedX = delta*0.5
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

                    
                    #controlls basket colour

                    if basket_color == "magenta":
                        basket = processedData.basket_m
                    elif basket_color == "blue":
                        basket = processedData.basket_b

                    #if that kind of basket is in our list
                    if basket.exists:
                        #deltabasket=((basket.x-reso_x_mid)/reso_x_mid)
                        if(abs(basket.x-reso_x_mid)<1):
                            robot.stop()

                            state="throw"
                            print("Robot is centering")
                            continue
                            #state="find_ball"

                        if basket.x<424:
                            basket_side=1
                        elif basket.x > 424:
                            basket_side=-1
                            
                        speedX=0.15*basket_side

                        
                        speedY=0
                        speedR=1000*speedX/(640-radius)
                        if radius > (dist-5):
                            speedY += (radius-dist)/100
                        elif radius > (dist+5):
                            speedY += (radius-dist)/100
                               
                        if xcord > (reso_x_mid + 1) or xcord < (reso_x_mid - 1):
                            speedR += (reso_x_mid- xcord) / 100 
                                                   
                        robot.move(speedX, speedR, speedY)    
                    else:
                        #Orbit
                        #orbit_speeds = Orbit(radius, reso_x_mid, xcord, dist)

                        speedY=0
                        speedX=0.6
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
                try:
                    processedData.balls.sort(key= lambda x: x.size)
                    targeted_ball=processedData.balls[-1]
                    dist=targeted_ball.distance
                    delta=((xcord-reso_x_mid)/reso_x_mid)

                


                    #Moves to ball
                    speedY = 0.3
                    speedR = 0
                    speedX=0
                    if dist > 440:
                        speedR = 0
                        speedX = 0
                        #speedT=950
                        basketdist=processedData.basket_m.distance*100
                        
                        #basketdist=processedData.basket_m.y
                        for i in range(len(tdist)):
                            
                            if basketdist<=tdist[i]:
                                #speedT=((-500+basketdist)*tspeeds[i])/(-500+tsize[i])
                                speedT1=(basketdist*tspeeds[i]/tdist[i])
                                
                                break
                        for i in range(len(tdist)):
                            if basketdist<=tdist[i]:
                                if i!=0:
                                    speedT2=(basketdist*tspeeds[i-1]/tdist[i-1])
                                elif i==0:
                                    speedT2=(basketdist*tspeeds[i]/tdist[i])
                                break
                        speedT=(speedT1+speedT2)/2   
                        print("Throwing with speed: ", speedT)
                        robot.throw(speedX, speedR, speedY, int(speedT))
                        time.sleep(2)
                        state = "find_ball"
                        continue
                    elif (xcord<422 or xcord>426) and dist <=440:
                        speedR+=delta/100
                        robot.move(speedX, speedR, speedY)
                
                

                
                    
                except:
                    state="find_ball"
                    #speedR = delta*(-1.75)
                    #speedX = delta*0.25
                    #robot.move(speedX, speedR, speedY)

                
                        
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
