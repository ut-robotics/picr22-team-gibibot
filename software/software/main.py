import image_processor
import camera
import motion
import cv2
import time

def main_loop():
    debug = True
    
    motion_sim = motion.TurtleRobot()
    motion_sim2 = motion.TurtleOmniRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    motion_sim.open()
    motion_sim2.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                if len(processedData.balls)==0:
                    motion.sendinfo(0,0,15,0,0)
                    motion.getinfo()
                
                else:
                    motion.sendinfo(0,0,0,0,0)
                    motion.getinfo()
                    
              
                ballssortedbysize = processedData.balls.sort(key= lambda x: x.size)
                biggestball=ballssortedbysize[0]
                print("Biggest ball: " + biggestball)
                xcord=biggestball.x
                ycord=biggestball.y
                print("X: " + xcord + "Y: " + ycord)
                
                #speed1 - x, speed2 - y, speed3 - rotation
                if xcord<930:
                    motion.sendinfo(0, 0, 15, 0, 0)
                    motion.getinfo()
                elif ycord>990:
                    motion.sendinfo(0, 0, -15, 0, 0)
                    motion.getinfo()
                else:
                    print("Pall on keskel")
                    motion.sendinfo(20,20,0,0,0)
                    motion.getinfo()
                
                
                    
                
                #if (frame_cnt > 1000):
                #    break

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()

main_loop()
