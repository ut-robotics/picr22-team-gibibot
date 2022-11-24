import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c
from jit import *


class Object():
    def __init__(self, x = -1, y = -1, size = -1, distance = -1, exists = False):
        self.x = x
        self.y = y
        self.size = size
        self.distance = distance
        self.exists = exists

    def __str__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)

    def __repr__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)


# results object of image processing. contains coordinates of objects and frame data used for these results
class ProcessedResults():

    def __init__(self, 
                balls=[], 
                basket_b = Object(exists = False), 
                basket_m = Object(exists = False), 
                color_frame = [],
                depth_frame = [],
                fragmented = [],
                debug_frame = []) -> None:


        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented
        # can be used to illustrate things in a separate frame buffer
        self.debug_frame = debug_frame


#Main processor class. processes segmented information
class ImageProcessor():
    def __init__(self, camera, color_config = "colors.pkl", debug = False):
        self.camera = camera
        
        self.color_config = color_config
        with open(self.color_config, 'rb') as conf:
            self.colors_lookup = pickle.load(conf)
            self.set_segmentation_table(self.colors_lookup)

        self.fragmented	= np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.t_balls = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_b = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_basket_m = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.closest_line = Object(x = 10, y = 10, size = 10, distance = 10, exists = True)

        self.debug = debug
        self.debug_frame = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, fragments, distance_b, distance_m) -> list:
        contours, hierarchy = cv2.findContours(t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #mask = t_balls
        #mask = cv2.drawContours(mask, contours, -1, (255,255,255), 24)
        #orig_mask     = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        kernel_size = 1
        kernel  = np.ones((kernel_size,kernel_size), np.uint8)
        dilated_mask  = cv2.dilate(t_balls, kernel, iterations=1)
        
        kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
        morphed  = cv2.morphologyEx(dilated_mask, cv2.MORPH_CLOSE, kernel)
        dilated_contours, hierarchy = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        basket_distance = 999

        if distance_b != -1:
            basket_distance = distance_b
        elif distance_m != -1:
            basket_distance = distance_m
        elif distance_b == -1 or distance_m == -1:
            basket_distance = 60
        #print("BASKET DISTANCE IMAGES: ", basket_distance)

        balls = []
        
        for contour in dilated_contours:
        #for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = cv2.contourArea(contour)

            if size < 20:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            ys	= np.array(np.arange(y + h, self.camera.rgb_height-30), dtype=np.uint16)
            xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            line_to_ball = fragments[ys, xs]

            inside = self.analyze_line(line_to_ball)

            #print(inside)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = obj_y

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)

            if (inside or obj_y > 350) and 50 < basket_distance :
                balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))
        
        balls.sort(key= lambda x: x.size)

        return balls

    def analyze_baskets(self, t_basket, depth_frame, debug_color = (0, 255, 255)) -> list:
        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        

        baskets = []
        for contour in contours:

            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            try:
                
                distance_arr = depth_frame[obj_y-10:obj_y+10, obj_x-10:obj_x+10]
                obj_dst = np.average(distance_arr) / 10
            except:
                obj_dst = depth_frame[obj_y, obj_x] / 10

            baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        baskets.sort(key= lambda x: x.size)

        basket = next(iter(baskets), Object(exists = False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

        return basket

    # def analyze_lines(self, t_lines, fragmented, depth_frame, color_nr, debug_color = (255, 255, 255)) -> list:
        
    #     t_lines[fragmented != color_nr] = 0
    #     t_lines[fragmented == color_nr] = 1


    #     contours, hierarchy = cv2.findContours(t_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     lines = []
    #     for contour in contours:

    #         # line filtering logic goes here. Example includes size filtering of the basket

    #         size = cv2.contourArea(contour)
    #         if size < 20:
    #             continue

    #         x, y, w, h = cv2.boundingRect(contour)

    #         obj_x = int(x + (w/2))
    #         obj_y = int(y + (h/2))
    #         obj_dst = depth_frame[obj_y, obj_x]

    #         if (obj_x > 394 and obj_x < 454) and obj_y < 440:

    #             lines.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

    #             if self.debug:
    #                 line = lines[-1]
    #                 if line.exists:
    #                     cv2.circle(self.debug_frame,(line.x, line.y), 15, debug_color, -1)

    #     lines.sort(key= lambda x: x.y)

    #     if len(lines) != 0:
    #         self.closest_line = lines[-1]

    #     return self.closest_line

    # def line_detection(self, colour_frame):
    #     cv2.namedWindow("jooned")
    #     low_threshold = 0
    #     ratio = 5
    #     kernel_size = 5

    #     gray_sacle = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2GRAY)

    #     blurred_image = cv2.blur(gray_sacle, (3,3))
    #     detected_edges = cv2.Canny(blurred_image, low_threshold, low_threshold*ratio, kernel_size)
       
    #     lines = cv2.HoughLines(detected_edges, 1, np.pi/180, 240)

    #     mask = lines != 0 #enne oli dedect_edges
    #     dst = colour_frame * (mask[:,:,None].astype(colour_frame.dtype))
    #     cv2.imshow("jooned", dst)

    def analyze_line(self, line):
        
        value = True

        np.trim_zeros(line)

        colours = color_sequence(line)

        #print("FILTERED LINE: ", colours)

        if len(colours) == 0:
            value = False
        else:
            value = is_inside(colours)

        return value


    def get_frame_data(self, aligned_depth = False):
        if self.camera.has_depth_capability():
            #depth_capability on defaulti peal true
            #tagastab color_farme ja depth_frame
            return self.camera.get_frames(aligned = aligned_depth)
        else:
            return self.camera.get_color_frame(), np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def process_frame(self, aligned_depth = False) -> ProcessedResults:
        color_frame, depth_frame = self.get_frame_data(aligned_depth = aligned_depth)

        segment.segment(color_frame, self.fragmented, self.t_balls, self.t_basket_m, self.t_basket_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)
        
        
        basket_b = self.analyze_baskets(self.t_basket_b, depth_frame, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, depth_frame, debug_color=c.Color.MAGENTA.color.tolist())
        balls = self.analyze_balls(self.t_balls, self.fragmented, basket_b.distance, basket_m.distance)
        


        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame)