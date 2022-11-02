import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c


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
                debug_frame = [],
                lines_w = Object(exists = False), 
                lines_b = Object(exists = False)) -> None: #lisada jooned


        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented
        self.lines_w = lines_w
        self.lines_b = lines_b
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
        self.t_lines_w = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)
        self.t_lines_b = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

        self.debug = debug
        self.debug_frame = np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, fragments) -> list:
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

        
        
        balls = []
        
        for contour in dilated_contours:
        #for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the fram to the ball

            size = cv2.contourArea(contour)

            if size < 15:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            ys	= np.array(np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
            xs	= np.array(np.linspace(x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = obj_y

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame,(obj_x, obj_y), 10, (0,255,0), 2)

            balls.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))
        #[Object: x=7; y=212; size=15.0; distance=212; exists=True]
        
        balls.sort(key= lambda x: x.size)

        return balls

    def analyze_baskets(self, t_basket, debug_color = (0, 255, 255)) -> list:
        contours, hierarchy = cv2.findContours(t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("BASKET CONTURES: ",contours)

        baskets = []
        for contour in contours:

            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = self.camera.distance(obj_x, obj_y)

            baskets.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        baskets.sort(key= lambda x: x.size)

        basket = next(iter(baskets), Object(exists = False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame,(basket.x, basket.y), 20, debug_color, -1)

        return basket

    def analyze_lines(self, t_lines, debug_color = (255, 255, 255)) -> list:
        contours, hierarchy = cv2.findContours(t_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("contours:", contours)
        lines = []
        for contour in contours:

            # line filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)
            print("SUURUS ON: ", size)
            if size < 10:
                print("pole pixleid")
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            obj_dst = self.camera.distance(obj_x, obj_y)

            lines.append(Object(x = obj_x, y = obj_y, size = size, distance = obj_dst, exists = True))

        lines.sort(key= lambda x: x.size)

        line = next(iter(lines), Object(exists = False))

        if self.debug:
            if line.exists:
                cv2.circle(self.debug_frame,(line.x, line.y), 20, debug_color, -1)

        return line

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
        #segment.segment(self.t_lines_w, self.t_lines_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)

        balls = self.analyze_balls(self.t_balls, self.fragmented)
        basket_b = self.analyze_baskets(self.t_basket_b, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(self.t_basket_m, debug_color=c.Color.MAGENTA.color.tolist())
        lines_w = self.analyze_lines(self.t_lines_w, debug_color=c.Color.WHITE.color.tolist())
        lines_b = self.analyze_lines(self.t_lines_b, debug_color=c.Color.BLACK.color.tolist())
        

        return ProcessedResults(balls = balls, 
                                basket_b = basket_b, 
                                basket_m = basket_m, 
                                color_frame=color_frame, 
                                depth_frame=depth_frame, 
                                fragmented=self.fragmented, 
                                debug_frame=self.debug_frame,
                                lines_w = lines_w, 
                                lines_b = lines_b)#lines eemaldada kui katki