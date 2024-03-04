import cv2 as cv
import numpy as np



def find_object(image,tuning_params):
    blur = 2
    x_min = tuning_params["x_min"]
    y_min = tuning_params["y_min"]
    x_max = tuning_params["x_max"]
    y_max = tuning_params["y_max"]
    
    search_window = [x_min, y_min, x_max, y_max]

    working_image    = cv.blur(image, (blur, blur))
    # cv.imshow('blur_image', working_image)
    if search_window is None: search_window = [0.0, 0.0, 1.0, 1.0]
    search_window_px = convert_rect_perc_to_pixels(search_window, image)
    #- Convert image from BGR to HSV
    working_image     = cv.cvtColor(working_image, cv.COLOR_BGR2HSV)  
    #- Apply HSV threshold
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    sz_min = tuning_params["sz_min"]
    sz_max = tuning_params["sz_max"]
    keypoints_normalized,image,tuining_image = object_detection(working_image,image,thresh_min,thresh_max,sz_min,sz_max)
        # object_detection(working_image,image,tuning_params,thresh_min,thresh_max,search_window_px)
    
    return keypoints_normalized,image,tuining_image
    
    
def normalise_keypoint(cv_image, kp):
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (kp[0] - center_x)/(center_x)
    y = (kp[1] - center_y)/(center_y)
    return cv.KeyPoint(x, y, kp[2]/cv_image.shape[1])

    
def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    
    # x_min_px    = int(cols*window_adim[0])
    # y_min_px    = int(rows*window_adim[1])
    # x_max_px    = int(cols*window_adim[2])
    # y_max_px    = int(rows*window_adim[3]) 
    return [int(a*b/100) for a,b in zip(rect_perc, scale)]

def draw_window2(image,              #- Input image
                rect_px,        #- window in adimensional units
                color=(255,0,0),    #- line's color
                line=5,             #- line's thickness
               ):
    
    #-- Draw a rectangle from top left to bottom right corner

    return cv.rectangle(image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)



# upper_frame = [0,240,0,640]
# lower_frame = [240,480,0,640]
# color_range = [np.array([96,100,100]),np.array([116,255,255])]

def detect_object(frame,frame_size,color_range):
    sliced_frame = frame[frame_size[0]:frame_size[1],frame_size[2]:frame_size[3]]
    sliced_hsv_frame = cv.cvtColor(sliced_frame,cv.COLOR_BGR2HSV)
    cv.imshow("hsv_frame",sliced_hsv_frame)
    mask = cv.inRange(sliced_hsv_frame,color_range[0],color_range[1])
    filter = cv.bitwise_and(sliced_hsv_frame,sliced_hsv_frame,mask = mask)
    cv.imshow('filter',filter)

def get_tuning_params():
    trackbar_names_tuning = ["x_min","x_max","y_min","y_max"]
    trackbar_names_objects = ["h_min","h_max","s_min","s_max","v_min","v_max","sz_min","sz_max"]
    params = {}
    for key in trackbar_names_tuning:
        params[key] = cv.getTrackbarPos(key, "Tuning")
    
    # for i in range(objects):
    window_name = "Object"
    for key in trackbar_names_objects:
        params[key] = cv.getTrackbarPos(key, window_name)
    
    return params


def wait_on_gui():
    cv.waitKey(2)

def create_tuning_window(initial_values):
    cv.namedWindow("Tuning", cv.WINDOW_NORMAL)
    cv.createTrackbar("x_min","Tuning",initial_values['x_min'],100,no_op)
    cv.createTrackbar("x_max","Tuning",initial_values['x_max'],100,no_op)
    cv.createTrackbar("y_min","Tuning",initial_values['y_min'],100,no_op)
    cv.createTrackbar("y_max","Tuning",initial_values['y_max'],100,no_op)
    
    window_name= "Object"
    cv.namedWindow(window_name, cv.WINDOW_NORMAL)
    cv.createTrackbar("h_min",window_name,initial_values['h_min'],180,no_op)
    cv.createTrackbar("h_max",window_name,initial_values['h_max'],180,no_op)
    cv.createTrackbar("s_min",window_name,initial_values['s_min'],255,no_op)
    cv.createTrackbar("s_max",window_name,initial_values['s_max'],255,no_op)
    cv.createTrackbar("v_min",window_name,initial_values['v_min'],255,no_op)
    cv.createTrackbar("v_max",window_name,initial_values['v_max'],255,no_op)
    cv.createTrackbar("sz_min",window_name,initial_values['sz_min'],100,no_op)
    cv.createTrackbar("sz_max",window_name,initial_values['sz_max'],100,no_op)



def no_op(x):
    pass


def object_detection(working_image,image,thresh_min, thresh_max,sz_min,sz_max):
    working_image_1    = cv.inRange(working_image, thresh_min, thresh_max)
    # Dilate and Erode
    working_image_1 = cv.dilate(working_image_1, None, iterations=2)
    working_image_1 = cv.erode(working_image_1, None, iterations=2)
    
    tuning_image_1 = cv.bitwise_and(image,image,mask = working_image_1)
    #removing noise 
    
    # Invert the image to suit the blob detector
    working_image_1 = 255-working_image_1
    #Morphology EX
    kernel = np.ones((3,3),np.uint8)
    working_image_1 = cv.morphologyEx(working_image_1, cv.MORPH_OPEN, kernel, iterations=1)
    # working_image = cv.morphologyEx(working_image, cv.MORPH_OPEN, kernel)
    kernel = np.ones((4,4),np.uint8)
    working_image_1 = cv.morphologyEx(working_image_1, cv.MORPH_CLOSE, kernel)
    cv.imshow('working_image',working_image_1)
    # Set up the SimpleBlobdetector with default parameters.
    # params = cv.SimpleBlobDetector_Params()
    # params.filterByCircularity = True
    # params.minCircularity = 0
    # params.maxCircularity = 1
    
    # # Change thresholds
    # params.minThreshold = 0
    # params.maxThreshold = 1000
        
    # # # Filter by Area.
    # params.filterByArea = True
    # params.minArea = 30
    # params.maxArea = 2000000
    # detector = cv.SimpleBlobDetector_create(params)
    
    # detector = cv.SimpleBlobDetector_create(params)

    # Run detection!
    # cv.imshow('working_image',working_image)
    canny = cv.Canny(working_image_1,100,150)
    contours, hierarchies  = cv.findContours(canny, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    size_min_px = sz_min*100
    size_max_px = sz_max*10000
    # cv.imshow("Canny",canny)
    cnt = []
    area = [None]*len(contours)
    for i in range(len(contours)):
        area[i] = cv.contourArea(contours[i])
        if area[i] > size_min_px and area[i] < size_max_px:
            print(area[i])
            cnt.append(contours[i])
    keypoints = [None]*len(cnt)
    for i in range(len(cnt)):    
        x,y,w,h = cv.boundingRect(cnt[i])
        cv.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
        keypoints[i] = [(x+w)/2,(y+h)/2,h]
    # for i, c in enumerate(contours):
    #     contours_poly[i] = cv.approxPolyDP(c, 3, True)
    #     boundRect[i] = cv.minAreaRect(contours_poly[i])
    #     area[i] = cv.contourArea(contours_poly[i])
    #     text = cv.putText(image, str(area[i]), (int(boundRect[i][0][0]), int(boundRect[i][0][1])), cv.FONT_HERSHEY_SIMPLEX, 0.5, (45, 255, 166), 1)
    # for i in range(len(contours)):
    #     box = cv.boxPoints(boundRect[i])
    #     box = np.int0(box)
    #     print(area[i])
    #     if area[i]>4000:
    #         cv.drawContours(image,[box],0,(0,0,255),2)
    #         cv.putText(image,str(area[i]),(int(boundRect[i]),int(box[0][1]/2)),cv.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
    #         print(contours_poly[i])
        # cv.rectangle(image, (int(boundRect[i][0]), int(boundRect[i][1])),(int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), (0,255,0), 2)
    #     else:
    #         cv.drawContours(image, [box], 0, (0,255,0), 2)
    #         # cv.putText(img,str(area[i]),(int(box[0][1]/2),int(box[0][1]/2)),cv.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
    cv.imshow("Rectangle",image)
        
    # keypoints = detector.detect(working_image_1)

    # size_min_px = tuning_params['sz_min']*working_image_1.shape[1]/100.0
    # size_max_px = tuning_params['sz_max']*working_image_1.shape[1]/100.0

    # keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]
    
    
    # # Set up main output image
    # line_color=(0,255,0)

    # out_image = cv.drawKeypoints(image, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # out_image = draw_window2(out_image, search_window_px)

    # # Set up tuning output image
    
    # tuning_image_1 = cv.drawKeypoints(tuning_image_1, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # # tuning_image = draw_window(tuning_image, search_window)
    # # cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
    # tuning_image_1 = draw_window2(tuning_image_1, search_window_px)
    # # cv.imshow('out_image',out_image)
    # # cv.imshow('tuining_image',tuning_image)
    # # print(k for k in keypoints if k.pt[0]>0)
    

    keypoints_normalised = [normalise_keypoint(working_image_1, keypoints[k]) for k in range(len(keypoints))]
    
    # print(keypoints_normalised[0])

    return keypoints_normalised, image, tuning_image_1