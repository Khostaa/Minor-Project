import cv2 as cv

capture = cv.VideoCapture('http://192.168.10.86:8080/video')  # Create a VideoCapture object to capture video from camera

while True:
    isTrue, frame = capture.read()
    cv.imshow('Video', frame)

    if cv.waitKey(20) & 0xFF == ord ('d'):
        break
capture.release()
cv.destroyAllWindows()

# import cv2
# import numpy as np
# from ultralytics import YOLO
# #Load YOLO Algorithm
# yolo = YOLO("best.pt")
# #To load all objects that have to be detected
# classes=[]
# with open("Biscuits_box","r") as f:
#     read=f.readlines()
# for i in range(len(read)):
#     classes.append(read[i].strip("\n"))

# #Defining layer names
# layer_names=yolo.getLayerNames()
# output_layers=[]
# for i in yolo.getUnconnectedOutLayers():
#     output_layers.append(layer_names[i[0]-1])
# #Loading the Image
# capture = cv2.VideoCapture(0)  # Create a VideoCapture object to capture video from camera

# while True:
#     isTrue, frame = capture.read()
#     cv2.imshow('Video', frame)

#     if cv2.waitKey(20) & 0xFF == ord ('d'):
#         break
# capture.release()
# cv2.destroyAllWindows()   


# # img=cv2.imread("Road.jpg")
# height,width,channels=capture.shape
# #Extracting features to detect objects
# blob=cv2.dnn.blobFromImage(capture,0.00392,(416,416),(0,0,0),True,crop=False)
#                                                         #Inverting blue with red
#                                                         #bgr->rgb
# #We need to pass the img_blob to the algorithm
# yolo.setInput(blob)
# outs=yolo.forward(output_layers)
# #print(outs)
# #Displaying informations on the screen
# class_ids=[]
# confidences=[]
# boxes=[]
# for output in outs:
#     for detection in output:
#         #Detecting confidence in 3 steps
#         scores=detection[5:]                #1
#         class_id=np.argmax(scores)          #2
#         confidence =scores[class_id]        #3
#         if confidence >0.5: #Means if the object is detected
#             center_x=int(detection[0]*width)
#             center_y=int(detection[1]*height)
#             w=int(detection[2]*width)
#             h=int(detection[3]*height)
#             #Drawing a rectangle
#             x=int(center_x-w/2) # top left value
#             y=int(center_y-h/2) # top left value
#             boxes.append([x,y,w,h])
#             confidences.append(float(confidence))
#             class_ids.append(class_id)
#            #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
# #Removing Double Boxes
# indexes=cv2.dnn.NMSBoxes(boxes,confidences,0.3,0.4)
# for i in range(len(boxes)):
#     if i in indexes:
#         x, y, w, h = boxes[i]
#         label = classes[class_ids[i]]  # name of the objects
#         cv2.rectangle(capture, (x, y), (x + w, y + h), (0, 255, 0), 2)
#         cv2.putText(capture, label, (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
       
# cv2.imshow("Output",capture)
# cv2.waitKey(0)
# cv2.destroyAllWindows()