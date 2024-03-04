# import threading
# from ultralytics import YOLO
# import cv2
# from ultralytics.utils.plotting import Annotator

# # Function to capture frames and perform object detection
# def detect_objects(model, rtmp_url):
#     cap = cv2.VideoCapture(rtmp_url)
#     cap.set(3, 640) # width of frame
#     cap.set(4, 480) # height of frame

#     while True:
#         _, img = cap.read() # reads a frame from the webcam and store it in the variables
        
#         # performs object detection on the captured frame using the YOLO model.
#         # returns a list of detection results, where each detection contains info. about the bounding box and class label of detected objects.
#         results = model.predict(img)

#         for r in results: # iterate over detection results
#             annotator = Annotator(img) # creates an annotator object to annotate the image with the bounding boxes
#             boxes = r.boxes
#             for box in boxes:
#                 b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
#                 c = box.cls #  get class label
#                 annotator.box_label(b, model.names[int(c)])  # Annotate the box with class label

#                 # Display the coordinates on the image
#                 cv2.putText(img, f"({b[0]:.0f}, {b[1]:.0f})", (int(b[0]), int(b[1]) - 10),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

#         img = annotator.result()
#         cv2.imshow('YOLO V8 Detection', img)

#         if cv2.waitKey(1) & 0xFF == ord(' '):
#             break
    
#     # release the video capture object, closing the connection to webcam
#     cap.release()
#     cv2.destroyAllWindows()

# # Create YOLO model instance
# model = YOLO('models/best.pt')

# # Define RTMP URL
# rtmp_url = 'rtmp://192.168.137.20/live/stream'

# # Start a new thread for object detection
# thread = threading.Thread(target=detect_objects, args=(model, rtmp_url))
# thread.start()

# # Wait for the thread to finish
# thread.join()
import cv2

# Replace the URL with your actual RTMP stream URL
rtmp_url = 'rtmp://192.168.43.15/live/stream'
# Open the RTMP stream
cap = cv2.VideoCapture(rtmp_url)

if not cap.isOpened():
    print("Error: Could not open stream.")
else:
    print("Stream opened successfully.")

# Read and display the stream
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Press Q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()