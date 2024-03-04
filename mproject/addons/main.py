from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator
import paho.mqtt.client as mqtt
import time

# importing custom YOLOv8 mode
model = YOLO('models/best.pt')
def on_connect(client, userdata, flags, reason_code, properties):
    if flags.session_present:
        print("Session present")
        # Add code for handling session present
    if reason_code == 0:
        print("Connected to MQTT broker")
        # Add code for successful connection
    elif reason_code > 0:
        print(f"Connection failed with reason code {reason_code}")
        # Add code for handling connection errors
        if reason_code == 1:
            print("Unacceptable protocol version")
        elif reason_code == 2:
            print("Identifier rejected")
        elif reason_code == 3:
            print("Server unavailable")
        elif reason_code == 4:
            print("Bad username or password")
        elif reason_code == 5:
            print("Not authorized")
        else:
            print(f"Unknown reason code: {reason_code}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set the on_connect callback
client.on_connect = on_connect

# Set the broker address and port
broker_address = "192.168.137.1"  # You can change this to your MQTT broker's address
port = 1883

# Connect to the broker
client.connect(broker_address, port, 60)

# Start the MQTT loop
client.loop_start()

# Topics and messages to be published

# rtmp_url = 'rtmp://192.168.137.142/live/stream'
# opens a connection of default webcam (index 0)
# cap = cv2.VideoCapture(rtmp_url)
cap = cv2.VideoCapture(0)
cap.set(3, 640) # width of frame
cap.set(4, 480) # height of frame

while True:
    _, img = cap.read() # reads a frame from the webcam and store it in the variables
    
    # performs object detection on the captured frame using the YOLO model.
    # returns a list of detection results, where each detection contains info. about the bounding box and class label of detected objects.
    results = model.predict(img)
    # print(type(results))
    for r in results: # iterate over detection results
        import pdb
        pdb.set_trace()
        annotator = Annotator(img) # creates an annotator object to annotate the image with the bounding boxes
        boxes = r.boxes
        # print(boxes)
        for box in boxes:
            # print(box)
            client.publish("/robot", "F")
            print(f"Published message:F to /robot")
            b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
            c = box.cls #  get class label
            class_label = model.names[int(c)]
            annotator.box_label(b, model.names[int(c)])  # Annotate the box with class label

            # Display the coordinates on the image
            cv2.putText(img, f"({b[0]:.0f}, {b[1]:.0f})", (int(b[0]), int(b[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            print(f"Detected Object: {class_label}")
    img = annotator.result()
    cv2.imshow('YOLO V8 Detection', img)
    # cv2.imshow('rtmp_url', img)
    # print(results)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
# release the video capture object, closing the connection to webcam
cap.release()
# closes all OpenCV windows created during the execution of the program
cv2.destroyAllWindows()



# from ultralytics import YOLO
# import cv2
# from ultralytics.utils.plotting import Annotator
# import threading
# import queue

# # importing custom YOLOv8 mode
# model = YOLO('models/best.pt')

# # rtmp_url = 'rtmp://192.168.137.142/live/stream'
# # opens a connection of default webcam (index 0)
# # cap = cv2.VideoCapture(rtmp_url)
# cap = cv2.VideoCapture(0)
# cap.set(3, 640) # width of frame
# cap.set(4, 480) # height of frame

# # Create a queue to hold the frames
# frame_queue = queue.Queue()

# # Create a queue to hold the results
# results_queue = queue.Queue()

# # Function to perform object detection
# def detect_objects():
#     while True:
#         # Get a frame from the queue
#         img = frame_queue.get()

#         # Perform object detection
#         results = model.predict(img)

#         # Put the results in the results queue
#         results_queue.put(results)

# # Start the object detection thread
# threading.Thread(target=detect_objects, daemon=True).start()

# while True:
#     _, img = cap.read() # reads a frame from the webcam and store it in the variables

#     # Put the frame in the queue
#     frame_queue.put(img)

#     # Check if there are any results in the results queue
#     if not results_queue.empty():
#         results = results_queue.get()

#         for r in results: # iterate over detection results
#             print(r)
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

#     if cv2.waitKey(1) & 0xFF == ord(' '):
#         break

# # release the video capture object, closing the connection to webcam
# cap.release()
# # closes all OpenCV windows created during the execution of the program
# cv2.destroyAllWindows()
