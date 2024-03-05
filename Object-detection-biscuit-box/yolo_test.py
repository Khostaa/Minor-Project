from ultralytics import YOLO
import cv2
import threading
from ultralytics.utils.plotting import Annotator

# Load YOLO model
model = YOLO('best.pt')

# Camera setup
cap = cv2.VideoCapture('http://192.168.160.74:8080/video')
cap.set(3, 640)
cap.set(4, 480)

# Function for processing frames
def process_frames():
    while True:
        _, img = cap.read()
        if img is None:
            break
        
        # Perform inference
        results = model.predict(img)
        
        # Process results
        for r in results:
            annotator = Annotator(img)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, model.names[int(c)])  # Annotate the box with class label

                # Display the coordinates on the image
                cv2.putText(img, f"({b[0]:.0f}, {b[1]:.0f})", (int(b[0]), int(b[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Display the annotated image
        cv2.imshow('YOLO V8 Detection', img)

        # Exit on spacebar press
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break

# Start the frame processing thread
frame_thread = threading.Thread(target=process_frames)
frame_thread.start()

# Release resources
frame_thread.join()
cap.release()
cv2.destroyAllWindows()
