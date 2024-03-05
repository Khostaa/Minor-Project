import cv2
import numpy as np
import threading
import time

class ObjectDetector:
    def __init__(self, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.detected_shape = None
        self.detected_contour = None
        self.lock = threading.Lock()

    def detect_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True, cv2.contourArea(contours[0]), contours[0]
        return False, None, None

    def detect_object(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]  # Filter small contours
        shape = None
        if filtered_contours:
            contour = max(filtered_contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) == 3:
                shape = "triangle"
            elif len(approx) == 4:
                shape = "rectangle"
            else:
                shape = "circle"
            return shape, contour
        return None, None

    def update(self, frame):
        shape, contour = self.detect_object(frame)
        with self.lock:
            self.detected_shape = shape
            self.detected_contour = contour

    def get_detected_object(self):
        with self.lock:
            return self.detected_shape, self.detected_contour

# Define the color range for the object (adjust as needed for different lighting conditions)
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])

# Create object detector
detector = ObjectDetector(lower_red, upper_red)

# Capture video from the camera
cap = cv2.VideoCapture(0)

def detection_thread():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        detector.update(frame)

# Start detection thread
detection_thread = threading.Thread(target=detection_thread)
detection_thread.daemon = True
detection_thread.start()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    shape, contour = detector.get_detected_object()

    # Draw contours
    if contour is not None:
        if shape == "circle":
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
        else:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the information
    if shape:
        cv2.putText(frame, f"Shape: {shape}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Shape: Not Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Object Detection", frame)
    if cv2.waitKey(1) == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
