import paho.mqtt.client as mqtt
import time
from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator

# Defining model
model = YOLO('models/best.pt')
rtmp_url = 'rtmp://192.168.43.15/live/stream'
# rtmp_url=1

# CV2 configs
# cap = cv2.VideoCapture(rtmp_url)
cap = cv2.VideoCapture(rtmp_url)
cap.set(3, 640)  # width of frame
cap.set(4, 480)  # height of frame



# Set the broker address and port
broker_address = "192.168.43.93"  # Change this to your MQTT broker's address
port = 1883

# Flag to check connection status and last message sent
is_connected = False
last_message = None

# Create an MQTT client instance
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, reason_code, properties):
    global is_connected
    if reason_code == 0:
        print("Connected to MQTT broker")
        is_connected = True  # Set the flag to True when connected
    else:
        print(f"Connection failed with reason code {reason_code}")

# Set the on_connect callback
client.on_connect = on_connect

# Connect to the broker
client.connect(broker_address, port, 60)

# Start the MQTT loop
client.loop_start()

# Wait until connected to the broker
while not is_connected:
    print("Waiting for MQTT broker connection...")
    time.sleep(1)  # Wait a bit before checking again

print("Broker connected. Starting YOLO model loop...")

# YOLO model loop
try:
    while True:
        _, img = cap.read()
        results = model.predict(img)
        box_detected = False

        for r in results:
            annotator = Annotator(img)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]
                c = box.cls
                annotator.box_label(b, model.names[int(c)])
                class_label = model.names[int(c)]
                
                if class_label == 'box':
                    box_detected = True
                    if last_message != 'F':
                        client.publish("/forward", "F")
                        print("Published 'F' in forward topic")
                        last_message = 'F'
                    break  # Exit the loop after the first box detection

        if not box_detected and last_message != 'S':
            client.publish("/stop", "S")
            print("Published 'S' in stop topic")
            last_message = 'S'

        img = annotator.result()
        cv2.imshow('YOLO V8 Detection', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()