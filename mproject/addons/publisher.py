# import paho.mqtt.client as mqtt
# import time

# # Callback when the client connects to the broker
# def on_connect(client, userdata, flags, reason_code, properties):
#     if flags.session_present:
#         print("Session present")
#         # Add code for handling session present
#     if reason_code == 0:
#         print("Connected to MQTT broker")
#         # Add code for successful connection
#     elif reason_code > 0:
#         print(f"Connection failed with reason code {reason_code}")
#         # Add code for handling connection errors
#         if reason_code == 1:
#             print("Unacceptable protocol version")
#         elif reason_code == 2:
#             print("Identifier rejected")
#         elif reason_code == 3:
#             print("Server unavailable")
#         elif reason_code == 4:
#             print("Bad username or password")
#         elif reason_code == 5:
#             print("Not authorized")
#         else:
#             print(f"Unknown reason code: {reason_code}")

# # Create an MQTT client instance
# client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# # Set the on_connect callback
# client.on_connect = on_connect

# # Set the broker address and port
# broker_address = "192.168.6.254"  # You can change this to your MQTT broker's address
# port = 1883

# # Connect to the broker
# client.connect(broker_address, port, 60)

# # Start the MQTT loop
# client.loop_start()

# # Publish a message to the "/test" topic every 5 seconds
# try:
#     while True:
#         message = input("Enter the message you want to send: ")
#         client.publish("/test", message)
#         print(f"Published message: {message} to /test")
        

#         client.publish("/forward", message)
#         print(f"Published message: {message} to /forward")

#         client.publish("/backward", message)
#         print(f"Published message: {message} to /backward")

#         client.publish("/left", message)
#         print(f"Published message: {message} to /left")
        
#         client.publish("/right", message)
#         print(f"Published message: {message} to /right")

#         client.publish("/stop", message)
#         print(f"Published message: {message} to /stop")

# except KeyboardInterrupt:
#     # Disconnect the client upon keyboard interrupt
#     client.disconnect()
#     print("Disconnected from MQTT broker")
#     client.loop_stop()

import detect
import paho.mqtt.client as mqtt
import time

# Callback when the client connects to the broker
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

# Create an MQTT client instance
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set the on_connect callback
client.on_connect = on_connect

# Set the broker address and port
broker_address = "192.168.10.66"  # You can change this to your MQTT broker's address
port = 1883

# Connect to the broker
client.connect(broker_address, port, 60)

# Start the MQTT loop
client.loop_start()

# Topics and messages to be published
topic_messages = [
    ("/forward", "forward"),
    # ("/stop", "stop")
    # ("/left", "Moving left"),
    # ("/right", "Moving right"),
    # ("/stop", "Stop moving")
]

try:
    # Loop through the topics and messages, and publish them serially
    for topic, message in topic_messages:
        while True:
            client.publish(topic, message)
            print(f"Published message: {message} to {topic}")
            time.sleep(10)  # Wait for 5 seconds between publishing messages

except KeyboardInterrupt:
    # Disconnect the client upon keyboard interrupt
    client.disconnect()
    print("Disconnected from MQTT broker")
    client.loop_stop()
