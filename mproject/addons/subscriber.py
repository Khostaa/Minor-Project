# import paho.mqtt.client as mqtt

# # Callback when the client connects to the broker
# def on_connect(client, userdata, flags, reason_code, properties):
#     if flags.session_present:
#         print("Session present")
#         # Add code for handling session present
#     if reason_code == 0:
#         client.subscribe("/test")
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

#     # Additional processing based on properties if needed
#     if properties is not None:
#         # Add code for handling properties
#         pass

# # Callback when a message is received from the broker
# def on_message(client, userdata, msg):
#     print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")

# # Create an MQTT client instance
# client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# # Set the callbacks
# client.on_connect = on_connect
# client.on_message = on_message

# # Set the broker address and port
# broker_address = "192.168.137.1"  # You can change this to your MQTT broker's address
# port = 1883

# # Connect to the broker
# client.connect(broker_address, port, 60)

# # Start the MQTT loop
# client.loop_start()

# try:
#     # Keep the script running to receive messages
#     while True:
#         pass

# except KeyboardInterrupt:
#     # Disconnect the client upon keyboard interrupt
#     client.disconnect()
#     print("Disconnected from MQTT broker")
#     client.loop_stop()
import paho.mqtt.client as mqtt

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, reason_code, properties):
    if flags.session_present:
        print("Session present")
        # Add code for handling session present
    if reason_code == 0:
        client.subscribe("/test")
        print("Connected to MQTT broker.")
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

    # Additional processing based on properties if needed
    if properties is not None:
        # Add code for handling properties
        pass

# Callback when a message is received from the broker
def on_message(client, userdata, msg):
    # Check if the received message is related to movement commands
    if msg.topic in ["/forward", "/backward", "/left", "/right", "/stop"]:
        print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")

# Create an MQTT client instance
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Set the callbacks
client.on_connect = on_connect
client.on_message = on_message

# Set the broker address and port
broker_address = "192.168.100.80"  # You can change this to your MQTT broker's address
port = 1883

# Connect to the broker
client.connect(broker_address, port, 60)

# Start the MQTT loop
client.loop_start()

try:
    # Keep the script running to receive messages
    while True:
        pass

except KeyboardInterrupt:
    # Disconnect the client upon keyboard interrupt
    client.disconnect()
    print("Disconnected from MQTT broker")
    client.loop_stop()