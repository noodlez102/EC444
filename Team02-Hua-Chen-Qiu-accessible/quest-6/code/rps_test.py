import numpy as np
import tensorflow as tf
import cv2
import socket
import pickle
import struct

IMAGE_HEIGHT, IMAGE_WIDTH, CHANNELS = 480, 640, 3  # Adjust based on your image size

# Load the saved model
model = tf.keras.models.load_model('rps.h5')

# Set up socket connection
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
print('HOST IP:', host_ip)
port = 9999
socket_address = (host_ip, port)
server_socket.bind(socket_address)
server_socket.listen(5)
print("LISTENING AT:", socket_address)

# Accept a single connection from the Raspberry Pi
client_socket, addr = server_socket.accept()
print('Got connection from', addr)

# Data buffer to store received frames
data = b""
payload_size = struct.calcsize("I")

while True:
    while len(data) < payload_size:
        packet = client_socket.recv(4 * 1024)  # 4K
        if not packet:
            print("No data received, closing connection.")
            break
        data += packet
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    try:
        msg_size = struct.unpack("I", packed_msg_size)[0]
    except struct.error as e:
        print("Error unpacking message size:", e)
        continue
    while len(data) < msg_size:
        data += client_socket.recv(4 * 1024)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    frame = pickle.loads(frame_data)
    resized_frame = cv2.resize(frame, (300, 200))  # Resize to match the input shape of your model
    processed_frame = resized_frame
    cv2.imshow('Processed Frame', processed_frame)

    # Add a batch dimension since the model expects batches of images
    processed_frame = np.expand_dims(processed_frame, axis=0)

    # Make predictions and send them to the Node.js server
    predictions = model.predict(processed_frame)
    predicted_class = np.argmax(predictions)
    classes = ['rock', 'paper', 'scissors']
    classes2 = ['1', '2', '3']
    predicted_label = classes[predicted_class]
    predicted_label2 = classes2[predicted_class]
    print("Predicted label:", predicted_label)

    # Send the prediction to the Node.js server
    predictions_str = predicted_label2

    HOST = '127.0.0.1'  # Change to the IP address of your Node.js server
    PORT = 3334  # Change to the desired port number
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(predictions_str.encode('utf-8'))

    if cv2.waitKey(1) == 13:
        break

# Clean up
cv2.destroyAllWindows()
client_socket.close()
server_socket.close()
print("Server closed.")