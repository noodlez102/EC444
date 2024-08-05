import cv2
import socket
import pickle
import struct
import csv
import os
import uuid

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

def generate_unique_filename():
    return str(uuid.uuid4()) + ".jpg"

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
    processed_frame = resized_frame.astype('float32') / 255.0  # Normalize pixel values
    cv2.imshow('Processed Frame', processed_frame)
    
    data_folder = 'test'
        # Processing the received frame (convert to grayscale)
    random_filename = generate_unique_filename()
    frame_filename = os.path.join(data_folder, random_filename)
    cv2.imwrite(frame_filename, resized_frame)
    print("Frame saved successfully as:", frame_filename)
        # Break the loop if the Enter key is pressed
    if cv2.waitKey(1) == 13:
        break

# Clean up
cv2.destroyAllWindows()
client_socket.close()
server_socket.close()
