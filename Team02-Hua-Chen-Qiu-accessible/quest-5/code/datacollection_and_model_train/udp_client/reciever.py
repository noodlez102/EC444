import cv2
import socket
import pickle
import struct
import csv
import os

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

def read_first_line_and_write(csv_file, output_file):
    """
    Reads the first line of a CSV file and writes it to a new CSV file.
    
    Args:
    csv_file (str): The path to the input CSV file.
    output_file (str): The path to the output CSV file.
    """
    try:
        with open(csv_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
            reader = csv.reader(infile)
            writer = csv.writer(outfile)
            
            # Attempt to read and write the first line
            try:
                first_line = next(reader)
                writer.writerow(first_line)
                print("First line of the CSV file has been written to", output_file)
            except StopIteration:
                print("The CSV file is empty.")
    except FileNotFoundError:
        print("File not found. Please check the file path.")
    except Exception as e:
        print("An error occurred:", e)



frame_count = 1559
# Open the CSV file to store the dataset
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
    data_folder = 'data_folder'
        # Processing the received frame (convert to grayscale)
    processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Optional: Display the processed frame
    frame_filename = os.path.join(data_folder,f"frame_{frame_count}.jpg")
    cv2.imwrite(frame_filename,processed_frame)
    frame_filename2 = os.path.join(data_folder,f"frame_{frame_count}.csv")

    read_first_line_and_write('messages.csv',frame_filename2)

    frame_count  += 1
        

        # Break the loop if the Enter key is pressed
    if cv2.waitKey(1) == 13:
        break

# Clean up
cv2.destroyAllWindows()
client_socket.close()
server_socket.close()

print("Server and CSV file closed.")
