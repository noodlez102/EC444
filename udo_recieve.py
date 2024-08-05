import cv2
import numpy as np
import socket
import io

# UDP settings
UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

try:
    while True:
        # Receive image bytes
        image_bytes = b""
        while True:
            data, addr = sock.recvfrom(4096)
            if not data:
                break
            image_bytes += data
        
        # Convert bytes to numpy array
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        
        # Decode JPEG image
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect edges using Canny edge detection
        edges = cv2.Canny(gray, 100, 200)
        
        # Display image with edges
        cv2.imshow('Edges', edges)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Close UDP socket
    sock.close()
    cv2.destroyAllWindows()
