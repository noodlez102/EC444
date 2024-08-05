import speech_recognition as sr
import pyttsx3
import pyaudio
import socket 
import time
tts = pyttsx3.init()
esp_ip = '0.0.0.0'
esp_port = 3000

r = sr.Recognizer()
message = "Hello from Python"
server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the local IP address and port
server_sock.bind((esp_ip, esp_port))
data, addr = server_sock.recvfrom(1024)  # Buffer size is 1024 bytes
print("Received message:", data.decode(), "from", addr)
server_sock.sendto(message.encode(),addr)
text = message
# Receive data from the ESP device
while True:
    data, addr = server_sock.recvfrom(1024)  # Buffer size is 1024 bytes
    print("Received message:", data.decode(), "from", addr)
    server_sock.sendto(text.encode(),addr)

    with sr.Microphone() as mic:
        r.adjust_for_ambient_noise(mic,duration=0.2)
        print("listening")
        audio = r.listen(mic,phrase_time_limit=4)
        print("translating")

    try:
        
        
        text = r.recognize_google(audio)
        
        text = text.lower()
        print(f"recognized {text}")
        if ("left" in text):
            text = "left"
        elif ("right" in text):
            text = "right"
        if ("slow" in text):
            text = "slow"
        if ("fast" in text):
            text = "fast"
        if ("straight" in text):
            text = "straight"    
        if ("stop" in text):
            text = "stop"

        tts.say(text)
        tts.runAndWait()

    except sr.UnknownValueError:
        r = sr.Recognizer()
        continue
    except sr.RequestError as e:
        r = sr.Recognizer()
        continue





