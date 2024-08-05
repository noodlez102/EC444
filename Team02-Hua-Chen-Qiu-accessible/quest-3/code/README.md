# Code Readme

We use the FreeRTOS real-time operating system to manage 6 tasks related to sensor data acquisition  It includes 3 tasks to read data from a thermistor, a voltage divider, and an accelerometer. These sensors provide  data, such as temperature of environment, battery level, and box movement. Additionally, another task controls a LED light based on UDP message. The is a task to communicate with node.js server over UDP protocol. The last task in the system periodically reports sensor readings and packs them into a UDP message string.

This setup could be used in our Node.js application where it reads the data using UDP communication and writes it to the webpage. The data is then visualized using Canvas.js in real-time. The webpage also has a button to control the LED light on the box. The video stream from the PiCam is also displayed on the webpage.

