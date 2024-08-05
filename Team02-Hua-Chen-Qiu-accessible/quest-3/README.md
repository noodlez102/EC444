# Hurricane Box

Authors: Sam Qiu, Kevin Chen, Arthur Hua

Date: 2024-03-22

### Summary

In this quest, we used the what we learned from the skills to wire up a thermistor and an accelerometer to get readings from the surrounding enviroment which we then convert into engineering units. We also implement a voltage divider with ADC to keep track on battery level. And we also wire up a LED for remote control. The values of the thermistor, the accelerometer, and the bettery level then sent to node.js over UDP transactions, where we write those values into a file, and also use canvas js to visualize the data in HTML website. We also have a button on the website to trigger the LED using UDP protocol. We also host another website to stream the video from picam.

### Self-Assessment

| Objective Criterion                                                                                                             | Rating | Max Value |
| ------------------------------------------------------------------------------------------------------------------------------- | :----: | :-------: |
| Measures acceleration, temperature, and battery level                                                                           |   1   |     1     |
| Displays real-time data (temperature, vibration, battery level) at remote client via portal using separate IP network.         |   1   |     1     |
| Controls LED on box from remote client via portal.                                                                              |   1   |     1     |
| Sources web cam video into remote client.                                                                                       |   1   |     1     |
| ESP32 and Rpi are connected wirelessly to router; ESP32 sensor data are delivered to local node server (on local laptop or Rpi) |   1   |     1     |
| Demo delivered at scheduled time and report submitted in team folder with all required components                               |   1   |     1     |
| Investigative question response                                                                                                 |   1   |     1     |

### Solution Design

main.c contains the code that handles 6 tasks, which were obtaining the readings from the sensors, composing reading in proper format, and sending the result to server over UDP.

server.js is where we handled getting the data from the UDP communication. This is also where we send the data to the html file to turn into graphs by using socket IO.

index.html is what we used to visualize our data that update real time, as well as controlling the LED using online button.

### Investigative question

To enhance the power efficiency of the Hurricane Box, several strategies can be implemented across its components. These include optimizing sensor readings by adjusting sampling rates and utilizing low-power modes, employing efficient data transmission protocols like UDP, and optimizing the web portal and remote access functionalities to reduce processing overhead.

### Sketches/Diagrams
![image](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/assets/47343227/c35f6d90-3396-4db1-bade-d448689aa775)
![image](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/assets/47343227/9b20554e-8fab-42b6-83b7-e1d8d4a1fd34)
![image](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/assets/47343227/636db40c-0590-4f4a-8136-afea29e273ac)

<p align="center">
<img src="./images/ece444.png" width="50%">
</p>
<p align="center">
Caption Here
</p>

### Supporting Artifacts

- [Link to video demo](https://youtu.be/zyKBpXfk-p8). Not to exceed 120s

### Modules, Tools, Source Used Including Attribution

- [Recipe for bringing up the pi-cam](/docs/recipes/docs/video.md)
- [ESP32 Pin Layout](https://learn.adafruit.com/assets/111179)
- [I2C Design Pattern](https://github.com/BU-EC444/01-Ebook-S2024/blob/main/docs/design-patterns/docs/dp-i2c.md)
- [ADXL343 Datasheet](https://cdn-learn.adafruit.com/assets/assets/000/070/556/original/adxl343.pdf?1549287964)
- [Calculate the angle of the accelerometer](https://www.nxp.com/docs/en/application-note/AN3461.pdf)
- [Thermistor Conversion Equation](https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html)
- [Thermistor Datasheet](https://www.eaa.net.au/PDF/Hitech/MF52type.pdf)
