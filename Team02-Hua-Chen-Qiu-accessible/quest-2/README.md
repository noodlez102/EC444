# Sonar

Authors: Sam Qiu, Kevin Chen, Arthur Hua

Date: 2024-02-28

### Summary

In this quest, we used the what we learned from the skills to wrire up a thermistor, IR, and ultrasonic sensor to get readings from the surrounding enviroment which we then convert into engineering units. The values are then printed into the serial port, and by using node.js we write those values into a file, and also use canvas js to visualize the data. We have 3 seperate graphs, where the first one has both the IR and ultrasonic sensor readings vs angle of the servo, so we can compare the two sensors, another IR sensor graph vs time, and the thermistor vs time graph.

### Self-Assessment

| Objective Criterion                                            | Rating | Max Value |
| -------------------------------------------------------------- | :----: | :-------: |
| Periodic reporting of ultrasonic range in m                    |   1    |     1     |
| Periodic reporting of IR range in m                            |   1    |     1     |
| Periodic reporting of temperature in C                         |   1    |     1     |
| Results displayed at host as text                              |   1    |     1     |
| Results graphed at host continuously based on reporting period |   1    |     1     |
| Demo delivered at scheduled time                               |   1    |     1     |
| Investigative question response                                |   1    |     1     |

### Solution Design

main.c contains the code that handles 5 tasks, which were obtaining the readings from the 3 sensors, the servo rotating, and writing all of the readings in the serial port.

node_handler.js is where we handled getting the data from the serial port and writing it into the a file. This is also where we send the data to the html file to turn into graphs by using socket IO.

graph.html is what we used to visualize our data by using dynamic graphs that update real time. Here we used a scatter plot for the IR & Ultrasonic vs angle graph to somewhat simulate a sonar, a line graph for the thermistor and IR vs time graph.

### Investigative question

After comparing the IR sensor and ultrasonic sensor readings on our Canvas.js graph, we decided that the IR gave better and more accurate readings and we would rather use that for a car. This is because it is more stable and consistance for each detection.

### Sketches/Diagrams

![img](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/blob/main/quest-2/images/graphs.png)
![img](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/blob/main/quest-2/images/hardware.png)

<p align="center">
Canvas.js graph of the data
</p>

### Supporting Artifacts

- [Link to video presentation](https://www.youtube.com/watch?v=AVc8LTiFUbw). Not to exceed 120s

### Modules, Tools, Source Used Including Attribution

- [Canvas Dynamic data Graphs](https://canvasjs.com/javascript-charts/dynamic-live-line-chart/)
- [Node.js getting data from serial and writing to file](https://github.com/BU-EC444/04-Code-Examples/tree/main/serial-esp-to-node-serialport)
- [Code base for adc readings](https://github.com/espressif/esp-idf/tree/39f090a4f1dee4e325f8109d880bf3627034d839/examples/peripherals/adc)
