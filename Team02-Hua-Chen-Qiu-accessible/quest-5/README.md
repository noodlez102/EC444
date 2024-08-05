# AI Driver

Authors: Sam Qiu, Kevin Chen, Arthur Hua

Date: 2024-04-23

### Summary


In our project, we applied our acquired skills to integrate a LiDAR and ESP with our Buggy car. The LiDAR enables us to gauge distances to obstacles ahead. Additionally, we incorporated a camera connected to a Raspberry Pi Zero positioned at the car's front to capture images. Through a manual control program, we steered the car multiple times to gather data from the camera, sensors, and our manual interventions. These datasets were transmitted via UDP transactions to a server, where they were stored in a database. Subsequently, we utilized this data to train a model to emulate human interventions and drive the car. This trained model was then employed to autonomously navigate the car by transmitting its output to the vehicle. Furthermore, we established a separate website to live stream video footage captured by the PiCam.

### Self-Assessment

| Objective Criterion                             | Rating | Max Value |
| ----------------------------------------------- | :----: | :-------: |
| Stops for obstacles (cars, pedestrians)         |   1   |     1     |
| Follows the lane without deviation              |   1   |     1     |
| Turn execution according to direction requested |   1   |     1     |
| Model inference on he device                    |   1   |     1     |
| Investigative Question                          |   1   |     1     |
| Demo point                                      |   1   |     1     |

### Solution Design

We developed an AI-driven car that relied on images from a Raspberry Pi camera and LiDAR to detect potential collisions. To gather data, we initially controlled the car using a Node.js website, assigning labels to define desired states for the car. Subsequently, we conducted testing in a custom-built environment, maneuvering the car through it multiple times to capture data for various actions such as driving straight, turning left or right, detecting obstacles in front, and correcting its course. Using TensorFlow, we trained our model and integrated it back into our website, enabling the car to be controlled autonomously by the trained model.

### Investigative question

Comparing neural network (NN) controllers with traditional Proportional-Integral-Derivative (PID) controllers in car steering, NNs offer advanced pattern recognition and adaptability but are prone to errors in novel scenarios, leading to possible catastrophic failures. And we did experience similar pronblem when testing our model on buggy car. To mitigate these risks, a hybrid approach can be effective. Integrating a PID controller can provide stability and reliability when the NN outputs deviate from expected behaviors. Additionally, enhancing the NN's robustness through data augmentation, adversarial training, and continual learning can help address edge cases and adapt to new conditions, improving overall safety and performance.

### Sketches/Diagrams
![image](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/assets/47343227/6a4ff0ba-db21-4af7-b5f2-a9a29a43fdf0)


### Supporting Artifacts (videos)

- [Practical Demo](https://youtu.be/lfNoCjfIg10?si=S4RRGe_7GaeMibT7). 
- [Technical Demo](https://youtu.be/wVeeY9LFZSo?si=KPDu4W2spJby0brU). 


### Modules, Tools, Source Used Including Attribution

- [TensorFlow.js Installation](https://www.tensorflow.org/js/tutorials/setup)
- [More TensorFlow.js Examples](https://github.com/tensorflow/tfjs-examples)
- [Additional AI resources for ESP](https://github.com/espressif/esp-nn)
- [EloquentTinyML](https://github.com/eloquentarduino/EloquentTinyML)
- [TensorFlow Lite for Microcontrollers](https://www.tensorflow.org/lite/microcontrollers)
- [TensorFlow Lite for esp-idf](https://github.com/espressif/tflite-micro-esp-examples)
