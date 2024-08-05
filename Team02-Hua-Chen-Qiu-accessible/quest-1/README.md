# Four legged Walker

Authors: Sam Qiu, Kevin Chen, Arthur Hua

Date: 2024-08-02

### Summary

In this quest we created a four legged walker. We wired a 14 segement display to show the countdown to the next walk. There are 3 buttons that controls the timer. One button to control the mode ie. change to hour, minutes, or seconds, and the other two to increase or decrease the time until the next walk.

### Self-Assessment

| Objective Criterion                                                                               | Rating | Max Value |
| ------------------------------------------------------------------------------------------------- | :----: | :-------: |
| Servo spins left then right three times without chatter at prescribed time intervals              |   1    |     1     |
| Alphanumeric display indicates hours and minutes                                                  |   1    |     1     |
| Display shows countdown time report every second with no time loss                                |   1    |     1     |
| Demo delivered at scheduled time and report submitted in team folder with all required components |   1    |     1     |
| Investigative question response                                                                   |   1    |     1     |

### Solution Design

For the physical design of the walker, originally the legs were all even, so when it "walked," it actually didn't move anywhere, but by making the head of our walker felxible, and the front 2 legs taller than the back it can move forward. We added pulldown resistors for the buttons, and we used the ISR to keep track of time, and RTOS to have 5 different tasks. 3 tasks for the buttons, 1 task for the display, 1 task for the timer. It was important to add the pull down resistors to the button because without it the buttons would keep activating. We used global flags so each task can interact with each other.

### Sketches/Diagrams

![IMG_0852](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/assets/47343227/73471fbb-4786-48ff-b942-4303c39f4807)
4 legged Walker Circuit
![IMG_0852](https://github.com/BU-EC444/Team02-Hua-Chen-Qiu/blob/main/quest-1/images/walker.png)
4 legged walker with two servos for articulation

### Supporting Artifacts

- [Link to video](https://youtu.be/ksF4FBvhhb4?si=9mF30gtiMO_ACXhK).

### Modules, Tools, Source Used Including Attribution

- [Frame work for Timer code taken](https://github.com/BU-EC444/04-Code-Examples/tree/main/timer-example-new)
- [Frame work for i2c display code](https://github.com/BU-EC444/04-Code-Examples/tree/main/i2c-display)
- [Adafruit display cheatbitmap](https://github.com/adafruit/Adafruit_LED_Backpack/blob/master/Adafruit_LEDBackpack.cpp)
