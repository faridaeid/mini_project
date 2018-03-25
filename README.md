# Embedded Systems Mini Project 
## Arrow following Dagu

The idea is having a moving vehicle moving towards a colored box with an arrow or a circle. If there is a left or right arrow, the vehicle moves 90 degrees accordingly, otherwise it stops

### Prerequisites 
* ROS framework
* Setup for camera (Jetson TX1 Camera used here)


### Main Components

```
Super Master
--> Rotation Controller (Publisher)
--> PID Controller (Publisher)
    --> PID
--> Color Segmentation (Publisher)
--> Motor Controller (Publisher) 
```

Super Node initializes the controllers. Then using a finite state machine it get values the controllers are publishing.

### States
* Move PID 
    * Get center of the color segmented box
    * Send the center to **PID controller**
        * PID controller publish motor values for **Motor Controller**
    * Checks area of segmented box to stop vehicle before the box
* Stop PID
    * Send signal to **PID Controller** to stop
        * PID controller publish brake logic for **Motor Controller**
* Detect Sign
    * Get the status of the detected sign
    * If it was an arrow, then publish rotation angle to **Rotation Controller**
    * If it was a circle, then set the next state to the Stop PID
* Rotate
    * After the rotation angle is set, publish rotate command for **Rotate Controller**
* Finish
