# April 17, 2026
Things on the dashboard:
- Turning angle of the wheels
- Speed of the car (Acceleration and deceleration like in Forza)
- Lidar visual (blue and opaque blue)

-  Have it show up with the simulation as well as real life testing

## Questions
1. Where do we access each of these data from in  the software

-  For now, we assume it's one-to-one with gazebo and we use the same as the physical model
-  Later, ask Robbie and confirm if it is 1-to-1 
- Lidar - /scan
- Velocity & Angle 
    - Input from algorithms are in DriveParmas from the /input/drive_param/autonomous
    - 
2. Which physical component of the car gets me this data?

- Servo controls turning angle
- wheel motor control the speed


3. How to save them onto a text file


4. What should be the required buffer length?


