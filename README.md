
# DIY 3D-Printed Linear Servo Motor

This repository summarizes my DIY 3D-printed linear servo motor.

![Image](pic/linear-servo-motor.jpg)

## Youtube Video

[![alt text](http://img.youtube.com/vi/ndlJ5Nt8iBA/0.jpg)](https://www.youtube.com/watch?v=ndlJ5Nt8iBA "title")

## Hardware Description

- Coil Specifications
  - Wire: 0.4 mm copper
  - Dimensions (W x L x H): 13 x 20 x 16.5 mm
  - Core dimensions: 3 × 10 × 16.5 mm (stack of three 1 mm iron plates)
  - Resistance: 2.2 Ω
  - Wire length: approximately 17 m
  - Number of turns: approximately 360

![Image](pic/make-coil-12.jpg)

- Magnet Specifications
  - Dimensions: 20 × 10 × 3 mm
  - Material: likely NdFeB
  - Pull force (perpendicular): 6 kg

![Image](pic/magnet-rail.jpg)

The arrangement of coils, magnets, and sensors is shown in the diagram below. The gap between the coils and magnets is approximately 0.5 to 1 mm.

![Image](pic/coil-magnet-sensor.png)

The stator position is detected using three OH49E linear Hall sensors, which are electrically arranged at 120° intervals. The outputs of these sensors are processed using a Clarke transformation to calculate the precise stator position. The image below shows simulated waveforms obtained using the script `utils/sim-linear-hall-sensor-to-angle.py`.

![Image](pic/linear-hall-sensor-angle-calcuration.png)

The linear Hall sensors provide a cyclic position signal, corresponding to the relative displacement from the initial power-on position.  
The signal represents the absolute position within a single magnetic pole pair, but it cannot determine which pole pair segment the stator is currently aligned with. To obtain a global absolute position, the system performs an initialization routine:  
the stator is moved until the calculated position no longer increases, allowing the controller to detect the physical end of travel.  
This detected endpoint is used as the absolute zero position for the entire linear range.

## Controller Description

- The main controller is a B-G431B-ESC1 board, and the motor is controlled using the SimpleFOC library.  
- A separate Raspberry Pi Pico is used to handle the linear Hall sensor inputs.  
  The Pico provides the stator angle to the main controller via I2C.  
  (It is implemented to appear as an AS5048 magnetic sensor.)

![Image](pic/controller-1.jpg)

## etc

![Image](pic/make-coil-1.jpg)
![Image](pic/make-coil-2.jpg)
![Image](pic/make-coil-3.jpg)
![Image](pic/make-coil-4.jpg)
![Image](pic/make-coil-5.jpg)
![Image](pic/make-coil-6.jpg)
![Image](pic/make-coil-7.jpg)
![Image](pic/make-coil-8.jpg)
![Image](pic/make-coil-9.jpg)
![Image](pic/make-coil-10.jpg)
![Image](pic/make-coil-11.jpg)
![Image](pic/make-coil-12.jpg)
![Image](pic/etc-1.jpg)
![Image](pic/etc-2.jpg)
![Image](pic/etc-3.jpg)
![Image](pic/etc-4.jpg)
![Image](pic/etc-5.jpg)

