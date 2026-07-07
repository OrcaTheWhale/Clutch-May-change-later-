# Clutch

### Section 1: Description and Project Overview

# Clutch

A hybrid high-precision 2D/3D Air-Mouse wearable smart glove with an expresssive 3D spatial macro trigger, designed for 3D CAD engineering softwares.

## Hardware

-  SEEED Studio XIAO ESP32-C3
-  MPU6050 6-axis IMUs (Accelerometer + Gyroscope) 
-  5 tactile switches

## Features

- Set your custom keybinds.
- Control device cursor using tilt and motion
- Contains the basic mouse controls and keyboard controls you might need to use

## Firmware

- Built with PlatformIO
- Built with jitter and other ambient data in mind
- Controllable through the web

## Getting started

1. Plug the glove in over USB-C.
2. You should see the bluetooth device. Connect to it
3. To calibrate, open the [<u>calibration dashboard</u>](https://fallout.hiralsinghal.me) in Chrome or Edge (desktop only).
4. Click "Connect Device" and pick the glove's serial port.
5. Adjust the panels, set your keybinds, and hit "Send to Device".
6. Put the glove on and make something.

## Wiring diagram

<img width="801" height="786" alt="image" src="https://github.com/user-attachments/assets/db952e88-88dd-4ecf-8354-9240640acc1f" />


## Zine

<img width="205" height="295" alt="image" src="https://github.com/user-attachments/assets/f1115d01-dcb5-46bc-9019-ca37ed70d413" />



### Section 2: The Creation Process

All these were under 2 days.


## Drawbacks

Original Plan:
- Glove that acts as a controller in 3d space, moving the cursor, and starting macros on gesture.
- Two MPU6050s that are compared for gesture recognition

Drawback 1: Machine Learning Accuracy
- ML may not be so accurate dealing with different kinds of gestures, and would take a lot of effort to ensure no mix ups.

Drawback 2: The MPU6050
- The accuracy and the data by the MPU6050 is not strong enough to have gestures standing out between each other.
- For example, just moving Left/Right causes problems.

- The first was left direction, while the second one was a right direction.

peakPosAx=1.89  peakNegAx=-1.81  peakPosAy=0.46  peakNegAy=-1.37  -> axPeak=1.89 ayPeak=1.37
>>> Gesture detected: RIGHT

peakPosAx=1.88  peakNegAx=-1.81  peakPosAy=0.22  peakNegAy=-0.89  -> axPeak=1.88 ayPeak=0.89
>>> Gesture detected: RIGHT

Drawback 3: Time 
- The fact that we only had the components one day before the deadline, meaning that any complex ML or recognition wouldn't hold up.
- Couldn't have enough time to find furry paws

Drawback 4: Items at hand
- The Fallout shop ran out of MPU6050s, with only one left. This was okay - as we couldn't do two anyways. More on this one next drawback.
- The soldering irons are trash. 
- Couldn't find gloves anywhere.

Drawback 5: Short on first connection
- With just the XIAO ESP C3, and the MPU6050, something was shorted on the first connection, meaning that it took us extra time to find out the problem. 
A multimeter was saying the reading was fine, but it was probably not working.

Drawback 6: No motor driver for vibration feedback
- Lost motor driver, and makerspace does not have diodes. 

## Iterations

This project has went through different iterations, versions, and downgrades, such as:

- Went from ML --> Hardcode gestures due to having the four gestures being "simple"
  
- Removed hardcode - MPU6050 data was too innaccurate, and didn't have enough time. Also, misunderstanding of the MPU6050 has caused this too.
  
- Due to removing ML, went from calling the second mode "ML Mode" to "Clutch Mode" - Not too important, but was a marker to realizing "The Great Downgrade"
  
- Changing from hardcode gestures to "simple" gestures, and mapping buttons depending on "mode".
  
- Changing CTRL Z, CTRL Y, etc gestures to LEFT CLICK, RIGHT CLICK, etc buttons, but changing their functions on "clutch" mode.

## Solutions, Process Work 

- Went from instant component connection to breadboard
  
- Needed extra Xiao ESP C3 for test while components are matched, and testing firmware and wiring



## Made by

Controls system, firmware, features, repository - Edward

Attaching components, features, building, zine - Ryaan
