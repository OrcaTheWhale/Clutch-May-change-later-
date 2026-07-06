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

Drawback 4: Items at hand
- The Fallout shop ran out of MPU6050s, with only one left. This was okay - as we couldn't do two anyways. More on this one next drawback.
- The soldering irons are trash. 
- Couldn't find gloves anywhere.

Drawback 5: Short on first connection
- With just the XIAO ESP C3, and the MPU6050, something was shorted on the first connection, meaning that it took us extra time to find out the problem. 
A multimeter was saying the reading was fine, but it was probably not working. 




