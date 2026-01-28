# Investigating And Understanding Motor Control

Motor status: [current status flag, power in percent, encoder position (degrees), velocity (dps)]

### LEGO-Motor_DPS.py
The program sets motor D to float, which allows the corresponding wheel to be manually turned.

The encoded rotation of motor D is constantly read, and the velocity of motor A is set to the read value. This means you can set the velocity of motor A by adjusting the rotation of wheel D.

Reading the status of motor A, we can see the power of the motor increases with the encoded rotation of motor D. The velocity behaves similarly, but with higher values. Both the power and the velocity fluctuate, albeit independently, likely due to natural noise in the hardware/sensors. 

The encoded rotation of motor A consistently changes in accordance with the velocity of motor A. 

Obstructing wheel A causes the power reading of motor A to increase, since the PID controls will try to compensate for the increased resistance. If the wheel is held strongly enough, the status of the robot reads "2", since the bit for motor overload is set.

### LEGO-Motor_Position.py
Similarly, motor D is set to float. For this program the position, rather than the velocity, of motor A is set to the rotation of motor D.

This causes motor A's position to mirror motor D. However, as there's a motor limit on A, it cannot turn as fast as motor D, which means it can take a while to catch up.

Motor A has a velocity cap of 200, so obstructing the wheel as it rotates makes the motor increase power to achieve a velocity of 200. Once again, if the motor receives enough resistance, the status of the robot reads "2", as it cannot provide enough power for the desired velocity.

### LEGO-Motor_Power.py
The behaviour of this program is extremely similar to LEGO-Motor_DPS.py. However, obstructing the wheel does not increase the power of motor A, as it no longer has a desired velocity to compensate for.
