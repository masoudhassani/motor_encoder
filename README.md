# Brushed DC Motor Controller 
This software works on arduino nano, pro min and uno in conjunction with TB9051FTG motor driver.
The motor controller receives custom made GCodes through I2C and drives the connected motor/encoder to
a angle setpoint. For this another micro-controller or computer is required to send I2C commands
to the motor controller. For instance, M2G2A120 moves the 3rd motor to 120 deg
