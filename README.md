Vehicle Control

This node is used for manual control and intervention of the f1tenth vehicle.
To this end, it publishes AckermannMsgs to the VESC and listens to both gamepad input and input from the autonomous control stack

Depending on the mode it is currently in, either the manual comands or the autonomous controls are forwarded to
the VESC.
