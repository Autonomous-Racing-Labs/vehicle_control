# Vehicle Control

This node is used for manual control and intervention of the f1tenth vehicle.
To this end, it publishes `AckermannDriveStamped` messages to the VESC and listens to both gamepad input and input from the autonomous control stack.

Depending on the mode it is currently in, either the manual comands or the autonomous controls are forwarded to
the VESC.

## Topics
Publishes `AckermannDriveStamped` messages on `/drive`

Listens for `AckermannDriveStamped` messages on `/to_drive`
Listens for `Bool` messages on `/emergency_brake`
Listens for `Joy` messages on `/joy`

## Configuration
Configuration file can be found under `config/gamepad.yaml`

Joystick is preconfigured for the wireless logitec gamepad F710.

TODO: description which button does what

## Launch Files
Launch file `launch/vehicle_control_simulation_launch.py` launches the whole simulation including rviz

Launch file `launch/vehicle_control_launch.py` launches the driver stack used on the real f1tenth vehicle.
