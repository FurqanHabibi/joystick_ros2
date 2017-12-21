# joystick_ros2
The first joystick driver for ROS2 created from scratch to support all platforms: Linux, OS X, Windows.

## Install
```
# with ROS2 already sourced
$ cd <ros2_workspace>/src
$ git clone https://github.com/FurqanHabibi/joystick_ros2
$ cd ..
$ ament build

# for Linux / OS X
$ source install/local_setup.bash

# for Windows
$ call install/local_setup.bat
```

## Usage
- Plug in the joystick
- Run the node with below command
    ```
    $ ros2 run joystick_ros2 joystick_ros2
    ``` 

## Supported joysticks
Windows:

    - All Xinput Controller

Linux, Mac OSX:

    - PS4 Controller
    - Logitech F710
    - Xbox One Controller

## ROS2 Node
### Published Topics
- joy ([sensor_msgs/Joy](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Joy.msg))

    Outputs the joystick state.
### Parameters
TODO : use ros2 param when it becomes available on rclpy.

- ~deadzone (double, default: 0.05)

    Amount by which the joystick has to move before it is considered to be off-center. This parameter is specified relative to an axis normalized between -1 and 1. Thus, 0.1 means that the joystick has to move 10% of the way to the edge of an axis's range before that axis will output a non-zero value. Linux does its own deadzone processing, so in many cases this value can be set to zero.

- ~autorepeat_rate (double, default: 0.0 (disabled))

    Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message.

- ~coalesce_interval (double, default: 0.001)

    Axis events that are received within coalesce_interval (seconds) of each other are sent out in a single ROS message. Since the kernel sends each axis motion as a separate event, coalescing greatly reduces the rate at which messages are sent. This option can also be used to limit the rate of outgoing messages. Button events are always sent out immediately to avoid missing button presses.

## Notes
### OS X Permissions
You probably have to use the settings application to allow gamepad input on OS X.
![Mac Settings](/images/mac_settings.png)
