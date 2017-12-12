# Copyright 2017 Muhammad Furqan Habibi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import platform
import time
from math import modf

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from inputs import devices, UnpluggedError

# Microsoft X-Box 360 pad
XINPUT_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_WEST': 2,
    'BTN_NORTH': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_START': 6,
    'BTN_SELECT': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

XINPUT_VALUE_MAP = {
    0: (-32768, 32767),
    1: (32767, -32768),
    2: (0, 255),
    3: (-32768, 32767),
    4: (32767, -32768),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Sony Computer Entertainment Wireless Controller
PS4_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_RX': 2,
    'ABS_Z': 3,
    'ABS_RZ': 4,
    'ABS_RY': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_EAST': 0,
    'BTN_C': 1,
    'BTN_SOUTH': 2,
    'BTN_NORTH': 3,
    'BTN_WEST': 4,
    'BTN_Z': 5,
    'BTN_TL2': 6,
    'BTN_TR2': 7,
    'BTN_MODE': 8,
    'BTN_SELECT': 9,
    'BTN_START':10
}

PS4_VALUE_MAP = {
    0: (0, 255),
    1: (0, 255),
    2: (0, 255),
    3: (0, 255),
    4: (0, 255),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Logitech Gamepad F710
F710_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

F710_VALUE_MAP = {
    0: (-32768, 32767),
    1: (-32768, 32767),
    2: (0, 255),
    3: (-32768, 32767),
    4: (-32768, 32767),
    5: (0, 255),
    6: (-1, 1),
    7: (-1, 1)
}

# Microsoft X-Box One pad
XONE_CODE_MAP = {
    'ABS_X': 0,
    'ABS_Y': 1,
    'ABS_Z': 2,
    'ABS_RX': 3,
    'ABS_RY': 4,
    'ABS_RZ': 5,
    'ABS_HAT0X': 6,
    'ABS_HAT0Y': 7,
    'BTN_SOUTH': 0,
    'BTN_EAST': 1,
    'BTN_NORTH': 2,
    'BTN_WEST': 3,
    'BTN_TL': 4,
    'BTN_TR': 5,
    'BTN_SELECT': 6,
    'BTN_START': 7,
    'BTN_MODE': 8,
    'BTN_THUMBL': 9,
    'BTN_THUMBR':10
}

XONE_VALUE_MAP = {
    0: (-32768, 32767),
    1: (-32768, 32767),
    2: (0, 1023),
    3: (-32768, 32767),
    4: (-32768, -32767),
    5: (0, 1023),
    6: (-1, 1),
    7: (-1, 1)
}

JOYSTICK_CODE_VALUE_MAP = {
    'Microsoft X-Box 360 pad': (XINPUT_CODE_MAP, XINPUT_VALUE_MAP),
    'Sony Computer Entertainment Wireless Controller': (PS4_CODE_MAP, PS4_VALUE_MAP),
    'Logitech Gamepad F710': (F710_CODE_MAP, F710_VALUE_MAP),
    'Microsoft X-Box One pad': (XONE_CODE_MAP, XONE_VALUE_MAP)
}

class JoystickRos2(Node):

    def __init__(self):
        super().__init__('joystick_ros2')

        # Node params
        # TODO : use rosparam
        self.deadzone = 0.05
        self.autorepeat_rate = 0.0
        self.coalesce_interval = 0.001
        self.sleep_time = 0.01

        # Joy message
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.header.frame_id = ''
        self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Joy publisher
        self.publisher_ = self.create_publisher(Joy, 'joy')

        # logic params
        self.last_event = None
        self.last_publish_time = 0

    def publish_joy(self):
        current_time = modf(time.time())
        self.joy.header.stamp.sec = int(current_time[1])
        self.joy.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.publisher_.publish(self.joy)
        self.last_publish_time = time.time()
        #print(self.joy)

    def normalize_key_value(self, key_value_min, key_value_max, key_value):
        normalized = ((key_value - key_value_min) / (key_value_max - key_value_min) * (-2)) + 1
        if (abs(normalized) > self.deadzone):
            return normalized
        else:
            return 0.0

    def run(self):
        device_manager = devices
        while rclpy.ok():
            # get the first joystick
            try:
                gamepad = device_manager.gamepads[0]
            except IndexError:
                print('Joystick not found. Will retry every second.')
                time.sleep(1)
                device_manager.find_devices()
                continue

            # detected joystick is not keymapped yet
            if (gamepad.name not in JOYSTICK_CODE_VALUE_MAP):
                print('Sorry, joystick type not supported yet! Please plug in supported joystick')
                time.sleep(1)
                device_manager.find_devices()
                continue

            # check unplugged joystick
            if (platform.system() == 'Windows'):
                try:
                    gamepad._GamePad__check_state()
                except UnpluggedError:
                    device_manager.find_devices()
                    continue

            # read inputs from joystick
            while True:
                try:
                    events = gamepad._do_iter()
                # check unplugged joystick
                except OSError:
                    print('Joystick not found. Will retry every second.')
                    time.sleep(1)
                    device_manager.find_devices()
                    break
                if events:
                    for event in events:
                        if (event.code in JOYSTICK_CODE_VALUE_MAP[event.device.name][0]):
                            key_code = JOYSTICK_CODE_VALUE_MAP[event.device.name][0][event.code]
                            if (event.ev_type == 'Key'):
                                self.joy.buttons[key_code] = event.state
                                self.publish_joy()
                                self.last_event = event
                            elif (event.ev_type == 'Absolute'):
                                value_range = JOYSTICK_CODE_VALUE_MAP[event.device.name][1][key_code]
                                self.joy.axes[key_code] = self.normalize_key_value(value_range[0], value_range[1], event.state)
                                if (self.last_event is None) or (self.last_event.code != event.code) or (time.time() - self.last_publish_time > self.coalesce_interval):
                                    self.publish_joy()
                                self.last_event = event
                else:
                    break
            ## autorepeat
            if ((self.autorepeat_rate > 0.0) and (time.time() - self.last_publish_time > 1/self.autorepeat_rate)):
                self.publish_joy()

            # sleep to decrease cpu usage
            time.sleep(self.sleep_time)

def main(args=None):
    rclpy.init(args=args)

    joystick_ros2 = JoystickRos2()
    joystick_ros2.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
