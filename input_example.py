"""Simple example showing how to get gamepad events."""

from __future__ import print_function


from inputs import devices
import platform


def main():
    """Just print out some event infomation when the gamepad is used."""
    while 1:
        gamepad = devices.gamepads[0]
        if (platform.system() == 'Windows'):
            gamepad._GamePad__check_state()
        events = gamepad._do_iter()
        if events:
            for event in events:
                print(event.ev_type, event.code, event.state)
                print('-' + event.device.name + '-')


if __name__ == "__main__":
    main()
