import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/genesis/Desktop/from_joystick_to_esp32/install/joystick_control'
