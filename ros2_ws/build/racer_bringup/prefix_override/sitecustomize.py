import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/f1tt3/f1tenth-t3/ros2_ws/install/racer_bringup'
