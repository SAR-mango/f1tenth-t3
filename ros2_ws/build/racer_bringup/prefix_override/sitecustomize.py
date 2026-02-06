import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/erk/f1tenth-t3/ros2_ws/install/racer_bringup'
