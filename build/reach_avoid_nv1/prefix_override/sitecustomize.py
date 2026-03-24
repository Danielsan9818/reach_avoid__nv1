import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bitdrones/ros2_ws/src/reach_avoid_nv1/install/reach_avoid_nv1'
