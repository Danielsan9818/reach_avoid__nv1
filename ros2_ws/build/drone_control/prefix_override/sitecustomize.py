import sys
if sys.prefix == '/home/quarrg/miniforge3/envs/ros_humble':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/quarrg/ros2_ws/install'
