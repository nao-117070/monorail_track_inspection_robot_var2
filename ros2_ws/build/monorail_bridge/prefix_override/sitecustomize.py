import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hitakalab/Documents/monorail_track_inspection_robot_var2/ros2_ws/install/monorail_bridge'
