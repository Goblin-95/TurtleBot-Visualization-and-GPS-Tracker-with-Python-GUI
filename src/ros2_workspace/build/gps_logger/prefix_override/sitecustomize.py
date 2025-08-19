import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/goblin/turtlebot_gps_workspace/install/gps_logger'
