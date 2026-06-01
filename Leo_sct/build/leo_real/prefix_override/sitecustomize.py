import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ecem/ros2_real_ws/src/Leo_sct_real/Leo_sct/install/leo_real'
