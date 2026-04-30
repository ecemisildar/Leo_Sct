import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ecem/ros2_ws/src/Leo_sct/install/swarm_basics'
