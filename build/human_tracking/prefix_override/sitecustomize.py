import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dtc/humanflow/ros2_ghadron_gimbal/install/human_tracking'
