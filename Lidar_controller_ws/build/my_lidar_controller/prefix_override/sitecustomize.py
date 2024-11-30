import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/thesis/Lidar_controller_ws/install/my_lidar_controller'
