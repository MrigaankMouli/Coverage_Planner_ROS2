import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zero/Desktop/ISDC/cov_ws/install/Coverage_Planner'
