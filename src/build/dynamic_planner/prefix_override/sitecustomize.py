import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nahom/Dynamic-Path-Planning-Robot-In-Crowded-Environment/src/install/dynamic_planner'
