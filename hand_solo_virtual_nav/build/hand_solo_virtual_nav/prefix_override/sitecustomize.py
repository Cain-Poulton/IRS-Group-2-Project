import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/irslab_ws/src/hand_solo_virtual_nav/install/hand_solo_virtual_nav'
