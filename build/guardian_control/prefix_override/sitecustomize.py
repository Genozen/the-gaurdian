import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/genozen/Documents/the_gaurdian/install/guardian_control'
