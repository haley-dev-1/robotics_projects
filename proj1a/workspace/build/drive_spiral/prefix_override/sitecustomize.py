import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/acct/halind/csce274/proj1a/workspace/install/drive_spiral'
