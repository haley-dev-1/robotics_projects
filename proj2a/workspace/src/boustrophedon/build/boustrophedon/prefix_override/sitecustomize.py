import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/acct/halind/Documents/csce274/proj2a/workspace/src/boustrophedon/install/boustrophedon'
