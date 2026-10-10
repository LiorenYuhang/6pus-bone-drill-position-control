#!/usr/bin/env python3
"""Test-only invalid consumer feedback fixture, with no serial access."""
import sys
import os
if '--hardware-mode' not in sys.argv or sys.argv[sys.argv.index('--hardware-mode')+1]!='mock':
    raise RuntimeError('F2 fixture requires explicit Mock')
import j14_demo_home as home
original_failure=home.feedback_failure
def invalid(node,feedback):
    if node.get_name()=='j14_demo_observer' and os.environ['J14_DEMO_F2_SCENARIO']=='invalid_feedback':
        return original_failure(node,dict(feedback,rx_ns=[0]*6))
    return original_failure(node,feedback)
home.feedback_failure=invalid
from j14_demo_launcher import main
sys.exit(main())
