"""Clean A1.9 bootstrap: load geometry data without kinematics package side effects."""
import importlib.util
from pathlib import Path
import sys
import types

# safety.py needs geometry_config, but the kinematics package __init__ eagerly
# imports ROS messages/state synchronizer. Load only its existing data module;
# no copied constants, altered geometry, ROS, IK or FK execution.
if 'pus6_kinematics' not in sys.modules:
    spec=importlib.util.find_spec('pus6_kinematics')
    if spec is None or spec.submodule_search_locations is None:
        raise ImportError('existing pus6_kinematics geometry configuration unavailable')
    package=types.ModuleType('pus6_kinematics')
    package.__path__=list(spec.submodule_search_locations)
    package.__spec__=spec
    sys.modules['pus6_kinematics']=package

from pus6_motor_driver.j14_process_uart import main
if __name__=='__main__':main()
