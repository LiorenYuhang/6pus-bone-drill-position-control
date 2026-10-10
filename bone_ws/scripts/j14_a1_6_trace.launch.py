"""Use the original production launch, adding a driver-only diagnostic prefix."""
import importlib.util
import os
from pathlib import Path
import shlex

ROOT = Path('/home/lyapunov/bone_drill')


def generate_launch_description():
    spec = importlib.util.spec_from_file_location('j14_original_bringup',
        ROOT/'bone_ws/src/pus6_bringup/launch/pus6_system.launch.py')
    original = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original)
    original_node = original.Node
    out = Path(os.environ['PUS6_J14_TRACE_DIR'])
    tty = os.environ.get('PUS6_J14_TRACE_TTY', '/dev/ttyUSB0')
    command = ['strace', '-D', '-ff', '-ttt', '-T', '-yy', '-I', '3',
        '-e', 'trace=write,select,pselect6,poll,ppoll,ioctl,read',
        '-P', tty, '-o', str(out/'driver_trace')]

    def diagnostic_node(*args, **kwargs):
        if kwargs.get('package') == 'pus6_motor_driver' and kwargs.get('executable') == 'motor_driver_node':
            kwargs['prefix'] = shlex.join(command)
        return original_node(*args, **kwargs)

    original.Node = diagnostic_node
    return original.generate_launch_description()
