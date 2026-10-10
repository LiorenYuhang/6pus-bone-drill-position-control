#!/usr/bin/env python3
"""F1 test-only FakeBus injection. Rejects active before importing the Child."""
import json
import os
from pathlib import Path
import runpy
import sys
import time

config=json.loads(sys.argv[sys.argv.index('--config')+1])
if config.get('hardware_mode')!='mock':
    raise RuntimeError('F1 child injection permits mock only')
bootstrap=Path(__file__).resolve().parents[1]/'src/pus6_motor_driver/pus6_motor_driver/j14_uart_child_main.py'
runpy.run_path(str(bootstrap),run_name='f1_bootstrap')
from pus6_motor_driver import j14_process_uart as uart
original=uart.MockServoBus.write
out=Path(os.environ['J14_DEMO_MOCK_INJECTION_DIR'])
last_scenario=None

class HeldPositions(dict):
    def __setitem__(self,key,value):
        super().__setitem__(key,1020)


def inject(bus,frame):
    global last_scenario
    request=out/'mock_injection.json'
    scenario=json.loads(request.read_text()).get('scenario') if request.exists() else None
    if scenario!=last_scenario:
        last_scenario=scenario
        if scenario:
            (out/'mock_injection_observed.json').write_text(json.dumps(dict(scenario=scenario,ns=time.monotonic_ns()))+'\n')
    if scenario=='hardware_fault':
        bus.faults[1]=1
    if scenario=='home_timeout' and not isinstance(bus.positions,HeldPositions):
        bus.positions=HeldPositions({mid:1020 for mid in range(1,7)})
    return original(bus,frame)

uart.MockServoBus.write=inject
uart.main()
