#!/usr/bin/env python3
"""F2 FakeBus boundary fixtures; reject active before importing UART owner."""
import json
import os
from pathlib import Path
import runpy
import sys
import time
config=json.loads(sys.argv[sys.argv.index('--config')+1])
if config.get('hardware_mode')!='mock':
    raise RuntimeError('F2 fixture permits Mock only')
out=Path(os.environ['J14_DEMO_MOCK_INJECTION_DIR'])
scenario=os.environ['J14_DEMO_F2_SCENARIO']
import serial
guard=dict(serial_open_attempts=0,physical_access_denied=True,hardware_mode='mock')
(out/'mock_serial_guard.json').write_text(json.dumps(guard)+'\n')
def deny_serial(*args,**kwargs):
    guard['serial_open_attempts']+=1
    (out/'mock_serial_guard.json').write_text(json.dumps(guard)+'\n')
    raise RuntimeError('Physical serial access forbidden in F2 Mock fixture')
serial.Serial=deny_serial
runpy.run_path(str(Path(__file__).resolve().parents[1]/'src/pus6_motor_driver/pus6_motor_driver/j14_uart_child_main.py'),run_name='f2_bootstrap')
from pus6_motor_driver import j14_process_uart as uart
original_init=uart.MockServoBus.__init__
original_write=uart.MockServoBus.write
class HeldPositions(dict):
    def __setitem__(self,key,value):super().__setitem__(key,1020)
def initialize(bus,*args,**kwargs):
    kwargs['initial_positions']=[1000]*6 if scenario in ('already_home','startup_fault','invalid_feedback') else [1090,1221,1163,912,786,842]
    original_init(bus,*args,**kwargs)
    if scenario=='startup_timeout':bus.positions=HeldPositions({i:1020 for i in range(1,7)})
    (out/'mock_initial_state.json').write_text(json.dumps(dict(scenario=scenario,positions=list(bus.positions.values()),hardware_mode='mock'))+'\n')
def write(bus,frame):
    request=out/'mock_injection.json'
    active_fault=request.exists() and json.loads(request.read_text()).get('scenario')=='hardware_fault'
    if scenario=='startup_fault' and (out/'relay.log').exists() or active_fault:
        bus.faults[1]=1
        if not (out/'mock_injection_observed.json').exists():
            (out/'mock_injection_observed.json').write_text(json.dumps(dict(scenario=scenario,ns=time.monotonic_ns()))+'\n')
    return original_write(bus,frame)
uart.MockServoBus.__init__=initialize
uart.MockServoBus.write=write
uart.main()
