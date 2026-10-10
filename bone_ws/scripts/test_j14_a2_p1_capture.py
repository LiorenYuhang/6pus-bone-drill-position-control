from j14_a2_p1_capture import admission,command
from pathlib import Path

def clean():
    return dict(stop_reason='window_complete',cycles=1500,tx_timeout_count=0,hardware_fault_count=0,
        packet_drop_count=0,first_cycle_seq=0,pause_ack_confirmed=True,child_returncode=0,
        bridge_returncode=0,serial_after=dict(free=True),residual_processes=[],handshake_valid=True,
        production_path_confirmed=True,unique_publishers_confirmed=True)

def test_R2_gates_true_failures_not_watchlist():
    assert admission(clean())
    for key,value in [('cycles',1499),('packet_drop_count',1),('tx_timeout_count',1),
                      ('hardware_fault_count',1),('child_returncode',1),('bridge_returncode',1),
                      ('handshake_valid',False),('production_path_confirmed',False),
                      ('unique_publishers_confirmed',False),('pause_ack_confirmed',False)]:
        m=clean();m[key]=value;assert not admission(m)
    m=clean();m.update(max_ipc_age_ms=101.,max_gap_ms=110.,max_wake_ms=31.,shutdown_exception=True)
    assert admission(m) # diagnosis/qualification watchlist, no extra hardware stop gate

def test_normal_launch_command_no_A1_flags():
    c=command(Path('/tmp/mock_only'),30)
    assert c[:4]==['ros2','launch','pus6_bringup','pus6_system.launch.py']
    assert not any('PUS6_J14_' in x for x in c)
    assert not any('driver_architecture:=' in x for x in c) # prove formal default
