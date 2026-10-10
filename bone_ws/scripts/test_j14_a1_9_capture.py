import copy
from j14_a1_9_capture import r2_admissible

def clean():
    return dict(stop_reason='no_natural_timeout_within_window',captured_timeout_count=0,
        child_final=dict(fault='window_complete',closed=True,motion_stop_ns=31_000_000_000,servo_start_ns=1_000_000_000),
        child_returncode=0,shutdown_exception=False,telemetry_complete=True,pause_ack_confirmed=True,
        residual_processes=[],serial_after=dict(free=True))

def test_R2_full_clean_only():
    assert r2_admissible(clean())
    for key,value in [('captured_timeout_count',1),('telemetry_complete',False),
                      ('pause_ack_confirmed',False),('child_returncode',-9),('residual_processes',[1]),
                      ('shutdown_escalation','SIGTERM'),('stop_reason','child_local_fault')]:
        m=clean();m[key]=value;assert not r2_admissible(m)
    m=clean();m['child_final']['motion_stop_ns']-=1;assert not r2_admissible(m)
    m=clean();m['child_final']['fault']='parent_lease_expired';assert not r2_admissible(m)
    m=clean();m['serial_after']['free']=False;assert not r2_admissible(m)


def test_non_hardware_ROS_shutdown_traceback_does_not_substitute_for_cleanup_audit():
    m=clean();m['shutdown_exception']=True;assert r2_admissible(m)
