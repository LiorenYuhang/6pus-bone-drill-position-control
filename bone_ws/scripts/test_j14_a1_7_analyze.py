"""Check offline joins, strict heartbeat overlap and legacy CPU labeling."""
from j14_a1_7_analyze import join_timeline,overlaps,event_matches_csv


def test_epoch_nanosecond_identity_keeps_integer_precision():
    value=1791445718842789385
    assert event_matches_csv({'epoch':value},{'epoch':str(value)})
    assert not event_matches_csv({'epoch':value},{'epoch':str(value+1)})


def test_heartbeat_requires_actual_interval_overlap():
    heartbeat=[dict(expected_ns='10',wake_ns='30')]
    assert overlaps(30,40,heartbeat)==[]
    assert overlaps(0,10,heartbeat)==[]
    assert overlaps(20,25,heartbeat)==heartbeat


def test_last_50_preceding_cycles_and_same_write_cpu_are_separate():
    cycles=[]
    for seq in range(101):
        row=dict(cycle_seq=str(seq),t_entry_ns=str(1000+seq*10),t_exit_ns=str(1005+seq*10),
                 expected_wake_ns=str(1000+seq*10),callback_total_ms='5',inter_callback_gap_ms='5',
                 wake_lateness_ms='0',scheduler_gap_ms='10',feedback_age_max_ms='10',
                 uart_tx_ms='2',uart_drain_ms='1',rx_max_single_read_wall_ms='.1',
                 process_cpu_delta_ms='20',thread_cpu_delta_ms='10',vol_ctx_switches_delta='3',
                 invol_ctx_switches_delta='0')
        cycles.append(row)
    writes=[dict(cycle_seq='100',serial_write_cpu_ms='1',process_cpu_duration_ns='2000000')]
    joined=join_timeline(cycles,[],writes,[],100)
    assert [x['cycle_seq'] for x in joined]==list(range(50,101))
    assert joined[-1]['entry_interval_process_cpu_ms']==20
    assert joined[-1]['write_interval_process_cpu_sum_ms']==2
    assert joined[-1]['write_interval_thread_cpu_sum_ms']==1
    assert not joined[-1]['heartbeat_overlaps']
