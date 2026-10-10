# J14-A-Demo source audit

Audit performed before implementation against the existing working tree, not HEAD.

- Historical `bone_ws/scripts/run_j11_demo_feedforward.sh` and `_pd.sh`: source Jazzy/install, call old staged launchers, assume legacy feedback, and independently run a real HOME runner during cleanup. Preserve these entries; do not carry that second hardware authority into J14 Demo.
- `bone_ws/src/h30_imu/scripts/j11_staged_launcher.py` and `j11_m2_staged_launcher.py`: useful staged sequencing, but their Stage A subscribes to legacy raw feedback without the J14 adapter. Older cleanup also uses broad process matching/escalation. Reuse runner nodes instead of these launchers.
- `j11_base_feedforward_runner.J11BaseFeedforwardRunner`: frozen SO(3) inverse Base rotation, Platform observer only; publishes `/pus6/target_cartesian_pose`. Existing HOME, freshness, publisher ownership and fault gates remain in the node.
- `j11_m2_pid_residual_runner.J11M2PidResidualRunner`: Q3-R1D controller implementation; read constructor gains from `j11_m2_pid_params.yaml` rather than CLI defaults (which differ for Pitch-D).
- `j13_ladrc_shadow_runner.J13LadrcShadowRunner`: Q3-R2 implementation with `controller_mode='3axis_active'`; read existing `j13_ladrc_params.yaml`, preserve takeover and all gates.
- `j14_a2_q3_capture.py`: reuse exact production launch, dual IMU mapping, relay and controller selection. Its 60-second limit, disturbance instructions, heartbeat capture, qualification capacity and heavy snapshots belong to qualification, not Demo. Do not copy its orchestration.
- `j14_a2_q3_feedback_relay.py`: preserve both 1:1 topic mappings and QoS. Previous signal handler called context shutdown while `spin` could be inside the wait set. Replace only this lifecycle with a stop event plus bounded `spin_once`; destroy the node/context outside signal handling. Observed graceful exit is the validated result; this does not establish a native DDS/GIL leaf cause.
- `pus6_system.launch.py`: use unchanged launch with existing three-process production defaults. Parent executor remains Q3J SingleThreadedExecutor; Child/Bridge ownership is unchanged. No production topic additions.
- Shutdown: runner timer cancelled before its node destruction, then relay, then launch leader SIGINT (one forwarding path), coordinated Child pause, Child/Bridge joins, IMU exit, serial/process checks. No independent HOME motion after Demo stop. The operator must begin at the existing OPERATING_HOME preflight state.

Demo lifecycle host imports unchanged runner classes and runs their existing callbacks/timer. The only instance retention changes are bounded telemetry/slew diagnostic buffers (250 rows). No finite session duration and no copied control algorithm. Runner `hardware_write_count` means Cartesian pose publications; actual hardware count is taken from `child_final.json`.

Production telemetry suppresses qualification-only `command_ingress.jsonl`/`parent_state_packets.jsonl`. The correct delivery evidence is `parent_ipc.jsonl` (`parent_send.seq`) and `child_cycles.jsonl` (`command_seq`), alongside observed target/IK/control/Bridge topics and graph ownership.
