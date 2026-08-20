# J5-C2-HW 装配后验收工具链（装配前准备）

**当前状态**：工具链已准备，当前仅允许 Mock / synthetic / replay；不代表 J5-C2-HW 已启动，也不执行实机运动、最终 B2 标定或 IMU/FK 融合。

## 1. 几何配置

唯一软件参考配置为：

```text
bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml
```

`Analytical6PusKinematics()` 默认从该 YAML 加载 base/platform anchors、strut length、名义高度、step↔meter、HOME/PARK raw reference 和 stroke limits。`config_path` 和旧的显式构造参数仍可用于测试/参数覆盖。

YAML 中 `status`、`geometry.confirmation`、`actuator.confirmation` 和 `imu.confirmation` 明确表示最终装配相关值仍待实测。IMU rotation、translation 和 lever arm 当前为 `null`，禁止填入猜测值。

## 2. 离线验收命令

```bash
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_hw_acceptance.py \
  --duration-s 10 --feedback-hz 30 --query-rate-hz 200 \
  --output-prefix /tmp/j5_c2_hw_acceptance
```

输入 replay 使用 JSON 数组或 JSONL，每行对应 `TimedJointFeedback` 的字段：`uart_cycle_sequence`、`per_axis_rx_done_monotonic_ns`、`positions_meters`、`raw_steps`、`snapshot_start_monotonic_ns`、`snapshot_end_monotonic_ns`、`fault_codes`。

脚本输出同名 `.json` 和 `.csv`，记录：

* query、per-axis bracket、`SYNCED`、`WAIT_*` 和 FK `DROP`；
* FK convergence、residual、iterations、branch jump 和 compute time；
* UART scan span、per-axis RX span、sequence gap/duplicate/out-of-order、timestamp violation 和 fault frame。

脚本不导入 `rclpy`，不启动 ROS executor，不打开串口，报告固定写入 `hardware_motion_executed=false`、`hardware_write_count=0`。

## 3. 装配后门禁顺序

1. `final_geometry_confirmed`：核对最终 base/platform anchors、杆长、安装角和坐标方向。
2. `six_axis_zero_mapping_confirmed`：逐轴确认 raw zero、OPERATING_HOME 和 STORAGE_PARK 映射。
3. `home_park_confirmed`：在独立运动授权下确认 HOME/PARK 基础运维链路。
4. `q_actual_fk_residual_stable`：采集真实 `q_actual`，执行 Timed feedback → Fixed-Lag → q_sync → FK，确认 residual 稳定且无持续 WAIT/DROP。
5. `small_xyz_rpy_consistency`：仅在前四项通过后进行小幅 XYZ/RPY 一致性验证。
6. 以上全部通过后，才允许进入最终 J5-B2 外参/lever arm 后的 IMU/FK 融合设计；ESKF 和 C3 不提前启动。

当前报告中的装配门禁默认全部为 `false`，软件工具链就绪不等于硬件 PASS。
