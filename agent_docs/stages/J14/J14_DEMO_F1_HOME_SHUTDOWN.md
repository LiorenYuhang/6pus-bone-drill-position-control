# J14-A-Demo-F1：正常 Ctrl+C 回 HOME 与干净退出

软件／Mock 门槛通过。当前只交给用户复验 Feedforward；PD/LADRC 真机入口继续 HOLD。本阶段未启动真机或产生真实电机动作。

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward.sh
```

等待 ACTIVE，做短时间功能确认，再按 Ctrl+C。正常停止应显示 HOME recovery PASS、六轴 Pause 6/6 ACK、Serial release PASS、Residual processes 0、Demo shutdown PASS。Fault 时应显示 SAFETY STOP 和 HOME recovery SKIPPED，不能要求故障后继续回 HOME。

## 1–18 复核结果

1. **原真机残留：**Platform H30 IMU PID **49596**、Base H30 IMU PID **49597**；原 `ros2 run` 包装进程分别为 49586、49585。包装进程等 SIGINT 3 秒后收到 SIGTERM，returncode=-15；IMU 子进程未收到信号，仍持有 ttyACM1／ttyACM0。叶进程原 exit code **NOT RECORDED**。IK/control 已退出，不能把它们写成这两项残留。两项经命令身份核对后各发一次 SIGINT，约 0.151 秒退出并释放串口；没有启动新的硬件进程。
2. **原路径没有 HOME：**Ctrl+C 直接走 controller→relay→整个 robot launch 的 teardown；Parent close 触发 Child Pause，未发布 Cartesian HOME。旧代码确实不含 HOME recovery。
3. **复用路径：**审计 J7 standalone HOME/PARK 后没有执行它，因为它会再次拥有 UART并绕过当前 shaper。采用已有 J10（冻结 J8/J9 Cartesian 链）`WorkspaceMeasurementHarness.publish_target_pose`、`_home_feedback_in_band`、`_home_timeout_from_feedback`及默认连续样本数。用原 runner 节点及其原 publisher 调用既有方法，没有第二个 command authority。
4. **正常 Ctrl+C：**停止原 runner ACTIVE timer→同一 publisher 发布 Cartesian HOME→IK→原 shaper→command_joint_steps→原 Parent→Child→新反馈确认→原 Parent/Child Pause／join→controller host退出→relay→直接持有的 IMU 叶进程→其余 ROS 节点→串口／进程检查。
5. **Fault STOP：**故障、过流、反馈 hard timeout、Child/driver STOP、unexpected controller/relay exit、SIGTERM 等跳过 HOME，立即请求原 Parent Pause；Child 原有独立 lease／安全 STOP 继续有效。HOME 中遇到故障或第二次中断也终止 HOME 发布。
6. **HOME 完成：**OPERATING_HOME=[1000]*6，既有 inclusive band **995–1005**；连续 **5 个更新后的实际 feedback 包**满足六轴 band，且每轴接收时间均晚于 HOME 开始。不是依据 command target 判成功。PARK=[0]*6 完全不参与。
7. **超时：**复用既有 J10 推导 `max(1.0, max_step_distance/60 + 1.5)` 秒。超时立即请求 Pause，明确 HOME RECOVERY TIMEOUT／Demo shutdown FAIL，继续清理，不无限等待、不把失败改成 HOME PASS。
8. **Feedforward Mock：PASS。**范围外初始反馈 [989,979,989,1011,1021,1011]→[999,999,999,1001,1001,1001]，5 个新样本确认；约 0.300 秒。这只是 Mock 回程结果。
9. **Classical Mock：PASS。**[989,978,989,1011,1022,1011]→[999,999,999,1001,1001,1001]，约 0.308 秒。
10. **LADRC Mock：PASS。**[988,977,988,1012,1023,1012]→[999,1000,999,1001,1000,1001]，约 0.321 秒。
11. **Fault／意外退出 Mock：PASS（策略验证）。**FakeBus ID1 故障实际触发 Child `RuntimeError: motor_fault_ID1`；HOME SKIPPED、无 HOME 文件／发布。controller／relay 外部 SIGTERM 意外退出同样无 HOME。会话 returncode=1、Demo shutdown FAIL，Pause和清理 PASS。
12. **HOME timeout Mock：PASS（策略验证）。**FakeBus 测量保持1020，目标为1000也不判成功；约1.835秒超过既有1.833秒预算，HOME_TIMEOUT→Pause，保留失败会话、返回1。
13. **退出：0 residual。**最终七个场景都确认Pause 6/6、串口空闲、Child／Bridge returncode=0、四个ROS节点returncode=0；无 `rcl_shutdown already called`／ExternalShutdownException退出错误。
14. **Relay：正常退出。**正常回程、timeout、fault、controller意外退出后约0.214秒退出、returncode=0，无SIGKILL／升级信号；relay意外退出场景的测试端SIGTERM也正常返回0。独立 normal／SIGINT／SIGTERM 回归通过。
15. **hardware_write_count=0。**全部新的运行是明确Mock；.02rad软件IMU输入用于建立范围外起点。失败注入是guarded Mock Child中的FakeBus数据，拒绝active；没有触碰真实电机UART。
16. **Production source：0变化。**187个源码／配置与阶段开始的快照逐字节一致；原production_driver.py仍保留Q3J SingleThreadedExecutor。Demo-owned lifecycle prefix使用同一factory/classes及原launch参数/remap，不修改production main，也不patch全局ROS shutdown。
17. **Frozen项：0变化。**lease、50Hz、1500mA、bounds、80-step reconnect、UART ownership、timeout recovery、shaper、控制器参数／算法保持。两份J11历史shell未改。原tracked diff仍为12 files、+1142/−76。
18. **允许用户复验Feedforward。**软件门槛通过，真实HOME返回及新入口完整退出尚待人工确认；本阶段没有运行PD/LADRC真机，也不提供其真机命令。

## 实现范围和证据

修改 `bone_ws/scripts/j14_demo_launcher.py`、`j14_demo_controller.py`、软件IMU输入参数及Demo测试；新增 `j14_demo_home.py`、`j14_demo_node_host.py`、Mock-only Child注入／检查与F1测试。shell入口和stage relay映射未改。

ROS节点由Demo-owned lifecycle prefix接收stop event，先destroy原节点、shutdown executor，最后shutdown仍有效的context；Parent使用SingleThreadedExecutor。实际IMU直接启动原安装executable，其Popen PID就是叶进程，避免终止ros2run包装进程后留下子进程。

[机读总结果](../results/j14/j14_a_demo_f1_software/validation_summary.json) · [历史残留归因](../results/j14/j14_a_demo_f1_software/historical_residual_attribution.json) · [旧IMU释放](../results/j14/j14_a_demo_f1_software/historical_owned_imu_release.json) · [源码保护](../results/j14/j14_a_demo_f1_software/source_protection_final.json)

原真机日志没有重写；关闭旧IMU时，其原日志自然追加了rclcpp signal_handler事件。失败／中间检查保留在独立目录：范围内起点导致回程证明不足、首次注入未触达write路径、一次Mock与单元测试重叠造成独占锁拒绝。后续串行检查没有覆盖这些证据。

测试：**38 Demo/HOME/relay targeted + 73 J14 driver + 4 J10 HOME = 115 PASS**；三个shell语法、Python编译、git diff --check通过。未运行300秒stress或实机qualification。No commit/push/tag；完成后STOP。
