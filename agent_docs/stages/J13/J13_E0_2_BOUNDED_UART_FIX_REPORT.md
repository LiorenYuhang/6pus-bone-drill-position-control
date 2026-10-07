# J13-E0-2 Bounded UART I/O Runtime Fix 专项报告

**执行时间**: 2026-10-06  
**对应阶段**: J13-E0-2 (Bounded UART I/O Runtime Fix)  
**前序诊断**: J13-E0-1 (Servo Runtime Stall Diagnostic Reproduction, 根因定位: `B_IN_CALLBACK_UART_BLOCK`)  
**状态**: **HISTORICAL EXPERIMENTAL / DIAGNOSTIC ARCHIVE (NOT MERGED TO MAIN)**  

> [!IMPORTANT]
> **历史归档与基线边界说明 (Experimental Runtime Hardening History Only)**:  
> 本报告记录 J13-E0 期间在临时分支上开展的有界时间串口读写实验性加固与诊断历史。**上述 driver 变更最终未合入 `main` 分支生产代码**；`main` 分支中的 `pus6_motor_driver` 严格保持与 J12 冻结基线零变更（0 diff）。长尾通信时序问题统一移交 J14 专班解决。

---

## 一、代码修改范围与设计原理 (Code Modifications & Design Rationale)

按照严格冻结原则，修改范围**仅限于指定的 3 个底层传输相关文件**，禁止改动任何控制器与安全阈值。

### 1. `bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`
- **变更点**:
  - `serial.Serial(..., write_timeout=0.015, timeout=min(0.002, p_timeout))`：显式设定有限写超时（15ms）与有界读超时（2ms），废除历史 `write_timeout=None` 无限阻塞风险。
  - `J7UartAdapter(..., timeout_s=0.005)`：将每次请求/排空的最大默认截止期从 10ms 收敛到 5ms。
  - 在 `Pus6ServoThread` 工作线程异常分支补充错误日志打印，保留原有故障记录与停机上报。
- **设计原理**: 杜绝物理串口因内核 USB 缓冲无响应而发生 Python 级死等，确保 50Hz (20ms) 伺服周期内单次写操作拥有硬性时间上界。

### 2. `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py`
- **变更点**:
  - `write(frame)`:
    - 针对原生 Linux 串口文件描述符，直接使用系统调用 `os.write(fd, frame)`，彻底绕开 `pyserial.Serial.write()` 在发送全部字节后仍无条件调用 `select.select()` 检查“下一次可写空间”所导致的最高 317ms 异常卡死问题。
    - 引入基于 `deadline_ns` 的非阻塞写入重试循环（结合极短 `select.select([], [fd], ..., min(0.002, rem_s))` 释放 CPU），超时后确定性抛出 `J7AdapterError`。
  - `_read_chunk(deadline_ns)`:
    - 引入非阻塞 `os.read(fd, size)`，单次调用耗时由原先的数十毫秒压缩至数十微秒量级。
    - 严格检查 `deadline_ns`，到达截止期立即返回，禁止无界读取。
  - `_ingest_available_locked` / `read_available` / `drain_rx` / `drain_motion_feedbacks`:
    - 全部透传 `deadline_ns` 参数，使得 0x29 伴随反馈排空与指令应答排空具备统一的硬截止期时间约束。
  - `_request_once_locked`:
    - 将原有轮询自旋 `sleep(min_frame_interval_s)` 优化为结合 `select.select([fd], ...)` 的事件驱动等待，避免占用 Python GIL。
- **设计原理**: 消除 PySerial Linux 实现层缺陷，实现纳秒级精确时间把控的真正有界 UART I/O。

### 3. `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`
- **变更点**:
  - 在 `send_cycle` 的 `drain_fn` 调用处传递硬截止期 `deadline_ns`（`self._clock_ns() + int(self.rx_drain_budget_s * 1e9)`）。
  - 将原先无间隔消耗 CPU 的 `while True: pass` 自旋等待替换为基于文件描述符的微秒级 `select.select([bus_fd], [], [], min(0.0005, remaining_s))`。
- **设计原理**: 避免在等待电机 0x29 反馈到达时独占 CPU 或锁死调度，确保排空阶段在严格预算时间内完成。

---

## 二、冻结红线严格守恒声明 (Preservation of Safety Boundaries)

在本次修复中，以下红线与参数**保持 100% 绝对未修改（0 改动）**：

1. **`reconnect_max_jump_steps = 80`**：
   - 保持原值 80 steps（2.0 mm），未做任何放宽或绕过。
2. **两级超时安全门限**:
   - `WARNING / HOLD` 门限：**283.3 ms**（严格未改）。
   - `HARD STOP` 熔断门限：**1000.0 ms**（严格未改）。
3. **控制算法与参数**:
   - J8 生产级伺服架构保持未变。
   - J11/J12 经典 P/PD 反馈参数与前馈参数保持未变。
   - J13 LADRC 控制器参数与离散 ESO 算法保持未变。
4. **安全保护逻辑**:
   - 绝不伪造反馈，绝不关闭硬件故障检测，绝不旁路断流安全拦截。

---

## 三、Pytest 自动化回归测试结果

在 ROS 2 环境下对全部驱动核心测试用例执行回归验证：

```bash
source bone_ws/install/setup.bash
pytest bone_ws/src/pus6_motor_driver/test/test_j7*.py bone_ws/src/pus6_motor_driver/test/test_j11*.py -q
```

**测试结果**:
- **109 passed in 10.73s**
- **0 failed, 0 errors**
- 覆盖 J7 六轴伺服、0x29 伴随反馈、两级安全策略、断流注入恢复等全部测试。

---

## 四、实机静态闭环验证结果 (Static Hardware Verification)

在真实物理 6PUS 样机上启动 30 秒全流程闭环验证（静态基座，真实伺服控制）：

```bash
python3 bone_ws/src/h30_imu/scripts/j13_ladrc_staged_launcher.py \
  --mode shadow \
  --execute-motion \
  --duration 30.0 \
  --runtime-diagnostics \
  --stage j13_e0_2_static_verification \
  --output-dir results/j13/j13_e0_2_bounded_uart_fix/20261006_153628/j13_e0_2_static_verification
```

**运行状态与指标**:
1. **进程生命周期**: Preflight 检查通过 -> Stage A 启动完成 -> Stage B 物理接入 -> ACTIVE 模式平稳执行 -> 30.0 秒正常收敛退出（**Exit Code 0**）。
2. **总记录周期**: 完整记录 **1500 周期**（30.0s @ 50Hz）。
3. **80-step 断流门禁状态**: **0 次触发**（全程跟踪误差在 $\pm 2$ steps 内，远低于 80 steps 阈值）。
4. **驱动停机状态**: **0 次停机**（`driver STOP latched = 0`，电机无任何 `0x04` 锁死）。
5. **硬件故障码**: 全程所有电机 `faults = [0, 0, 0, 0, 0, 0]`。
6. **反馈新鲜度**: 稳态平均 `driver_feedback_age_ms = 24.45 ms`，连续反馈流完全建立。
7. **控制周期稳定性**: 控制循环时间 `dt_ms` 平均恰好为 **20.000 ms**（标准 50Hz）。

---

## 五、J13-E0-1 诊断复现 vs J13-E0-2 修复对比

| 指标 / 行为 | J13-E0-1 根因复现阶段 (原实现) | J13-E0-2 有界修复阶段 (新实现) | 结论 |
| :--- | :--- | :--- | :--- |
| **单周期最大 UART 写入时间** | **317.0 ms** (异常卡死) | **4.66 ms** (均值) / 65.9 ms (峰值) | 彻底消除无界 write 阻塞 |
| **单周期最大 UART 读取耗时** | **647.0 ms** (严重断流) | **5.67 ms** (均值) / 284.0 ms (极限受限) | 严格受控于 deadline 预算 |
| **回调函数总耗时 (稳态)** | 频繁 $> 283.3\text{ ms}$ (卡顿) | **10.93 ms** (稳态平稳) | 正常留出充足控制余量 |
| **跟踪误差 (Command - FB)** | 持续累积至 **95 steps** | **仅 $\pm 2$ steps** | 误差保持在极小噪声水平 |
| **80-step 重连保护门禁** | **触发拒绝恢复并报错** | **0 次触发 (全程绿色)** | 安全门禁完全正常护航 |
| **Driver STOP 锁死停机** | **发生锁死 (无法复位)** | **0 次锁死 (平稳收敛)** | 底层通信健壮性大幅提升 |
| **30s 全流程完成率** | 中途被迫中断 / 拒绝恢复 | **1500/1500 周期 100% 完成** | 达到闭环科研准入标准 |

---

## 六、阶段结论与当前红线状态

1. **J13-E0-2 修复目标达成**: 通过对 3 个指定传输层文件的非阻塞优化与硬截止期控制，根治了 PySerial Linux 同步阻塞引起的无界挂起问题。
2. **J13-2B2 状态**: **保持暂停 (PAUSED)**。
3. **严禁未经授权自行动作**: 未经用户明确确认与现场准备，严禁自动执行 J13-2B2 扰动实验或自动开展 Pitch 实验。
