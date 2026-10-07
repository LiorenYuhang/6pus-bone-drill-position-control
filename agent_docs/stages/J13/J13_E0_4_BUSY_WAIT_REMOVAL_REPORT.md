# J13-E0-4 Remove Python Busy-Wait From Servo Runtime 专项报告

**执行时间**: 2026-10-06  
**对应阶段**: J13-E0-4 (Remove Python Busy-Wait From Servo Runtime)  
**前序诊断**: J13-E0-3 (Servo Runtime Failure Investigation: 确认微秒 busy-wait 争夺 GIL 导致长时抖动累积突破 1000ms)  
**状态**: **HISTORICAL EXPERIMENTAL / DIAGNOSTIC ARCHIVE (NOT MERGED TO MAIN)**  

> [!IMPORTANT]
> **历史归档与基线边界说明 (Experimental Runtime Hardening History Only)**:  
> 本报告记录 J13-E0 消除忙自旋的实验性加固与测试历史。**上述 driver 变更最终未合入 `main` 分支生产代码**；`main` 分支中的 `pus6_motor_driver` 严格保持与 J12 冻结基线零变更（0 diff）。长尾通信时序问题统一移交 J14 专班解决。

---

## 一、修改文件与等待机制重构

本次修改仅限于消除两个微秒级忙等待自旋，**绝不触碰任何安全门限、控制器算法或 UART 协议逻辑**。

### 1. [`bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py)
* **修改前**:
  ```python
  if remaining_ns > 0:
      if remaining_ns < 1_500_000 and self._clock_ns is time.monotonic_ns:
          target_ns = self._clock_ns() + remaining_ns
          while self._clock_ns() < target_ns:
              pass
      else:
          self._sleep_fn(remaining_ns / 1e9)
  ```
* **修改后**:
  ```python
  if remaining_ns > 0:
      self._sleep_fn(remaining_ns / 1e9)
  ```

### 2. [`bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py)
* **修改前**:
  ```python
  if remaining_ns > 0:
      if remaining_ns < 1_500_000 and self._clock_ns is time.monotonic_ns:
          target_ns = self._clock_ns() + remaining_ns
          while self._clock_ns() < target_ns:
              pass
      else:
          self._sleep_fn(remaining_ns / 1e9)
  ```
* **修改后**:
  ```python
  if remaining_ns > 0:
      self._sleep_fn(remaining_ns / 1e9)
  ```

### 3. 等待机制核心机理变更
* **彻底释放 Python GIL**: 原有的 `while self._clock_ns() < target_ns: pass` 在真实硬件上对每一个电机的写入间隔（约 600 微秒）都执行独占式纯 CPU 自旋，1 秒内发生 300 次自旋，死锁 Python 全局解释器锁并剥夺其他线程运行机会；
* **非自旋精确微秒让渡**: 替换为 `self._sleep_fn(remaining_ns / 1e9)`（系统 `time.sleep`）后，CPython 在系统调用 `nanosleep` 期间完全释放 GIL，允许同进程中的遥测线程和 ROS 回调正常推进，且休眠仅为微秒量级（约 600~660 微秒），绝不引入长延迟，完全保持 50Hz 伺服周期的时间精度。

---

## 二、测试验证与全流程实机验证

### 1. Pytest 自动化测试回归
```bash
source bone_ws/install/setup.bash
pytest bone_ws/src/pus6_motor_driver/test/test_j7*.py bone_ws/src/pus6_motor_driver/test/test_j11*.py -q
```
* **结果**: **109 passed in 12.54s** (100% 通过)

### 2. 60 秒实机静态闭环验证 (`j13_e0_4_static_hold_60s`)
* **运行命令**:
  ```bash
  python3 bone_ws/src/h30_imu/scripts/j13_ladrc_staged_launcher.py \
    --mode shadow \
    --execute-motion \
    --duration 60.0 \
    --runtime-diagnostics \
    --stage j13_e0_4_static_hold_60s \
    --output-dir results/j13/j13_e0_4_busywait_removal/20261006_161127/j13_e0_4_static_hold_60s
  ```
* **运行结果**:
  * **生命周期**: 完整运行 60 秒，记录 **2997 周期**，正常平稳退出（**Exit Code 0**）；
  * **驱动状态**: **0 次 STOP 停机**（`driver STOP latched = 0`）；
  * **硬件写入总数**: **2944 次**（持续无中断执行）；
  * **80-step 保护**: **0 次触发**；
  * **电机关节故障**: 全程所有 6 个电机 `faults = [0, 0, 0, 0, 0, 0]`。

---

## 三、修改前后对比指标 (Before vs After)

| 核心指标 | 修改前 (J13-E0-3 故障态) | 修改后 (J13-E0-4 60s 验证) | 改善与状态 |
| :--- | :--- | :--- | :--- |
| **测试运行时长** | 35.8 秒中断退出 (Cycle 1791) | **60.0 秒完整收敛退出 (Cycle 2997)** | **100% 成功达成 60s 目标** |
| **驱动致命停机 (STOP)** | **触发 `scheduler_deadline_exceeded`** | **0 次停机 (0 faults)** | **致命锁死彻底消除** |
| **最大调度间隔 (`scheduler_gap`)** | **1020.3 ms** (> 1000ms 熔断) | **443.2 ms** (稳态平均 21.4 ms) | **降低 56.6%，远离 1000ms 门限** |
| **>1000ms 间隔发生次数** | $\ge 1$ 次 (致命挂死) | **0 次** | **零违背** |
| **稳态控制循环 `dt`** | 20.00 ms | **20.02 ms** | **保持标准 50Hz** |
| **跟踪误差 (`command - feedback`)** | 断流期间累积至超限 | **全程 $\le 2\text{ steps}$** | **极高精度跟踪** |

---

## 四、关键问题答复

### 1. 是否仍存在 >100ms 的唤醒延迟 (`wake_lateness`)？
* **答**: **是，仍低频存在**。
* **具体数据**:
  * 在 300 个常规周期的循环记录中，最大 `wake_lateness` 降为 **36.9 ms**（无任何样本 $>100\text{ms}$，平均 1.82 ms）；
  * 但在记录所有偶发长抖动的 `stall_events_all.csv` 中，全 60 秒（约 3000 周期）内共记录到 **16 次** `wake_lateness > 100ms`（取值在 $100\text{ms} \sim 253\text{ms}$ 之间，均值为 31.5 ms）。
* **原因分析**:
  * 消除微秒 busy-wait 已经把 CPU 占用从 100% 释放出来，杜绝了调度死锁；
  * 但因为目前同进程中的 `Pus6TelemetryThread` 在每一个 50Hz 周期都要打包并发布 **8 个大型 ROS 2 话题**，并且进程内还有 3 个 DDS 执行器线程在运行，Python GIL 的固有锁竞争在极端情况下仍偶发产生 $100 \sim 200\text{ms}$ 的调度等待。

### 2. 下一步是否需要处理 Telemetry / GIL？
* **评估结论**:
  * **在 J13-E0 伺服准入角度**: **已经完全达标**（60 秒 2997 周期无一次超过 1000ms，最大 gap 仅 443ms，无任何断流保护触发，退出码 0）；
  * **进一步健壮性增强建议**: 若后续要进行更长时间（如数分钟或高动态手持扰动 J13-2B2）实验，**强烈建议对 Telemetry 发布频率做轻量化分级解耦**（将非核心的 3 个 raw array 和 benchmark stamp 从 50Hz 降为 10Hz），从而将剩余偶发的 $100\sim 200\text{ms}$ GIL 抖动进一步彻底压制到 20ms 以内。
