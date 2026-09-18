# J11 One-Shot Runtime Validation: Static Home Hold Audit Report

## 1. 实验真实工况定义 (Experiment Definition)
- **实验重定义**: **`STATIC HOME HOLD VALIDATION`**（静态工作原点保持验证）。
- **执行事实**:
  - 用户在从头到尾的 182.52 秒运行中，**未对 Base 施加任何手动姿态扰动**；
  - Base 全程保持静止：$\text{Base SO}(3)_{\max} \approx 0.052^\circ$，均值 $\approx 0.021^\circ$；
  - Platform 基本保持静止：$\text{Platform SO}(3)_{\max} \approx 0.106^\circ$，Tilt 误差 $\le 0.042^\circ$（RMS $0.020^\circ$）；
  - 六轴电机始终停留在 `OPERATING_HOME` 附近，步数范围 `[997, 1001]` 步；
  - 指令与反馈跟踪误差始终保持 $\le 2$ 步；
  - **`slew_recovery_events = 0`**；
  - **`current_spikes >= 400mA = 0`**；
  - 电机电流极低：全量 8,887 个高频电流样本中最大峰值仅 **48 mA**（Axis 6），各轴平均电流 $\le 0.1\text{ mA}$；
  - **最终仍触发致命故障停机**: `J7 driver STOP latched: scheduler_deadline_exceeded`。

---

## 2. 核心直接结论 (Direct Experimental Proof)
本次实验提供了决定性的直接证据：
> **`scheduler_deadline_exceeded` 可以在完全没有 Base 扰动、没有明显机械运动、没有高电流负载、没有 slew recovery 介入的纯静止状态下独立发生。**

这确凿无疑地证明：
**`pus6_motor_driver_node` 的运行期调度器与反馈处理路径（J7 Driver runtime scheduler / feedback path）本身存在固有的时延抖动与停滞（stall）缺陷。**

---

## 3. 因果链严格判定 (Causal Chain Evaluation)

### 3.1 正式确认项 (CONFIRMED)
$$\text{scheduler stall} \longrightarrow \text{feedback\_age 线性积压} \longrightarrow \text{突破 283.3ms 阈值} \longrightarrow \text{scheduler\_deadline\_exceeded STOP}$$
- **证据级别**: **`CONFIRMED`（直接实验证实）**。
- **微观数据支撑**: 停机前最后 14 个周期（Cycle 9168~9181），驱动层反馈完全中断，反馈账期以精确 20.0ms 连续线性攀升（$12.3 \to 32.3 \to 52.2 \dots \to 292.3\text{ ms}$），在突破 283.3ms 看门狗后被底层驱动直接锁死。

### 3.2 强相关假说 / 部分支持项 (SUPPORTED / PARTIALLY CONFIRMED)
$$\text{scheduler stall} \longrightarrow \text{gap 累积} \longrightarrow \text{slew recovery 突发分发} \longrightarrow \text{瞬态电流反冲} \longrightarrow \text{overcurrent STOP}$$
- **证据级别**: **`SUPPORTED / HYPOTHESIZED`（历史运动工况下强相关，本次静止实验未触发）**。
- **依据与边界**:
  - 本次静止实验中：无运动 $\to$ 无 gap 扩大 $\to$ 无 Slew $\to$ 无电流激增（最大仅 48mA），因此静止工况无法证实该链路的后半段；
  - 历史扰动实验（如 Case C `python3_449248`）中：用户移动 Base 时遭遇 223.9ms 调度卡顿，导致 20 步 gap 积累并触发 Slew 恢复分发，随后因过流触发停机；
  - 因此，过流故障是调度卡顿在“存在外界动态扰动”时的一种**次生派生表现**，不能在静止实验中声称完全证明同一根因。

---

## 4. 后续工程优先级调整 (Adjusted Engineering Roadmap)
1. **优先解决 Driver runtime scheduler stall**：
   在驱动层解决 50Hz 调度卡顿（优化 Python 执行器、回调组隔离与串口 I/O 吞吐）；
2. **调度卡顿解决后，再进行受控运动验证**：
   在确认 50Hz 调度账期稳定（无 $>50\text{ms}$ 卡顿）的前提下，开展单次低速受控运动验证；
3. **针对性处理 overcurrent**：
   若调度问题解决后，在动态运动中仍偶发过流，再对 Slew 速度限制及过流单帧检测策略进行专项排查；
4. **当前冻结红线**：
   - **在 scheduler stall 未解决前，严禁进行新的 Base 手动扰动实验**；
   - 严禁修改 J7/J8 生产包代码及安全阈值；
   - 严禁擅自启动通电运动。
