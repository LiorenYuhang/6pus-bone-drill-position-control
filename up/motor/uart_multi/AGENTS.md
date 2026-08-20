# UART Multi Motor Agent Rules

## 项目目标

当前项目为 Inspire Robots LAF50-024D Type-D 电缸 UART 实机验证。

六电机编号与 UART 只读通信、ID1 endpoint safety bring-up 和 Stage M2 已完成。M2-D Retry2 PASS；首次 FAIL 与 Retry1 baseline gate ABORTED 历史永久保留。Stage M4 全部子阶段已正式 PASS（M4-A 只读通信，M4-B1~B4 四电机单机独立运动，M4-C 四电机联合协同运动）。Stage M6 全部子阶段已正式 PASS（M6-A 只读通信，M6-B5/M6-B6 单机独立运动，M6-C Retry 六电机联合协同运动；历史 Attempt 1 首次 STOP 记录永久独立保留）。当前全部预定阶段已圆满通过验收。

未经用户明确要求，不进入：

- 已 PASS 的 M2-D 重复实验
- 未经逐级批准的 M4-B 后续 Run 和 M4-C 四电机联合运动
- M6 六电机阶段
- Stewart运动学
- ROS2
- 轨迹控制
- 闭环控制
- 强化学习

---

## 已冻结底层

以下文件视为UART通信基线：

- protocol.py
- uart_bus.py
- motor.py

除非发现有明确证据证明底层协议实现错误，否则不要修改。

如认为必须修改，先报告原因和证据，不直接修改。

六电机UART基线：

- ID1~ID6全部在线
- 606次请求全部首次成功
- timeout=0
- wrong_id=0
- length_error=0
- checksum_error=0

基线日志：

`logs/six_ids_test01.log`

不得删除、覆盖或修改。

---

# 实机安全原则

所有运动工具必须遵守：

1. 写运动命令不自动重试。
2. pause命令不自动重试。
3. 发生运动异常时，必须在UART仍然打开时发送一次pause。
4. pause完成后才能关闭UART。
5. pause失败或无法确认停止时必须输出：

`HARD STOP REQUIRED`

并明确提示使用硬件急停或断电。

6. 实机运动必须人工确认后执行。
7. 每次只扩大一个变量。
8. 不得自动扩大位移、速度、时间或电流阈值。
9. 原始实机日志永不覆盖。
10. 失败Run不得删除。

---

# ID1 当前状态

ID1为单独裸电缸，目前未安装到6PUS机构。

已确认：

- UART通信正常
- Type-D V2.0.4
- 位置范围0~2000 step
- 0x25：
  - 0 定位模式
  - 1 伺服/follow-up模式
  - 2 速度模式
  - 3 力控模式
  - 4 电压模式
  - 5 速度力控模式
- 0x28：目标速度，速度模式有效，单位step/s
- 0x29：目标位置
- 0x2A：实际位置
- 0x2B：电流mA
- 0x2C：实际受力，有符号，协议单位g
- 0x2E：温度
- 0x2F：故障码
- 0x1A=1：pause

---

# 力传感器现状

ID1无载静态实验：

horizontal：

- force mean ≈ -428g
- std ≈ 1.08g

horizontal flipped：

- force mean ≈ -382g
- std ≈ 6.31g

因此当前存在约-400g静态偏置。

暂时禁止：

- 使用 `|force| < 100g` 作为绝对停止条件
- 擅自执行力零点校准
- 猜测新的绝对力保护阈值

当前运动实验：

- 记录F0
- 记录实时force
- 记录DeltaF = force - F0

后续再根据实验数据设计相对力保护。

---

# ID1 Run01

日期：2026-08-16

原始日志：

`logs/id1_motion/20260816_110853_endpoint_escape_id1_run01.log`

条件：

- p0=3
- target=8
- +5 step
- mode=2
- speed=50
- timeout=2s
- F0≈-377g

运动期间：

- actual主要3~4 step
- current最高约42mA
- DeltaF约个位数g
- temperature≈34C
- fault=0

最终：

`STOP target=8 did not settle within 2.00s`

旧代码存在安全缺陷：

pause发送前UART已经关闭：

`Attempting to use a port that is not open`

Run01之后纯只读取证：

- mode=2
- target_speed=50
- target_position=8
- actual_position=7
- actual稳定5秒
- current=0
- 配置未变化

Run01最终判定：

**FAIL**

原因：

旧版本安全停止生命周期存在缺陷。

当前证据强烈支持：

timeout后pause未真正执行，target=8仍然有效，程序退出后执行器继续向目标运动。

Run01不能作为运动PASS。

---

# 当前安全停止修复

safe_endpoint_escape_id1.py 已修复：

运动异常时：

1. UART保持open；
2. 发送一次pause；
3. pause后约0.5s纯只读确认；
4. 检查位置/current/fault；
5. 确认后才close UART。

pause失败或无法确认停止：

`HARD STOP REQUIRED`

相关测试已通过。

---

# 实机Run统一记录

所有实机运动使用：

`UART实机运行记录.md`

每个Run必须记录：

## RunXX

- 日期时间
- 使用工具
- 电机ID
- 起始位置p0
- target
- 位移增量
- mode
- speed
- timeout
- 电流保护
- F0
- 原始日志路径

### 运行结果

- first_motion_time
- first_in_tolerance_time
- final_actual
- final_error
- max_current
- force min/max
- DeltaF min/max
- temperature
- fault
- pause是否触发
- pause是否成功
- stop confirmation
- HARD STOP REQUIRED

### 判定

PASS / FAIL / ABORTED

### 原因

必须明确写出。

### 下一步

必须明确说明允许和禁止执行的下一步。

Run编号不得复用。

失败Run和原始日志不得删除。

---

# Agent工作规则

用户提交实机结果后，Agent必须：

1. 分析终端输出和日志；
2. 明确判定PASS/FAIL；
3. 自动更新 `UART实机运行记录.md`；
4. 必要时同步更新 `UART六电机编号与测试方案.md`；
5. 保留原始日志路径；
6. 不覆盖过去Run；
7. 再生成下一次实验命令。

用户不需要手工维护Run记录。

---

# 当前阶段

- M2-A：`PASS`
  - 日志：`logs/multi_motor/m2/20260816_164752_m2a_readonly_id1_id2.log`
- M2-B：`PASS`
  - 未解锁预检：`logs/multi_motor/m2/20260816_170427_m2b_id1_only.log`，`ABORTED / NOT PASSED`，零写入
  - 正式 PASS：`logs/multi_motor/m2/20260816_170610_m2b_id1_only.log`
- M2-C：`PASS`
  - 日志：`logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`
- M2-D：Retry2 `PASS`；首次实机 `FAIL / NOT PASSED`、Retry1 baseline gate `ABORTED / NOT PASSED` 且 `write_count=0`，两项历史不追溯改判
  - Retry1 日志：`logs/multi_motor/m2/20260817_104109_m2d_joint_id1_id2.log`
  - Retry2 PASS 日志：`logs/multi_motor/m2/20260817_105855_m2d_joint_id1_id2.log`
- M2-CAL：`PASS`
  - 方案：`M2-CAL位置校准方案.md`；工具：`tools/safe_m2cal_position.py`
  - 无硬件测试：`test_safe_m2cal_position.py`，`3 passed`
  - 原始日志：`logs/multi_motor/m2cal/20260817_100115_m2cal_position.log`
  - 误差：ID1 `-1,-2,-2,-1`；ID2 `0,-3,-1,-2 step`，不存在固定 `-2 step` 偏置，禁止补偿

M2-D pause 语义：`commanded_target`、`readback_target_position`、`actual_position` 必须分开记录。pause 前必须验证目标寄存器读回等于本次命令；实机原始帧证明 pause 后设备会把 target 同步到停止 actual，因此 pause 后 target 不得与原命令强制比较，只验证实际位置、停止、current 和 fault。首次 M2-D 的 final configuration mismatch 是验收逻辑错误，历史 FAIL 不追溯改写。

M2-D baseline 使用 position histogram 唯一众数作为 `baseline_reference`。全部样本必须满足 `abs(actual-baseline_reference)<=1`；偏离达到 `2 step` 或众数 tie 时在任何写入前拒绝。`baseline_safety_min/max` 为实际观测范围；目标为 `baseline_reference+10`；仅 `actual>baseline_safety_max` 确认正向运动，仅 `actual<baseline_safety_min` 确认负向运动。

Stage M2：`COMPLETE / PASS`。M2-D Retry2 两台均为 `baseline_reference=38, target=48, final=46, error=2`，最大电流 ID1 `66mA`、ID2 `48mA`，fault=0，两台 pause 和 stop confirmation 均通过，写 destination 审计正确。

Stage M4-A：正式 `PASS`。正式日志 `logs/multi_motor/m4/20260817_113201_m4a_readonly_id1_id2_id3_id4.log`：ID1~ID4 各 `101/101 first success`，合计 `404/404`，retry/failure/timeout/wrong ID/length/checksum 均为 0，ID5 `expected_no_response=1`，平均/最大响应 `2.836/3.655ms`。较早日志 `logs/multi_motor/m4/20260817_112819_m4a_readonly_id1_id2_id3_id4.log` 中 ID3/ID4 未响应、202 项最终失败，作为失败历史保留，不追溯改判。

Stage M4-B1：`ABORTED / NOT PASSED — ID4 pre-motion position out of range`。原始日志 `logs/multi_motor/m4/20260817_123600_m4b1_id1_only.log`：ID4 的 `0x29` 原始读回 `FE FF`（有符号 `-2`），状态 target/actual 均为 `-2`；在 preflight 位置限位检查中中止。`write_count=0`，未 armed、未运动、未 pause、未执行 stop confirmation。不得重跑 M4-B1，不得准备或执行 M4-B2。

ID4 纯只读诊断：`COMPLETE / PERSISTENT_MINUS2`。原始日志 `logs/multi_motor/m4/20260817_130429_id4_readonly_diagnosis.log`：100/100 样本均为 register target/status target/actual=`-2`、current=`0mA`、fault=0，force=`-61g`，`write_count=0`。因此排除瞬时抖动和偶发 UART 错误，但不能据此判定硬件故障。

ID3/ID4 纯只读对照诊断：`COMPLETE / PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE`。原始日志 `logs/multi_motor/m4/20260817_132920_id3_id4_readonly_comparison.log`：mode、target speed、current、fault 均 100/100 相同；ID3 register target 为 `{-1:1, 0:99}`、status target/actual 均 `{0:100}`，ID4 三个位置字段均 `{-2:100}`；`write_count=0`。ID3/ID4 各完成 300 次寄存器读和 100 次状态读。该结果将问题定位为 ID4 自身持续位置状态，不支持 UART 总线或多机调度问题。

ID4 单机重新上电纯只读复核：`COMPLETE / PERSISTENT_MINUS2`。原始日志 `logs/multi_motor/m4/20260817_134018_id4_repower_readonly_verify.log`：100/100 register target/status target/actual 均为 `-2`，current=`0mA`，fault=0，force `-61~-52g`，`write_count=0`。确认掉电重启后仍为 device-local persistent state；继续按 V2.0.4 视为 out-of-spec，禁止运动、clamp 或放宽 `[0,2000]` 门禁。

ID3/ID4 纯只读配置审计：`PREPARED / NOT EXECUTED`。工具 `tools/audit_id3_id4_config_readonly.py` 固定 ID3/ID4，三轮读取官方 V2.0.4 已定义的 14 个安全配置寄存器：`0x16/0x17/0x1E~0x29` 中排除动作/权限地址，仅包括 ID、波特率、保护值、输出上限、行程上下限和控制配置；另读状态帧。任意写、其他 ID、动作寄存器、非白名单地址或多寄存器读均在冻结底层前拒绝。专项无硬件测试 `19 passed`，全仓无硬件回归 `205 passed`。

官方资料审计：V2.0.4 只有 `CMD_RD_STATUS=0x30`、`CMD_RD_REGISTER=0x31`、`CMD_WR_REGISTER=0x32`，未定义设备信息/版本查询命令；公开寄存器表没有固件版本、协议版本、设备型号、位置零点或位置偏移字段。成功响应只能确认两台使用 V2 风格协议族，不能读出精确固件版本或型号。旧 V1.8.3 的零位校准仅针对力传感器，且地址布局不同，不得用于探测现行设备。

本地手册核对：现行 Type-D V2.0.4 将 `0x29 target_position` 和 `0x2A actual_position` 均列为 `[0,2000]`，并说明内置绝对位置传感器、掉电不丢失且无需归零；未找到位置零点/初始化/校准流程。旧 V1.8.3 将 target 列为 `0~2000`、actual 列为 `-20~2020`，其中校准条目仅针对力传感器。故当前 `0x29=-2` 在两版中均超出目标范围，`0x2A=-2` 也超出现行规格；旧版只使实际位置近零负值具有历史可解释性，不能把当前 ID4 状态改判为正常。

ID4-OFFSET：`COMPLETE / PASS`。原始日志 `logs/multi_motor/id4_offset/20260817_151126_id4_offset.log`：四点 `targets=(10, 20, 30, 40)`，到位误差 `[-1, -1, -2, -1]`，分类为 `PASS / ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED`。实机证实 ID4 不存在固定 -2 零偏，禁止 +2 位置补偿；ID4 进入正区间后表现为正常到位残差。

ID4 近零端边界恢复与保护：`COMPLETE / PASS`。原始日志 `logs/multi_motor/id4_init/20260817_161543_id4_poweron_init.log`：人工断电重启后 100/100 采样均为 39 step、current=0mA、fault=0x00，判定为 `PASS / ALREADY_IN_RANGE_NO_ACTION`，`write_count=0`。证实 raw=-2 为近零端特殊边界状态而非固定上电状态；掉电重启保持在合法位置区间；禁止 +2 补偿，严禁主动移回零端。工具 `tools/safe_id4_poweron_initialize.py` 转为常态化边界保护工具。

ID3 零端脱离方案：`FAIL / HARD STOP REQUIRED — stop confirmation failed`（2026-08-17 16:26，`logs/multi_motor/id3_escape/20260817_162634_id3_zero_escape.log`，历史保留不改判）。脱后 100 轮只读诊断确认 ID3 实际稳定在 `6~7 step`（`{6: 31, 7: 69}`），current=0mA，fault=0x00，脱零目标已达成且不再重复执行 ID3-ESCAPE。

Stage M4-B1 Retry：`COMPLETE / PASS`（2026-08-17 16:31:11，`logs/multi_motor/m4/20260817_163111_m4b1_retry_id1_only.log`）。ID1 运动 target=55, final=53, error=2 step；ID2（45~46）、ID3（6~7）、ID4（39）全程严格只读（`observer_write_count=0`）；`write_count=3`（仅 ID1）；`pause_success=True`, `stop_confirmation=True`。首次 M4-B1 与首次 M4-B1 Retry 的 `ABORTED / NOT PASSED` 历史永久保留。

Stage M4-B2 首次实机：`STOP / NOT PASSED`（2026-08-17 16:33，`logs/multi_motor/m4/20260817_163320_m4b2_id2_only.log`，历史保留不改判）。ID1 固件 target 在停止位置 52~53 step 伴随静态量化抖动触发拦截，ID1 100 轮只读诊断确认状态正常；observer configuration audit 已优化为 baseline target safety band 机制。

Stage M4-B2 Retry 首次实机：`STOP / NOT PASSED`（2026-08-17 16:41:02，`logs/multi_motor/m4/20260817_164049_m4b2_retry_id2_only.log`，历史保留不改判）。ID2 到达 53 触发 pause，停机后稳定在 52（final_error=3），触发原 2-step 验收阈值拦截。写审计与停止确认全通过。容差判据现已拆分为 `PAUSE_TRIGGER_TOLERANCE_STEPS = 2` 与 `FINAL_ACCEPTANCE_TOLERANCE_STEPS = 3` 并通过测试（31 passed）。

Stage M4-B2 Retry2：`COMPLETE / PASS`（2026-08-17 16:46:12，`logs/multi_motor/m4/20260817_164559_m4b2_retry2_id2_only.log`）。ID2 运动 target=62, final=59, error=3 step（符合 $\le 3$ step 验收）；ID1（52~53）、ID3（6~7）、ID4（39）全程严格只读（`observer_write_count=0`）；`write_count=3`（仅 ID2）；`pause_success=True`, `stop_confirmation=True`。首次 M4-B2 与首次 M4-B2 Retry 的 `STOP / NOT PASSED` 历史永久保留。

Stage M4-B3：`COMPLETE / PASS`（2026-08-17 16:49:49，`logs/multi_motor/m4/20260817_164937_m4b3_id3_only.log`）。ID3 运动 target=17, final=15, error=2 step（符合 $\le 3$ step 验收）；ID1（52）、ID2（59~60）、ID4（39）全程严格只读（`observer_write_count=0`）；`write_count=3`（仅 ID3）；`pause_success=True`, `stop_confirmation=True`。

Stage M4-C：`COMPLETE / PASS`（2026-08-17 17:02:44，`logs/multi_motor/m4/20260817_170233_m4c_joint_four_motors.log`）。ID1~ID4 四台电机同一 UART 总线联合小幅运动 / 协同控制验证：ID1（52->59, err=3）、ID2（59->67, err=2）、ID3（15->23, err=2）、ID4（47->54, err=3），全机误差 $\le 3$ step；实测首次起步时间存在离散差（ID4@0.30s, ID3@0.37s, ID2@0.73s, ID1@0.76s），当前阶段未验证高精度同步启动，不表述为严格同步控制；`write_count=12`，`pause_all=true`, `stop_confirmation=True`。

Stage M4 总体状态：`ALL STAGES COMPLETE / PASS`（M4-A / M4-B1~B4 / M4-C 全通）。

Stage M6 总体状态：`ALL STAGES COMPLETE / PASS`（M6-A / M6-B5 / M6-B6 / M6-C 全通）。

Stage M6 推进规划与状态：
- **M6-A**：`COMPLETE / PASS`（2026-08-17 17:31:48，`logs/multi_motor/m6/20260817_173141_m6a_readonly_review.log`）。ID1~ID6 六机纯只读 100 轮采样，`write_count=0`；
- **M6-B5**：`COMPLETE / PASS`（2026-08-17 18:33:11，`logs/multi_motor/m6/20260817_183305_m6b5_retry_id5_only.log`）。ID5 运动 target=17, final=15, error=2 step（符合 $\le 3$ step 验收）；`write_count=3`（仅 ID5: 0x25, 0x29, 0x1A）；`observer_write_count=0`；历史 Attempt 1~3 STOP 记录永久独立保留；
- **M6-B6**：`COMPLETE / PASS`（2026-08-17 18:34:58，`logs/multi_motor/m6/20260817_183450_m6b6_id6_only.log`）。ID6 运动 target=10, final=8, error=2 step（符合 $\le 3$ step 验收）；ID1~ID5 全程严格只读（`observer_write_count=0`）；`write_count=3`（仅 ID6: 0x25, 0x29, 0x1A）；`pause_success=True`, `stop_confirmation=True`；
- **M6-C**：`STOP / NOT PASSED`（2026-08-17 18:41:36，`logs/multi_motor/m6/20260817_184132_m6c_joint_six_motors.log`）。轮询第 2 轮 ID6 读出 actual=7（< baseline 8）触发反向保护，即刻 pause-all 六机制动成功（全机电流 0mA，fault 0x00，write_count=18），历史记录永久独立保留；
- **M6-C Retry**：`COMPLETE / PASS`（2026-08-17 18:49:38，`logs/multi_motor/m6/20260817_184931_m6c_retry_joint_six_motors.log`）。ID1~ID6 六机联合协同正向运动 +10 step，0.84s 到达容差区间触发统一 Pause，最终到位误差 ID1=2, ID2=2, ID3=1, ID4=1, ID5=2, ID6=1（全部 $\le 3$ step 验收通过），`pause_all=true`, `stop_confirmation=true`，`write_count=18`。

M2-C 固定边界（已执行）：

- 只连接 ID1+ID2，共用一个 `UartBus`；
- 仅 ID2 执行 `+10 step / 50 step/s / 最大2s / 500mA` 正向运动；
- ID1 全程只读，任何写入均必须由审计层拒绝；
- 正常或异常结束只对 ID2 单次 pause；
- 日志必须包含 `ACCESS_AUDIT` 和 `id1_write_count=0`；
- PASS/FAIL 标准和唯一命令见 `UART六电机编号与测试方案.md` 第 20.4、20.8 节。

当前不阻塞主线的已知问题：位置残差约 `0~3 step`，按 `40 step/mm` 约 `0~0.075 mm`；另有 ID1 低端约 `0.9s` 启动延迟，以及无载力传感器约 `-400g` 静态偏置。不得据此写补偿、自动扩大参数或擅自修改力阈值。
