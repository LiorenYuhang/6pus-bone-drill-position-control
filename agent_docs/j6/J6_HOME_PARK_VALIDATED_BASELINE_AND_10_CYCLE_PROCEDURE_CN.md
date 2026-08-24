# J6 HOME/PARK 已验证基线与十循环回归程序

## 1. 文档边界

本文档只归档软件回归基线和重复循环程序，不授权真实硬件执行，不修改控制逻辑、协议、安全门、ROS2 或 J5 冻结资产。

本次基线证据来自用户已确认的实机结果；本文档本身未执行硬件。

## 2. 冻结 commissioning 基线

| 项目 | 冻结值 |
| --- | --- |
| 控制模式 `0x25` | `0`，Positioning Mode |
| 速度参数 | `50 step/s` |
| `0x28` | 保持现有 speed-register 映射和读回校验 |
| `0x29` HOME 目标 | `[1000,1000,1000,1000,1000,1000]` |
| `0x29` PARK 目标 | `[0,0,0,0,0,0]` |
| standalone 控制周期 | `10 Hz` |
| target safety range | 保持现有 `[0,2000]` |
| feedback tolerance | 保持现有反馈容差 |
| watchdog | 保持现有 tracking/fault/overcurrent/communication 处理 |

已确认结果：

- HOME，mode=0，speed=50：PASS；
- PARK，mode=0，speed=50：PASS；
- HOME/PARK，speed=50：已重复通过（超过 10 个循环）；
- HOME/PARK，speed=60：FAIL，运行不稳定，可能触发 tracking watchdog；
- HOME/PARK，speed=100/200：FAIL，运行不稳定。

`speed=50` 是当前 commissioning 的已验证工作点，不是电机最大能力或额定速度上限。速度边界结果用于确定安全、可重复的调试基线，不得外推为电机性能上限。

高速度优化、50 Hz 改造和 watchdog 调参均不属于本基线。speed=60 及以上不纳入当前 commissioning 基线。

## 2.1 速度边界记录

| 速度 | HOME/PARK 结果 | 基线结论 |
| ---: | --- | --- |
| 50 | PASS，重复运行超过 10 个循环 | **Validated commissioning point** |
| 60 | FAIL，不稳定，可能触发 tracking stop | 不采用 |
| >=100 | FAIL，不稳定 | 不采用 |

这些结果只描述当前 mode=0、standalone 10 Hz commissioning 配置下的可重复性，不代表电机的最大可用速度范围。任何更高速度实验都必须单独设计、单独授权、单独记录。

## 3. 每次单循环定义

一个循环定义为：

```text
PARK target [0]*6
    -> HOME target [1000]*6
    -> settle / stop confirmation
    -> PARK target [0]*6
    -> settle / stop confirmation
```

若当前已经处于 PARK，可先执行一次 HOME，再执行 PARK，作为第 1 个完整循环。不得自动改变起始参考、offset、geometry 或目标限位。

## 4. 十循环 Mock/FakeBus 回归程序

该程序用于软件回归和日志结构检查，不连接串口：

1. 固定 `hardware_mode=mock` / `real_hardware=False`。
2. 固定 `mode=0`、`speed=50`、`control_rate_hz=10`。
3. 对 `cycle=1..10` 重复执行：
   - HOME `[1000]*6`；
   - 检查 `safety_aborted == false`；
   - 检查 `q_target_raw == [1000]*6`；
   - 检查最终误差和 stop confirmation；
   - PARK `[0]*6`；
   - 检查 `safety_aborted == false`；
   - 检查 `q_target_raw == [0]*6`；
   - 检查最终误差和 stop confirmation。
4. 每个方向均检查写审计：
   - 六个配置写入的 `0x25` payload 首值为 `0`；
   - 配置阶段没有 `0x29`；
   - 轨迹目标写入只属于 `0x29`；
   - 目标向量与 HOME/PARK 目标完全一致；
   - `unexpected_writes == []`。
5. 记录每循环的 cycle ID、起始反馈、目标、最终反馈、最大误差、样本数、写入分类和异常原因。
6. 任一循环失败即停止后续循环，报告 `FAIL`；不得自动重试失败循环。

## 5. 十循环判定

只有以下条件全部满足，软件回归才标记 `PASS`：

- HOME 10 次均完成；
- PARK 10 次均完成；
- 每次目标分别严格为 `[1000]*6` 和 `[0]*6`；
- 每次 mode 配置均为 `0`；
- 每次配置无隐藏 `0x29` 写入；
- 每次目标写入均为 `0x29`；
- 无 unexpected write；
- 无 fault、overcurrent、communication failure 或 tracking watchdog stop；
- 所有原始循环记录和汇总可追溯。

该 PASS 只代表 Mock/FakeBus 软件回归通过，不代表新的真实硬件性能结论，也不自动授权下一次实机运动。

## 6. 证据归档建议

每次十循环测试保存新的、不覆盖既有证据的文件，例如：

```text
agent_docs/j6/validation/j6_home_park_10cycle_mock_<timestamp>.json
```

归档内容应至少包括：

- 软件版本 / git revision；
- 固定基线参数；
- 十个循环的逐循环记录；
- 完整 write audit 分类；
- `hardware_write_count=0`（本程序不得打开真实 UART）；
- 测试命令、pytest 输出和最终 PASS/FAIL。

## 7. 明确禁止事项

本基线程序不得：

- 使用 `--real-hardware`；
- 发送真实电机命令；
- 修改 `safety.py`、watchdog、trajectory、protocol、ROS2、geometry 或 offset；
- 将 speed=20 或 speed>=100 的结果替代 speed=50 基线；
- 自动重试失败循环；
- 覆盖已有报告、日志或冻结文件。
