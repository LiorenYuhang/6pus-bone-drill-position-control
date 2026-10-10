# J11 Demo B USB/UART Fault Audit

**状态**：Canonical engineering note（工程审计归档）

**事件级别**：runtime transport robustness observation（**非** controller defect / P/PD instability / D-term failure / operator amplitude 控制失败）

**结论性质**：只读根因审计；本轮未运行任何真实硬件。

---

## 1. Executive Conclusion

J11 Demo B 首次 `motion_write_failure` 由 **motor CH340 USB-UART 的瞬时 disconnect / re-enumeration** 直接导致，**不是** P/PD 控制器不稳定，也**不是**操作者运动幅度导致控制失败。

因果链（全部有日志证据）：

1. `11:47:55.0516 CST`：motor CH340 USB 设备（`usb 3-2`，VID:PID = `1a86:7523`）发生 port-level disconnect（`USB disconnect, device number 3`）。
2. 内核记录 `failed to send control message: -19`（= `ENODEV`，设备在传输中途消失）。
3. ch341 驱动解绑 `ttyUSB0`。
4. 驱动 `serial.write()` 失败 → `j7_six_axis_servo_transport.py` 在 `11:47:55.1238`（disconnect 后约 72 ms）锁存 `motion_write_failure`。
5. runner 经 `/rosout` 检测到 fatal latch，提前结束并自动 HOME 恢复成功。
6. 设备约 270 ms 后重新枚举（device number 3 → 11，`ttyUSB0` → `ttyUSB1`），此后保持稳定。

相同 controller / gains / wrapper 的第二次 Demo B 正常完成，未再出现该 fault。

---

## 2. Exact Fault Timeline

时钟域对齐：ROS logger 与 kernel 均使用 wall-clock（CST, Asia/Shanghai +0800），fault 窗口内可直接对齐；telemetry CSV / stall 文件为 monotonic（仅用于相对时长）。

| 时间 (CST, 2026-09-20) | 来源 | 事件 |
|---|---|---|
| 11:47:22.89 | ROS launch | Demo B #1 启动（`motor_driver_node` pid 192458） |
| 11:47:55.051602 | kernel | `usb 3-2: USB disconnect, device number 3` |
| 11:47:55.057985 | kernel | `usb 3-2: failed to send control message: -19` |
| 11:47:55.058102 | kernel | `ch341-uart ttyUSB0: converter now disconnected` |
| 11:47:55.123826 | ROS driver | `[FATAL] J7 driver STOP latched: motion_write_failure` |
| 11:47:55.320637 | kernel | `usb 3-2: new full-speed USB device number 11`（重枚举开始） |
| 11:47:55.445600 | kernel | `New USB device found, idVendor=1a86, idProduct=7523` |
| 11:47:55.447822 | kernel | `ch341-uart converter now attached to ttyUSB1` |
| 11:47:55.567263 | ROS driver | `Active node shutting down: executing J7 0x1A stop_all` |

关键间隔：disconnect → fatal latch = **72.2 ms**；disconnect → re-attach = **396.2 ms**。fatal latch 在设备重新出现之前就已触发，驱动没有机会在带内恢复。

---

## 3. Kernel USB Evidence

fault 前后 ±3 min 的未过滤 kernel log 仅包含以下 USB 行（无 xhci/hub reset、无 over-current、无 autosuspend、无 power 错误）：

```text
11:47:55.051602  usb 3-2: USB disconnect, device number 3
11:47:55.057985  usb 3-2: failed to send control message: -19
11:47:55.058102  ch341-uart ttyUSB0: ... disconnected from ttyUSB0
11:47:55.058156  ch341 3-2:1.0: device disconnected
11:47:55.320637  usb 3-2: new full-speed USB device number 11 using xhci_hcd
11:47:55.445600  usb 3-2: New USB device found, idVendor=1a86, idProduct=7523
11:47:55.445905  usb 3-2: New USB device strings: Mfr=0, Product=2, SerialNumber=0
11:47:55.446039  usb 3-2: Product: USB Serial
11:47:55.447822  usb 3-2: ch341-uart converter now attached to ttyUSB1
```

- **分类 A（仅 motor CH340 掉线）= 成立**：只有 `usb 3-2` 掉线；两个 H30 IMU（`ttyACM0/1`）及 mouse/keyboard/BT/RealSense 全部保持枚举。
- **分类 B（整 bus/hub reset）= 排除**：无 xhci reset、无 hub reset。
- **分类 C（多设备同时掉线）= 排除**。
- **isolated**：2026-09-20 全天仅此一次 USB disconnect；设备 boot 后稳定约 2.5 h，故障后稳定约 1 h（当前 `ttyUSB1`）。

`error -19` = `ENODEV`（No such device）：内核 ch341 驱动的 in-flight control transfer 命中了已从 bus 消失的设备。区别于 `-71`（EPROTO 协议错误）、`-110`（ETIMEDOUT）、power loss、userspace close（userspace close 不会产生 `USB disconnect` 与新 device number）。新 device number（3 → 11）证明是完整 disconnect + reconnect，而非简单 reset。

---

## 4. motion_write_failure Source Code Path

完整路径（`bone_ws/src/pus6_motor_driver/pus6_motor_driver/`）：

1. `motor_driver_node.py` 专属 50 Hz worker 线程 `_servo_dedicated_worker_loop()` → `_execute_servo_cycle()` → `self.j7_writer.send_cycle(...)`。
2. `j7_six_axis_servo_transport.py` `send_cycle()`：6 轴 `0x29` 写循环内 `written = self.bus.write(frame)`；循环外层 `except Exception` 执行 `self.stop("motion_write_failure")`（`:351`）并 re-raise。
3. `uart_bus.py:200`：`written = self.serial.write(frame)`（pyserial → os.write 到 tty fd）；异常包装为 `TransportError(f"UART write failed: {exc}")`（`:204-205`）。
4. `motor_driver_node.py:1098` 的外层 `except` 看到 `j7_writer.stopped == True` → `_latch_j7_fault(self._j7_stop_reason())` → `"motion_write_failure"`。

- 锁存点在 **serial.write 路径**（非 drain / flush / readback / ack / timeout 专用路径）。
- `motion_write_failure` 是 **aggregate latch reason**（`except Exception` 包裹整个 6 轴写循环，不区分 motor ID / 错误子类型）。
- **底层 exception string 未保留**：FATAL 行只记录 `motion_write_failure`，内层 `TransportError("UART write failed: <OSError…>")` 文本被丢弃。当前日志无法恢复具体 exception string。

---

## 5. First vs Repeat Comparison

| 方面 | Demo B #1 (114722) | Demo B #2 (115754) | 是否一致 |
|---|---|---|---|
| Launcher | `run_j11_demo_feedforward_pd.sh` | 相同 | 是 |
| Gains | Kp=[0.05,0.05,0.025], Kd=[0,0.0015,0.0015], fc=5Hz | 相同 | 是 |
| Driver / transport | `J7SixAxisServoTransport`, mode=1 speed=50 | 相同 | 是 |
| Port | by-id `usb-1a86_USB_Serial-if00-port0` | 相同 | 是 |
| Final state | **FAULT**（motion_write_failure） | **ACTIVE**（clean SIGINT） | 否 |
| Hardware writes / cycles | 1126 / 1184 | 2375 / 2429 | #2 更长 |
| Peak current | 410 mA | 487 mA | #2 更高 |
| Platform yaw peak | 8.47° | 27.26° | #2 更高 |
| 运行期间 USB 事件 | 1 次 disconnect | 0 次 | 否 |

第二次 run 需求更高（电流 487 mA、platform yaw 27°）却无 fault，是区分 **controller failure**（会复现）与 **transient transport interruption**（随机、不复现）的决定性证据。

---

## 6. Root-Cause Evidence Ranking

**STRONG EVIDENCE**

1. motor CH340（`usb 3-2`, `1a86:7523`）发生 port-level disconnect + re-enumeration（device 3→11，ttyUSB0→ttyUSB1）。
2. disconnect 先于并导致 serial write 失败：`-19`(ENODEV) → `motion_write_failure`（72 ms 后）。
3. 单端口事件：仅 motor 掉线，双 IMU 与其余 USB 全部正常（排除 bus/hub/power 全局故障）。
4. isolated：全天唯一一次 USB disconnect。
5. 不可复现：相同 controller、更高需求的第二次 run 无 fault。

**POSSIBLE / NOT PROVEN**（日志无法判定，需物理检查）

- motor USB cable/connector 松动。
- Base 扰动时线缆被拉扯（motor 直连 NUC root port `3-2`，是单点，不同于 IMU 走 Genesys hub）。
- CH340 adapter 自复位 / 欠压。
- motor 线 EMI / power 波动。

**UNSUPPORTED / RULED OUT**

- P/PD controller instability；D-term failure；operator 幅度导致控制失败；整 bus/hub reset；over-current / power 全局故障；串口 process conflict。

---

## 7. Why HOME Recovery Succeeded

生产栈**不使用** `/dev/ttyUSB0`，而使用稳定 by-id 路径：

- `motor_driver_node.py:142` / `safety.py:61` / HOME runner `run_j7_servo_home_park.py:177` 默认 `port = /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。

设备重枚举为 `ttyUSB1` 后，udev 自动把 by-id 符号链接改指向 `ttyUSB1`（已实机验证 `usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB1`）。HOME runner 打开 by-id 路径时透明解析到新节点，重新建立通信，因此 HOME recovery 成功（0 fault / 0 timeout，final error ≤ 2 steps，0x1A stop PASS）。

---

## 8. Stable Serial Naming

- production **未硬编码 `/dev/ttyUSB0`**，已使用 by-id 路径 —— 这是对 renumbering 的正确缓解。
- 但 motor CH340 **无唯一 serial**（`SerialNumber=0`，Product "USB Serial"）。by-id 名 `usb-1a86_USB_Serial-if00-port0` 仅在“恰好只有一个该 generic 身份设备”时才稳定。若未来存在第二个相同 CH340，generic by-id 可能产生识别风险。
- 两个 IMU 有唯一 serial（`5B32028000` / `5B0B029000`），可稳定区分。
- 观测到的 renumbering（ttyUSB0 → ttyUSB1）已被 by-id 路径吸收。

**这是 future robustness note，不是当前 blocker。不改 production path。**

---

## 9. Impact on J11 Freeze

**不阻塞 J11 controller freeze。** 三个层面分离：

- **A. Controller performance evidence = 保持有效**（fault 是 transport，非 controller）。
- **B. Demo runtime reliability = PASS ON REPEAT**（第二次正常完成）。
- **C. USB hardware robustness = OPEN ENGINEERING OBSERVATION**（单次瞬时 disconnect，作为硬件鲁棒性项跟踪，非 controller defect）。

---

## 10. Physical Inspection Recommendation

仅记录，不修改代码，不加自动 reconnect / retry / safety 变更：

1. 检查 motor CH340 USB cable 与 NUC `usb 3-2` connector。
2. 检查线缆应力释放（fault 发生在 Base 手动扰动期间）。
3. 如发现松动：重新插紧或更换 cable。
4. EMI / power / adapter reset 保持 possible but unproven。

因是 isolated transient，**不**为此增加复杂恢复机制，除非未来可重复出现。
