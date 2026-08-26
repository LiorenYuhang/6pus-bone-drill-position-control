# 6PUS 并联机器人日常实机操作指南 (Servo Operation Guide)

> **权威性声明**：本文档为 **6PUS 并联骨科手术机器人平台当前唯一日常实机操作指南**。  
> 基于已通过实机验证并冻结的 **J7 六轴 50Hz Servo 伺服运动控制基线**（`v1.0-j7-servo-hardware-frozen`）编写，面向实验人员日常开机、HOME 初始化、实验后受控 PARK 收拢与应急安全停机。

---

## 1. 新终端标准初始化

所有实机操作前，必须在新终端中执行环境初始化：

```bash
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
cd /home/lyapunov/bone_drill
```

---

## 2. 标准实验全流程概览

```text
POWER ON (开启 24V 动力电源)
   ↓
J7 SERVO HOME (执行一键开机就位)
   ↓
OPERATING_HOME (处于工作零位 [1000]*6)
   ↓
ROS 2 / Cartesian / Anti-Vibration Operation (开展控制与防抖实验)
   ↓
Experiment Finish (实验完成)
   ↓
J7 SERVO PARK (执行一键受控收拢)
   ↓
STORAGE_PARK (处于停放收拢位 [0]*6)
   ↓
0x1A STOP (抱闸停机确认)
   ↓
POWER OFF (关闭 24V 动力电源)
```

---

## 3. 开机就位：HOME 初始化

在 24V 动力电源开启后，执行生产级 J7 伺服 HOME 就位命令：

```bash
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py \
  --action home \
  --speed 100 \
  --rate 50 \
  --real-hardware \
  --confirm-motion
```

* **执行机理**：
  ```text
  0x30 预检读取六轴当前实际位置 (q_current)
  → 写 0x25=1 配置 Servo 模式并验证 ACK
  → J7 50Hz 纯写连续目标流 (20ms 周期，五次多项式平滑 S 曲线)
  → 到达 OPERATING_HOME ([1000]*6, 行程中位伸出 50.0mm, z_nom ≈ 132.61mm)
  → 驻留保持 2.0s 并执行最终全状态校验
  → 0x1A 干净停机，退出程序
  ```

---

## 4. 正常实验

* 当 HOME 命令成功退出且终端报告 `Result: PASSED` 后，机构已处于绝对工作基准原点 `OPERATING_HOME`；
* 此时可按实验需求启动后续 ROS 2 上位规划、逆解控制节点或防抖感知融合管线。

---

## 5. 实验结束：PARK 安全收拢

实验结束或准备关机维护前，必须执行生产级 J7 伺服 PARK 收拢命令：

```bash
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py \
  --action park \
  --speed 100 \
  --rate 50 \
  --real-hardware \
  --confirm-motion
```

* **执行机理**：
  ```text
  0x30 预检读取六轴当前实际位置 (q_current)
  → 写 0x25=1 配置 Servo 模式并验证 ACK
  → J7 50Hz 纯写连续目标流 (20ms 周期，五次多项式平滑 S 曲线)
  → 到达 STORAGE_PARK ([0]*6, 6 根推杆完全缩回至 25.0mm 机械极限)
  → 驻留保持 2.0s 并执行最终全状态校验
  → 0x1A 干净停机，退出程序
  ```
* **关机断电**：终端提示 `Result: PASSED` 且 6 轴均回位到 `[0]*6` 后，可安全关闭 24V 动力电源。

---

## 6. 异常处置与应急停止 (Emergency Stop)

* **键盘中断 (Ctrl + C)**：在终端运行任意阶段按下 `Ctrl + C`，Runner 内置 `try ... finally:` 机制将自动向全六轴下发最佳努力（Best-Effort）的 `0x1A` 抱闸停机指令；
* **自动安全熔断**：运行期检测到任何轴 `fault != 0`、通信超时、跟踪偏差 $>35\,\text{step}$ 或电流 $>1500\,\text{mA}$ 时，程序立即触发 `stop_all()` 停机；
* **物理急停**：若实机观察到任何机械干涉、异常异响或机构失控，**优先按下 24V 电源物理总急停开关或切断动力电源**。

---

## 7. 历史/回退说明 (Legacy Fallback Note)

> [!NOTE]
> 原 J6 定位模式（`mode=0, 10Hz, speed=50`）HOME/PARK 脚本（`run_j3_startup_homing.py`）及原长篇操作指南已归档至 [`agent_docs/archive/6PUS_REAL_HARDWARE_OPERATION_GUIDE_LEGACY.md`](file:///home/lyapunov/bone_drill/agent_docs/archive/6PUS_REAL_HARDWARE_OPERATION_GUIDE_LEGACY.md)，仅保留用于底层装配调试与历史故障排查，**不再作为正常日常运动方案**。
