# 6PUS 并联手术机器人装配后阶段化调试与验收方案 (Post-Assembly Commissioning Plan)

> **文档性质**：本文档为 **6PUS 并联骨科手术机器人机械总装完成后的唯一权威上电、调试与阶段化验收规程**。  
> **适用范围**：完整 6PUS 机械机构（静基座、6 根定长连杆、动平台、6 组一体化推杆电缸）总装完成后，从**完全不上电静态检查**开始，直至**J5-C2-HW 真实机构 FK 验收通过**并完成**双 IMU 最终安装与外参准备**的全过程。  
> **安全红线铁律**：
> 1. **严禁一装好就执行 J4 连续 6DoF 动态轨迹**；
> 2. **所有运动权限逐级开放**，未通过前一阶段门禁前，严禁跨阶段执行高风险操作；
> 3. **首次物理运动必须在单轴极小微动 (Micro-Jog) 下确认方向与无干涉，严禁在方向未确认前直接让完整机构执行全行程 HOME**；
> 4. **`6pus_geometry.yaml` 中的待定参数必须以实测真值冻结**，严禁使用 CAD 理论值或经验猜测值；
> 5. **软件数值正解 PASS 绝不代表真实物理机构 FK 已经 PASS**；
> 6. **严禁使用装配前的同板双 IMU 数据冒充最终机械安装标定**；
> 7. **严禁在 J5-C2-HW 验收通过前提前启动 B4 平动融合、ESKF 或 Dual-IMU/FK 闭环控制**；
> 8. **所有未经验证的几何公差与物理精度指标统一标定为“待实测建立基线 / 建议目标”**，禁止伪装成已确定的硬指标。

---

## 阶段化验收全景与门禁流转图 (Commissioning Roadmap)

```
 [ Phase 0: 机械静态检查 ] ─────────► [ 完全不上电: 结构刚度、连杆等长、球铰间隙、无卡滞 ]
           │ PASS
           ▼
 [ Phase 1: 电气/串口 READ_ONLY ] ──► [ 24V 上电，E-Stop 就绪，0 写入监听，6 电机 0 故障响应 ]
           │ PASS
           ▼
 [ Phase 2: 轴号/反馈/零位确认 ] ───► [ 确认 ID1~ID6 空间拓扑、静态读数与绝对机械底位 ]
           │ PASS
           ▼
 [ Phase 3: 极小单轴微动确认方向 ] ─► [ 【首次真实运动】单轴逐一极小位移微动，确认运动正方向与无机械干涉 ]
           │ PASS
           ▼
 [ Phase 4: PARK/HOME 可达性与就位 ] ─► [ 确认 raw=0 与 raw=1000 机械包络安全，低速试探就位 ]
           │ PASS
           ▼
 [ Phase 5: 六轴协调小幅运动 ] ─────► [ 低速三维平移与俯仰小幅联动，验证多轴协调与无几何干涉 ]
           │ PASS
           ▼
 [ Phase 6: 几何参数实测与冻结 ] ───► [ 物理实测基座/动平台半径、杆长与安装角，更新 6pus_geometry.yaml ]
           │ PASS
           ▼
 [ Phase 7: J5-C2-HW 真实机构 FK 验收 ] ► [ 真实 q_actual → FixedLag → FK，比对外部真值，生成验收报告 ]
           │ PASS
           ▼
 [ Phase 8: 双 IMU 最终安装与 B2 准备 ] ─► [ 双 IMU 最终固定、测量 lever arm，完成 B2 外参准备 ]
           │ PASS
           ▼
 [ 允许进入后续 J5-B4 / ESKF / 多传感器融合规划 ]
```

---

## Phase 0: 机械静态检查 (Mechanical Static Inspection)

### 1. 目标与定位
在**系统完全不上电、未连接 24V 动力源**的前提下，对 6PUS 机构机械总装质量、几何对称性、活动铰链间隙以及机械极限行程进行全面静态核验，杜绝机械装配缺陷引起的卡死或干涉。

### 2. 前置条件
* 6PUS 机构本体机械总装完成（静基座、6 根碳纤维连杆、上端球铰、下端万向铰、动平台已安装固定）；
* **24V 动力电源物理断开**（电源开关处于 OFF 状态，电源线未接入电缸）；
* 主机与 RS485 串口线物理断开。

### 3. 允许执行的操作
* 目视检查所有机械紧固件、碳纤维杆胶接处、球铰弹簧夹紧环；
* 使用游标卡尺/精密高度尺测量 6 根连杆的铰点间距；
* 手动轻推各推杆滑块在行程范围内上下滑动（确认无卡滞、无异常摩擦力）；
* 手动使动平台在设计工作空间内偏转，目视检查动平台球铰在极限角度下是否发生结构干涉；
* 逐项填写《Phase 0 机械静态检查清单》。

### 4. 严禁执行的操作
* **严禁连接 24V 动力电源**；
* **严禁电机通电**；
* **严禁使用工具野蛮撬动或过载扭曲铰链**。

### 5. 使用工具与检查手段
* 0~150mm 精密游标卡尺（精度 0.02mm）；
* 0~300mm 精密高度规 / 平台测量平板；
* 塞尺（测量球铰与万向铰径向/轴向间隙）。

### 6. 需要记录的数据与产出
* 6 根碳纤维连杆的实测长度 $L_{0, 1} \sim L_{0, 6}$（单位：mm）；
* 静基座半径 $c$ 与动平台球铰圆半径 $d$ 实测值；
* 各滑块机械行程上下极限物理测量值；
* 签署完成的纸质/电子版《Phase 0 机械静态核验记录单》。

### 7. PASS 判定标准 (含基线依据)
1. **杆长一致性**：6 根连杆实测长度极差建议控制在 $\max_i L_{0,i} - \min_i L_{0,i} \le 0.05\text{ mm}$（建议设计目标，均值以实测为准，名义 $84.00\text{ mm}$）；
2. **运动平顺性**：6 个滑块全行程手动推拉无任何卡滞点，手感阻力均匀；
3. **铰链自由度**：所有 6 个动平台球铰在设计工作锥角范围内自由旋转无死点、无异常晃旷（游隙待实测建立基线）；
4. **结构刚度**：静基座与推杆导轨安装底座螺栓无松动，整体无结构晃量。

### 8. FAIL 停机与处置条件
* 连杆长度偏差显著超标或滑动存在明显卡死阻滞 $\to$ **立即停止调试，退回机械装配工序重新配平与修配**。

### 9. 下一阶段入口
* 仅当 Phase 0 全部指标达标并签字确认后，方可进入 **Phase 1**。

---

## Phase 1: 电气与串口通信只读审计 (Electrical & UART READ_ONLY Audit)

### 1. 目标与定位
在电缸通电后，**不执行任何电机运动指令**，完成供电电压、静态电流、总线通信完整性与电机故障码的全面安全审计。

### 2. 前置条件
* Phase 0 静态检查已 PASS；
* 24V 直流稳压电源（带过流保护，限流设定为 $2.0\text{ A}$）；
* 硬件急停按钮 (E-Stop) 串联接入动力回路，且测试人员手持急停按钮；
* USB-to-RS485 转换器连接至上位机，确认串口设备号（`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_...`）。

### 3. 允许执行的操作
* 开启 24V 电源；
* 观察各电机指示灯状态；
* 运行 J2-A 只读审计脚本，进行连续 30 秒的静默遥测监听与寄存器读取。

### 4. 严禁执行的操作
* **严禁向串口发送任何写寄存器指令**（0x29 目标位置写入、0x1A 抱闸释放等）；
* **严禁执行任何 J3 HOME/PARK 或 J4 控制脚本**；
* `total_hardware_writes` 计数必须严格保持为 0。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j2_a_real_hardware_readonly.py \
    --duration 30.0 --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10N18X4-if00-port0 --baudrate 921600
  ```

### 6. 需要记录的数据与产出
* 24V 稳压电源静态待机总电流（mA）；
* 6 台电缸只读快照：ID1 ~ ID6 是否全部在线；
* 各轴静态位置反馈 `actual_pos`；
* 各轴驱动板温度 `temperature`（°C）、工作电流 `current_val`（mA）；
* 各轴故障码 `fault_codes` 与通信丢包率；
* 生成的只读审计 JSON 报告文件（`/home/lyapunov/bone_drill/bone_ws/log/j2_a_readonly_*.json`）。

### 7. PASS 判定标准
1. **设备在线率**：ID1 ~ ID6 连续 30 秒 100% 在线，通信超时率 $= 0.00\%$；
2. **故障码清零**：6 台电缸 `error_code` 全部严格为 `0x00`（无过流、堵转、过温或欠压故障）；
3. **静态功耗**：整机 24V 静态待机电流处于合理正常范围（参考基线 $\le 300\text{ mA}$，无短路），单电机温度处于室温正常区间；
4. **写入安全**：审计报告中 `total_hardware_writes == 0`。

### 8. FAIL 停机与处置条件
* 出现任何电机掉线、通信超时、`error_code != 0` 或静态电流异常激增 $\to$ **立即拍下急停按钮断开 24V 电源，排查接线、终端电阻与波特率**。

### 9. 下一阶段入口
* 仅当 Phase 1 审计报告 100% 满足 PASS 条件后，方可进入 **Phase 2**。

---

## Phase 2: 六轴编号、静态反馈与零位映射确认 (Axis ID, Feedback & Zero Mapping Confirmation)

### 1. 目标与定位
在完全静止状态下，严格核对 6 台推杆电缸的物理安装编号（ID1 ~ ID6）与静基座几何安装角（$\lambda=30^\circ, \mu=60^\circ$）的一一对应关系，记录机械下限位静态读数，确认 zero mapping 基础数据。

### 2. 前置条件
* Phase 1 只读审计已 PASS；
* 机构处于低位停放区附近；
* 24V 稳压电源限流设定为 $1.5\text{ A}$，急停按钮就绪。

### 3. 允许执行的操作
* 运行只读查询，读取当前编码器实际值；
* 核对物理电缸 ID 标签与静基座铰点 $A_1 \sim A_6$ 的空间拓扑对应关系。

### 4. 严禁执行的操作
* **严禁发送任何电机运动指令**；
* **严禁执行多轴联动**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 逐轴读取当前编码器实际值
  python3 -c "
  from pus6_motor_driver.six_motor_controller import SixMotorController
  ctl = SixMotorController(port='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10N18X4-if00-port0', baudrate=921600)
  ctl.connect()
  snap = ctl.read_snapshot()
  print('Current raw positions:', snap.actual_positions)
  print('Fault codes:', snap.fault_codes)
  ctl.disconnect()
  "
  ```

### 6. 需要记录的数据与产出
* 6 台电缸 ID 物理空间布置图（自顶向下看顺时针/逆时针排序）；
* 机械最底端时各轴物理整步读数 $q_{\text{bottom}, 1\dots 6}$；
* 确认拓扑与 `6pus_geometry.yaml` 中的铰点 $A_1 \sim A_6$ 对应无误。

### 7. PASS 判定标准
1. **拓扑匹配**：ID1 ~ ID6 的空间位置与 [`6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) 中定义的铰点 $A_1 \sim A_6$ 完全一致；
2. **零位一致**：在机械收拢最底位时，6 轴编码器读数均处于合理零位区间（通常位于 $[0, 50]\text{ step}$ 内，以实测基准为准）。

### 8. FAIL 停机与处置条件
* 发现轴号错位 $\to$ **立即断电，重新排查 RS485 地址烧录与物理贴签**。

### 9. 下一阶段入口
* 仅当 6 轴编号与零位拓扑 100% 确认后，方可进入 **Phase 3**。

---

## Phase 3: 极小单轴微动确认方向与无干涉 (Single-Axis Micro-Jog Direction & Clearance Verification)

> [!IMPORTANT]
> **首次允许真实微动阶段**：本阶段为系统装配后**首次允许电缸产生动态微动**的阶段！  
> **严格安全限制**：必须采用**单轴独立逐一执行**、**极小位移（$\le 1.0\text{ mm} = 40\text{ steps}$）**、**超低速度（$\le 50\text{ step/s} = 1.25\text{ mm/s}$）**，目的是在安全裕量下确认电缸正向运动方向定义（$+Z$ 轴向上抬升）与被动连杆小摆动下的无卡滞性。**严禁在方向未确认前直接执行全行程 HOME！**

### 1. 目标与定位
逐一让 6 根推杆执行极小幅度的单轴微动往复（Micro-Jog），验证电机旋转方向与软件定义方向的一致性、编码器反馈单调性以及微动下的被动球铰顺应性。

### 2. 前置条件
* Phase 2 已 PASS；
* 机构处于安全初始位，动平台上方无任何外部干涉；
* 24V 电源限流设定为 $1.5\text{ A}$，急停按钮手持就绪。

### 3. 允许执行的操作
* 逐轴执行 $\pm 20\text{ steps} \sim \pm 40\text{ steps}$ ($0.5\text{ mm} \sim 1.0\text{ mm}$) 的单轴极小往复点动；
* 限速设定为超低速 $50\text{ step/s} \ (1.25\text{ mm/s})$；
* 每次微动后复位回初始基准。

### 4. 严禁执行的操作
* **严禁单轴位移超过 $40\text{ steps} \ (1.0\text{ mm})$**；
* **严禁多轴联动**；
* **严禁直接下发全行程就位指令**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 运行单轴低速极小位移微动（以 Axis 1 为例，微动 40 步，速度 50 step/s）
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j2_b_real_hardware_single_axis.py \
    --axis 1 --displacement 40 --speed 50 --cycles 1 \
    --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10N18X4-if00-port0 --confirm-motion
  ```

### 6. 需要记录的数据与产出
* 6 轴各自微动时的物理动作方向（是否为绝对向上抬升）；
* 编码器实际读数随指令增加是否严格单调递增；
* 微动期间动态电流与回程残留误差。

### 7. PASS 判定标准
1. **方向单调**：当写入目标整步增加时，推杆必须沿 $+Z$ 向上顶升，且编码器读数 `actual_pos` 严格单调增加；
2. **定位重复性**：微动返回初始位，归零残留误差建议满足 $|\Delta q_i| \le \mathbf{1\text{ step}}$ ($0.025\text{ mm}$)；
3. **顺应无阻**：单轴微动期间，其余 5 根连杆球铰摆动顺畅，无异常摩擦阻滞；
4. **故障码清零**：全程 `fault_codes == 0`。

### 8. FAIL 停机与处置条件
* 发现方向反向、单轴卡死或电流激增 $\to$ **立即急停断电，排查电机相序或机械间隙**。

### 9. 下一阶段入口
* 仅当 6 轴运动方向全部经实测验证正确后，方可进入 **Phase 4**。

---

## Phase 4: STORAGE_PARK / OPERATING_HOME 机械可达性与基础就位 (Mechanical Reachability of PARK/HOME & Baseline Homing)

### 1. 目标与定位
在已确认 6 轴运动方向正确的前提下，首次在超低速受控模式下验证 **`STORAGE_PARK`（停放位，raw $\approx 0$）** 与 **`OPERATING_HOME`（工作零位，raw $\approx 1000$，中位伸出 50.0mm）** 的完整行程机械可达性与连杆姿态，建立系统软件基准原点。

### 2. 前置条件
* Phase 3 单轴微动与方向确认已全部 PASS；
* 动平台上方无多余负载；
* 测试人员一人手持急停，一人近距离目视观察动平台连杆间隙。

### 3. 允许执行的操作
* 使用超低速模式（$\text{speed} = 50\text{ step/s} = 1.25\text{ mm/s}$）执行 `run_j3_startup_homing.py` 进行协调顶升与收拢；
* 在 `OPERATING_HOME` 处进行停顿并进行全方位间隙测量。

### 4. 严禁执行的操作
* **严禁使用标称高速（$200\text{ step/s}$）执行就位**；
* **严禁跳过 HOME 直接执行笛卡尔姿态变换**；
* **严禁在未开启 `maintenance_mode` 授权时强行下发 raw=0 步数**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 1. 以 50 step/s 超低速首次安全顶升至 OPERATING_HOME
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py \
    --action home --speed 50 --real-hardware --confirm-motion --validated-config
  
  # 2. 检查稳定在 HOME 时的状态
  python3 -c "
  from pus6_motor_driver.six_motor_controller import SixMotorController
  ctl = SixMotorController(port='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10N18X4-if00-port0', baudrate=921600)
  ctl.connect()
  snap = ctl.read_snapshot()
  print('Home positions (should be ~1000):', snap.actual_positions)
  print('Currents (mA):', snap.currents)
  ctl.disconnect()
  "
  ```

### 6. 需要记录的数据与产出
* 6 轴顶升到达 HOME 时的稳态整步读数 $q_{\text{home}, 1\dots 6}$；
* 顶升全程中 6 轴最大动态工作电流（mA）；
* `OPERATING_HOME` 状态下动平台中心距静基座顶面的实际垂直高度 $z_{\text{actual}}$（mm）；
* 连杆与导轨立柱之间的最小物理距离（mm）。

### 7. PASS 判定标准 (含基线依据)
1. **稳态精度**：6 轴平稳到达工作零位，稳态步数位于 $[998, 1002]\text{ step}$ 范围内；
2. **无干涉死区**：在 `OPERATING_HOME` 与 `STORAGE_PARK` 状态下，相邻碳纤维连杆之间最小间隙充足（建议目标 $\ge 5.0\text{ mm}$），球铰未达死挡极限；
3. **负载电流正常**：匀速顶升过程中电流平稳无突变（建议动态参考上限 $\le 400\text{ mA}$，稳态维持 $\le 250\text{ mA}$）；
4. **水平度合格**：动平台在 HOME 状态下经水平仪测量，倾角处于合理平整区间（建议目标 $\le 0.5^\circ$）。

### 8. FAIL 停机与处置条件
* 顶升过程中听到机械异响、电流异常突增或连杆干涉 $\to$ **立即拍下急停，查明几何干涉点**。

### 9. 下一阶段入口
* 仅当 HOME 与 PARK 状态机械可达性 100% 确认后，方可进入 **Phase 5**。

---

## Phase 5: 六轴协调小幅笛卡尔运动测试 (Six-Axis Coordinated Small Cartesian Motion)

### 1. 目标与定位
验证在多轴协调联动下，动平台沿基础笛卡尔自由度（Heave $Z$, Surge $X$, Sway $Y$, Pitch, Roll）进行小幅运动的机构运动学正确性、控制平稳性与动态无碰撞性。

### 2. 前置条件
* Phase 4 HOME 就位已 PASS，平台处于 `OPERATING_HOME`；
* 24V 电源限流设定为 $2.0\text{ A}$。

### 3. 允许执行的操作
* 执行低速（$\text{peak\_speed} = 100\text{ step/s} = 2.5\text{ mm/s}$）的 J3 点到点单自由度测试：
  * **垂直升降 Heave**：$Z \pm 2.0\text{ mm}$；
  * **水平平移 Surge / Sway**：$X \pm 2.0\text{ mm}, Y \pm 2.0\text{ mm}$；
  * **空间俯仰/横滚 Pitch / Roll**：$\pm 0.020\text{ rad} \ (\pm 1.15^\circ)$；
* 每次运动后平稳返回 `OPERATING_HOME`。

### 4. 严禁执行的操作
* **严禁执行满幅空间轨迹**（如平移 $>5.0\text{ mm}$ 或转角 $>0.05\text{ rad}$）；
* **严禁执行 J4-F1E 等全自由度复合高速动态轨迹**；
* **严禁在未做几何实测前提高运动速度**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 1. 垂直小幅升降 (+2mm)
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j3_cartesian_motion.py \
    --test heave_pos --peak-speed 100 --real-hardware --confirm-motion --validated-config
  
  # 2. 俯仰小幅摆动 (+0.02rad)
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j3_cartesian_motion.py \
    --test pitch_pos --peak-speed 100 --real-hardware --confirm-motion --validated-config
  ```

### 6. 需要记录的数据与产出
* 6 轴指令与实际反馈整步的时域波形；
* 6 轴最大动态电流与同步协调性；
* 动平台实际运动方向与理论方向的目测/水平仪一致性；
* J3 Cartesian 测试日志。

### 7. PASS 判定标准 (含基线依据)
1. **多轴协调性**：6 轴动作严格同步，关节空间跟随建议满足 $\text{MAE} \le \mathbf{2\text{ steps}}$ ($0.05\text{ mm}$)；
2. **返回精度**：各测试项结束后返回 HOME，6 轴各轴归位误差建议满足 $\le 1\text{ step}$；
3. **功耗正常**：6 轴联动动态峰值总电流处于稳态合理范围（参考建议上限 $\le 1.0\text{ A}$）；
4. **安全前检通过**：驱动层前检拦截次数为 0，无任何安全门禁触发。

### 8. FAIL 停机与处置条件
* 联动时平台姿态歪斜、出现剧烈振动或前检报错 $\to$ **立即急停，检查逆解坐标系与参数**。

### 9. 下一阶段入口
* 仅当基础自由度小幅联动稳定通过后，方可进入 **Phase 6**。

---

## Phase 6: 最终机械几何参数实测与配置冻结 (Final Geometry Measurement & YAML Freezing)

### 1. 目标与定位
在完成基础运动验证后，使用高精度测量仪器对实际物理装配后的全部关键几何参数进行实测标定，将权威真值更新写入 [`6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)，并执行全工作空间回归测试予以**正式冻结 (FROZEN)**。

### 2. 前置条件
* Phase 5 多轴协调小幅运动已 PASS；
* 机构断电并固定在稳固水平光学平台上；
* 准备好精密测量工具（三坐标测量机 CMM / 便携式测量臂 / 高精度数显卡尺与激光测距仪）。

### 3. 允许执行的操作
* 实测基座铰点分布圆半径 $c$、动平台铰点圆半径 $d$；
* 实测各铰点安装对角偏角 $\lambda$（基座铰点配对半角）与 $\mu$（动平台铰点配对半角）；
* 实测 6 根连杆有效杆长 $L_{0, i}$；
* 实测动平台在 `OPERATING_HOME` 处的标称高度 $z_{\text{nom}}$；
* 修改并保存 `6pus_geometry.yaml` 中的对应参数字段；
* 运行工作空间全量自动化回归测试 `pytest bone_ws/src -v`。

### 4. 严禁执行的操作
* **严禁保留 YAML 中未经实测的 `pending_hardware_confirmation: true` 占位状态**；
* **严禁凭借 CAD 理论图纸数值代替实测值**；
* **严禁在 pytest 回归未全量通过前进入下一阶段**。

### 5. 配置文件与修改点
* **权威配置文件**：[`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)
* **核对与修改字段**：
  ```yaml
  status: "FROZEN_POST_ASSEMBLY"
  geometry:
    base_radius_m: <实测值，如 0.04000>
    platform_radius_m: <实测值，如 0.02750>
    strut_length_m: <实测均值，如 0.08400>
    lambda_rad: <实测值，如 0.5235987756>
    mu_rad: <实测值，如 1.0471975512>
    nominal_height_m: <实测标称高度>
    confirmation:
      measured_date: "2026-08-XX"
      confirmed_by: "Assembly & Calibration Team"
      calibrated_with_hardware: true
  ```

### 6. 需要记录的数据与产出
* 几何测量原始记录表（包含多次重复测量与不确定度分析）；
* 更新后的 `6pus_geometry.yaml` 配置文件；
* 全工作空间 `pytest /home/lyapunov/bone_drill/bone_ws/src -v` 回归全量 PASS 日志。

### 7. PASS 判定标准
1. **参数自洽**：所有几何尺寸具备可重复测量数据支持（建议测量重复性 $\le 0.05\text{ mm}$）；
2. **回归通过**：执行 `pytest /home/lyapunov/bone_drill/bone_ws/src -v`，当前收集范围内的全工作空间回归集 100% 全部 PASS，零 FAIL、零 ERROR；
3. **逆解/正解一致性**：新参数加载后，纯数值 IK↔FK 随机测试 1000 样本收敛率仍为 $100.00\%$。

### 8. FAIL 停机与处置条件
* pytest 回归出现任何断言失败或数值不收敛 $\to$ **立即检查 YAML 格式与测量数值，排查是否存在物理不可达参数**。

### 9. 下一阶段入口
* 仅当 `6pus_geometry.yaml` 完成实测更新并达成全量回归 PASS 后，方可正式进入 **Phase 7 (J5-C2-HW 验收)**。

---

## Phase 7: J5-C2-HW 真实机构正运动学验收 (Real Hardware FK Acceptance)

> [!IMPORTANT]
> **J5-C2-HW 正式里程碑判定点**：本阶段为 **J5-C2-HW 阶段的正式启动与验收执行点**。将物理硬件真实反馈的 `q_actual` 接入 `TimedJointFeedback → FixedLag → q_sync → Numerical FK` 观测链路，并借助外部高精度基准测量仪器验证真实末端位姿 $\mathbf{T}_{BP}$ 的绝对测量精度。

### 1. 目标与定位
在真实 6PUS 机构静态及低速小幅运动过程中，验证高精度时间同步对齐与解析雅可比 LM 数值正解在真实物理机上的收敛稳定性、残差表现及与外部物理真实位姿的符合度，达成 J5-C2-HW 阶段正式结项。

### 2. 前置条件
* Phase 6 几何参数已实测冻结，全量回归 pytest 全部 PASS；
* 动平台末端安装外部高精度真值测量设备（如光学跟踪仪反光靶球或多点千分表）；
* 上位机启动独立的 `Pus6FkNode`（必须独占 `SingleThreadedExecutor`）。

### 3. 允许执行的操作
* 运行完整系统节点图（`pus6_motor_driver_node` + `pus6_fk_node`）；
* 采集实际运行中的 `/pus6/joint_feedback/timed` 遥测流并保存录包；
* 运行离线验收脚本 `run_j5_c2_hw_acceptance.py` 生成最终验收报告；
* 记录外部测量实测动平台位姿真值 $\mathbf{T}_{BP}^{\text{GT}}$ 与 FK 估计位姿 $\mathbf{T}_{BP}^{\text{FK}}$ 进行对比。

### 4. 严禁执行的操作
* **严禁将 FK 输出接入任何控制回路（严禁闭环反馈到 J4 controller）**；
* **严禁在缺少外部物理测量手段时单凭数值残差宣称物理机构精度**；
* **严禁修改 FK 数值解算核心算法与测地线门禁阈值**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 1. 启动硬件驱动与单线程 FK 节点系统
  ros2 launch pus6_bringup pus6_system.launch.py
  
  # 2. 运行 J5-C2-HW 真实硬件数据验收脚本
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_hw_acceptance.py \
    --duration-s 30 --feedback-hz 25 --query-rate-hz 200 \
    --input-jsonl /home/lyapunov/bone_drill/bone_ws/log/real_hw_timed_feedback.jsonl \
    --output-prefix /home/lyapunov/bone_drill/bone_ws/log/j5_c2_hw_final_acceptance
  ```
* **核心话题监听**：
  * `/pus6/joint_feedback/timed` (`pus6_msgs/msg/TimedJointFeedback`)
  * `/pus6/fk/state` (`pus6_msgs/msg/FkStateStamped`)
  * `/pus6/fk/pose` (`geometry_msgs/msg/PoseStamped`)

### 6. 需要记录的数据与产出
* 验收工具生成的 JSON 与 CSV 汇总报告；
* 200Hz 固定延迟双向夹逼对齐成功率 (`SYNCED` 比率)；
* LM 迭代收敛率、平均迭代次数与定长几何残差范数；
* $\text{SO}(3)$ 测地线跳变门禁触发次数（必须为 0）；
* FK 解算位姿与外部测量真值的平移残差 $\Delta \mathbf{p}$ (mm) 与旋转残差 $\Delta \mathbf{R}$ (deg)。

### 7. PASS 判定标准 (含基线依据)
1. **时间同步性能**：Fixed-Lag 双向夹逼对齐成功率建议满足 $\ge \mathbf{99.50\%}$，无持续或异常的 `WAIT_FUTURE_SAMPLE` 堆积，严格 0 外推；
2. **正解数值收敛**：在全运动周期内，LM 数值正解收敛率严格为 $\mathbf{100.00\%}$，几何定长残差均值建议满足 $\le 1.0\times 10^{-4}\text{ m}$；
3. **物理分支连续**：$\text{SO}(3)$ 测地线角度跳变门禁拦截率 $= \mathbf{0.00\%}$，工作分支稳定保持；
4. **外部物理精度**：FK 计算位姿与外部基准测量对比，误差建议达到工程目标（平移参考目标 $\le 0.50\text{ mm}$，姿态参考目标 $\le 0.50^\circ$，最终以骨科手术精度需求与量具标定为准）；
5. **验收报告签署**：`j5_c2_hw_final_acceptance.json` 自动生成且字段中 `hardware_acceptance_passed == true`。

### 8. FAIL 停机与处置条件
* 正解出现不收敛、几何残差发散或与外部真值误差超标 $\to$ **退回 Phase 6 重新检查并标定机械几何参数与零位偏置**。

### 9. 下一阶段入口
* 仅当 J5-C2-HW 验收报告正式通过并签署后，方可进入 **Phase 8**。

---

## Phase 8: 双 IMU 最终安装、B2 外参标定与 Lever Arm 测量准备 (Dual-IMU Final Installation & B2/Lever-Arm Preparation)

### 1. 目标与定位
在机构总装与物理 FK 精度验收完成的前提下，将两只 H30 IMU 分别刚性安装至基座与动平台的最终物理位置，完成传感器空间外参矩阵 $\mathbf{R}_{SB}, \mathbf{R}_{SP}$ 标定与动平台质心到 IMU 敏感中心的 Lever Arm 杠杆臂矢量 $\mathbf{r}_{\text{arm}}$ 测量，为后续多传感器融合奠定基础。

### 2. 前置条件
* Phase 7 (J5-C2-HW) 验收已正式 PASS 并签署；
* 两只 H30 传感器已从最初的同板测试治具上拆下；
* 基座 IMU (Base IMU) 与动平台 IMU (Platform IMU) 专用机械固定支架加工完成。

### 3. 允许执行的操作
* 将 Base IMU 刚性锁紧在静基座机架上，Platform IMU 刚性锁紧在动平台顶部；
* 使用测绘工具实测 Platform IMU 相对动平台几何中心的偏移矢量 $\mathbf{r}_{\text{arm}} = [r_x, r_y, r_z]^\top$；
* 启动 `h30_imu` 节点与 `pus6_fk_node`，在静态及小幅慢速倾斜下采集双 IMU 与 FK 的同步数据；
* 运行 J5-B2 标定算法，解算并核验相对旋转外参矩阵。

### 4. 严禁执行的操作
* **严禁使用装配前的“同板共刚体”标定数据冒充当前最终机械外参**；
* **严禁在此阶段启动 B4 平动模型融合算法、ESKF 滤波器或闭环姿态融合控制**；
* **严禁在未做 Lever Arm 补偿的情况下直接将 IMU 角速度用于动态位姿更新**。

### 5. 使用脚本与 ROS 2 话题
* **执行命令**：
  ```bash
  # 启动双 IMU 采集与同步解算流水线
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j5_b1_b3_pipeline.py \
    --duration 60.0 --output-dir /home/lyapunov/bone_drill/bone_ws/log/imu_b2_calibration
  ```
* **核心话题监听**：
  * `/imu/base/data` (`sensor_msgs/msg/Imu`)
  * `/imu/platform/data` (`sensor_msgs/msg/Imu`)
  * `/pus6/fk/state` (`pus6_msgs/msg/FkStateStamped`)

### 6. 需要记录的数据与产出
* Lever Arm 物理测量记录（包含 $r_x, r_y, r_z$ 坐标及不确定度说明）；
* 最终安装状态下的双 IMU 零偏与频偏标定值（PPM）；
* 动平台倾斜运动下，双 IMU 差分姿态 $\mathbf{R}_{\text{rel}}^{\text{IMU}}$ 与正解姿态 $\mathbf{R}_{\text{rel}}^{\text{FK}}$ 的连续残差曲线；
* 更新后的 `6pus_geometry.yaml` 中 `imu` 节点配置。

### 7. PASS 判定标准 (含基线依据)
1. **外参自洽**：双 IMU 相对姿态估计与 FK 正解姿态在慢速倾斜测试中趋势一致（静态残差建议目标 $\le 0.50^\circ$，动态 RMS 待实测建立基线）；
2. **Lever Arm 记录**：$\mathbf{r}_{\text{arm}}$ 物理测量完成并写入配置文件；
3. **时钟同步稳健**：双 IMU 软件时间同步残差保持在正常微秒/毫秒级稳定区间。

### 8. FAIL 停机与处置条件
* 双 IMU 相对姿态与 FK 差异发散严重或时钟丢帧严重 $\to$ **检查传感器刚性固定螺栓、串口接线与外参标定算法**。

### 9. 验收结项与下一阶段衔接
* **Phase 8 验收完成标志着 6PUS 手术机器人装配后全部基础调试、硬件几何冻结与底层正解/IMU 观测链的全面贯通**！
* 此时方正式允许规划并启动后续高级算法研发：**J5-B4 平动融合模型、ESKF 滤波以及 Dual-IMU/FK 多源融合闭环控制**。
