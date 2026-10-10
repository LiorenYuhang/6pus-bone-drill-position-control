# LADRC 控制器参数与状态变量技术说明

## 1. 方法体系界定

自抗扰控制（ADRC, Active Disturbance Rejection Control）并非某一个单一固定的算法公式，而是一类以“扩张状态观测器（Extended State Observer, ESO）估计并动态补偿总扰动”为核心思想的控制方法族。

在骨科手术机器人 6PUS 并联机构姿态稳定系统中，本项目采用的是：
**面向 6PUS 动平台姿态残差控制（Attitude Residual Control）的一阶离散化线性自抗扰控制实现（First-Order Linear ADRC / LADRC）**。

其连续时间单轴标称被控对象与符号约定（严格与当前生产代码 `j13_ladrc.py` 一致）为：
$$\dot{e} = f - b_0 u$$
其中：
- $e$：世界坐标系下的姿态残差跟踪误差分量（rad 或 deg，误差测量反馈 $y = e$）；
- $u$：姿态残差补偿角控制量指令（rad 或 deg）；
- $f$：等效集总扰动项（Equivalent Lumped Disturbance，单位 rad/s 或 deg/s）；
- $b_0 > 0$：高频名义输入增益参数（$\text{s}^{-1}$）。

---

## 2. 状态观测器与控制律公式约定（与 `j13_ladrc.py` 严格一致）

### 2.1 连续二阶线性扩张状态观测器（Continuous Second-Order LESO）
$$\dot{z}_1 = z_2 - b_0 u_{\text{actual}} + \beta_1 (y - z_1)$$
$$\dot{z}_2 = \beta_2 (y - z_1)$$
其中：
- 测量反馈 $y = e$（姿态残差角误差）；
- 状态估计 $z_1 \approx e$（跟踪误差估计，rad 或 deg）；
- 扩张状态 $z_2 \approx f$（等效集总扰动项，rad/s 或 deg/s）；
- 观测器增益：$\beta_1 = 2 \omega_o$，$\beta_2 = \omega_o^2$；
- $u_{\text{actual}}$：上一周期实际作用于被控对象的物理控制量（经过限幅截断后的值）。

### 2.2 精确离散化实现（Exact Discrete LESO, ZOH Discretization）
代码通过零阶保持器（ZOH）精确离散转移矩阵更新状态：
$$\mathbf{z}_k = \boldsymbol{\Phi} \mathbf{z}_{k-1} + \boldsymbol{\Gamma}_u u_{\text{actual}, k-1} + \boldsymbol{\Gamma}_y y_k$$
其中 $T_s = \Delta t$，$w = \omega_o T_s$，$E = e^{-w}$：
$$\boldsymbol{\Phi} = E \begin{bmatrix} 1 - w & T_s \\ -\omega_o^2 T_s & 1 + w \end{bmatrix},\quad \boldsymbol{\Gamma}_u = \begin{bmatrix} -b_0 T_s E \\ b_0 [1 - E(1 + w)] \end{bmatrix},\quad \boldsymbol{\Gamma}_y = \begin{bmatrix} 1 - E(1 - w) \\ E \omega_o^2 T_s \end{bmatrix}$$

### 2.3 控制律与输出限幅（LADRC Control Law）
$$u_{\text{raw}} = \frac{\omega_c z_1 + z_2}{b_0}$$
$$u_{\text{actual}} = \text{clip}(u_{\text{raw}}, -u_{\text{limit}}, +u_{\text{limit}})$$
闭环代入 nominal plant：
$$\dot{e} = f - b_0 u_{\text{raw}} = f - (\omega_c z_1 + z_2) = -\omega_c z_1 + (f - z_2) \approx -\omega_c e$$
使姿态跟踪误差以带宽 $\omega_c$ 指数衰减收敛。

---

## 3. 核心参数与状态变量工程含义

### 3.1 $b_0$（Nominal Input-Gain Estimate，名义输入增益估计）
- **工程含义**：名义上表征“控制器认为施加单位残差补偿角指令 $u$，能对动平台姿态残差变化率带来多大的驱动增益”。
- **关键技术说明**：
  - $b_0$ 是控制算法设计人员根据机构几何传递比与闭环响应需求设定的**标称尺度参数（Nominal Tuning Parameter）**；
  - **$b_0$ 绝非真实 6PUS 并联机构的精确物理参数**；
  - 在控制律 $u_{\text{raw}} = (\omega_c z_1 + z_2) / b_0$ 中，$b_0$ 决定了控制量输出的整体尺度基准：$b_0$ 选取偏大时，控制器输出相对平缓温和；$b_0$ 选取偏小时，控制器输出更为激进。

### 3.2 $\omega_c$（Controller Bandwidth，控制器闭环带宽）
- **工程含义**：决定误差比例反馈项 $\omega_c z_1$ 的调节刚度与闭环响应快慢。
- **关键技术说明**：
  - $\omega_c$ 越大，系统对姿态残差收敛越迅速；
  - 过大的 $\omega_c$ 会放大高频传感噪声并引发执行器抖动；本项目三轴取值在 $0.25 \sim 1.0\,\text{rad/s}$ 之间。

### 3.3 $\omega_o$（Observer Bandwidth，扩张状态观测器带宽）
- **工程含义**：表征线性扩张状态观测器（LESO）对动平台状态 $z_1$ 与等效集总扰动项 $z_2$ 的跟踪估计速度与动态敏捷度。
- **关键技术说明**：
  - $\omega_o$ 决定了扰动估计的相位滞后大小；$\omega_o$ 越高，对外部晃动和偏心力矩等效影响的捕获越敏捷；
  - $\omega_o$ 过高会放大高频噪声和采样延时引起的抖动；工程整定中 $\omega_o \approx (3 \sim 5) \omega_c$ 属于常见的经验整定参考（common tuning heuristic, not a universal requirement），并非固定或普适的必然准则。

### 3.4 residual clamp / $u_{\text{limit}}$（残差控制量输出限幅）
- **工程含义**：对控制器每周期计算出的残差补偿量实施的对称硬截断阈值（本项目三轴严格统一设置为 $\pm 0.5^\circ$）。
- **关键技术说明**：
  - **残差限幅是工程落地中防止执行器越界、保护电机行程和多层控制解耦的安全物理护栏**；
  - **clamp 绝不是理论 ADRC 算法公式中天然存在的自适应调节参数**；
  - 一旦所需控制量超出限幅（$|u_{\text{raw}}| > u_{\text{limit}}$），系统即进入非线性饱和状态，未被抵消的残余扰动将直接暴露并表现为姿态跟踪误差。

### 3.5 $z_1$（State Estimate，状态估计值）
- **工程含义**：LESO 基于历史控制量 $u_{\text{actual}}$ 与当前 IMU 姿态误差测量值滤波重构出的姿态残差状态估计（单位：rad 或 deg）。
- **关键技术说明**：$z_1$ 相比原始测量反馈具有滤波平滑效果，抑制了部分瞬时测量噪声。

### 3.6 $z_2$（Equivalent Lumped Disturbance，等效集总扰动项）
- **工程含义**：LESO 实时估计出的等效集总扰动项（单位：rad/s 或 deg/s）。
- **关键技术说明**：
  - **$z_2$ 绝非真实力矩、真实扰动力矩、物理扭矩或扰动变化率**，其实际量纲为角速度量纲（rad/s 或 deg/s）；
  - $z_2$ 将 6PUS 机构内部未建模动力学（如连杆摩擦、铰链间隙、几何装配公差、偏心重物的附加重力效应）与外界未知扰动（如基座人工晃动输入）统摄为一个“等效集总扰动项”；
  - **$z_2$ 的实时前馈补偿是 LADRC 区别于经典 P/PD 控制的核心代差机制**，使系统无需依赖积分环节（I）的时间累积即可主动消除稳态与动态偏差。

---

## 4. 本项目当前生产参数配置

| 通道 | $b_0$ ($\text{s}^{-1}$) | $\omega_c$ (rad/s) | $\omega_o$ (rad/s) | 观测器状态更新 | 残差限幅 |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Roll**  | 20.0 | 0.80 | 2.50 | 闭式 ZOH 离散递推 | $\pm 0.5^\circ$ |
| **Pitch** | 10.0 | 1.00 | 4.00 | 闭式 ZOH 离散递推 | $\pm 0.5^\circ$ |
| **Yaw**   | 6.0  | 0.25 | 1.20 | 闭式 ZOH 离散递推 | $\pm 0.5^\circ$ |
