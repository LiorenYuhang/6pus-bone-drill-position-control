# J13 动平台偏心负载实验交接包 (Classical vs 3-axis LADRC)

## 1. 本交接包简介
本交接包汇集了骨科手术机器人 6PUS 并联平台在 J13 阶段偏心负载工况下，两组 10 s 真实硬件运行（Classical controller 与 3-axis LADRC）的完整原始数据、统计分析表、高清对比图件、离线处理脚本以及技术说明文档。所有材料均采用自包含结构组织，便于在 Windows 操作系统上直接开展离线数据审查、报表查阅与技术整理。

---

## 2. 原始数据来源
- **Classical 组数据路径（Linux 原机）**：  
  `/home/lyapunov/bone_drill/results/j13/j13_2a2_shadow/20261007_143007/j13_3axis_payload_classical_10s/`
- **3-axis LADRC 组数据路径（Linux 原机）**：  
  `/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/`

---

## 3. 两组控制器结构定义
- **Classical Controller**：  
  $\text{Base SO(3) feedforward} + \text{Roll P} + \text{Pitch PD} + \text{Yaw PD}$  
  （残差输出限幅：$\pm 0.5^\circ$）
- **3-axis LADRC**：  
  $\text{Base SO(3) feedforward} + \text{3-axis LADRC residual control}$  
  （残差输出限幅：$\pm 0.5^\circ$；三轴分别运行一阶离散化线性自抗扰算法）

---

## 4. 文件夹结构说明
```text
J13_payload_AB_win_package_20261007/
|-- 00_README.md                  # 本说明文件
|-- 01_report/                    # 分析报告与参数技术说明
|   |-- J13_payload_AB_analysis.md         # 偏心负载 A/B 对比技术分析报告
|   |-- controller_parameter_notes.md      # LADRC 核心参数与状态变量工程解释
|-- 02_figures/                   # 全部 10 张高清分析图件 (PNG)
|   |-- fig01_base_roll_comparison.png
|   |-- fig02_base_pitch_comparison.png
|   |-- fig03_base_yaw_comparison.png
|   |-- fig04_roll_error_comparison.png
|   |-- fig05_pitch_error_comparison.png
|   |-- fig06_yaw_error_comparison.png
|   |-- fig07_classical_control_output.png
|   |-- fig08_ladrc_control_output.png
|   |-- fig09_ladrc_z2_states.png
|   |-- fig10_initial_static_bias_zoom.png
|-- 03_tables/                    # 统计与配置汇总表 (CSV 格式，Excel 友好)
|   |-- controller_parameters.csv          # 控制器各通道增益与结构参数
|   |-- experiment_conditions.csv          # 实验阶段元数据与运行条件
|   |-- base_excitation_metrics.csv        # 基座输入晃动幅值统计与可比性
|   |-- platform_error_metrics.csv         # 动平台姿态误差指标对比 (RMS/MAE/P95/Peak)
|   |-- control_output_metrics.csv         # 控制器输出幅值与饱和率统计
|   |-- ladrc_observer_metrics.csv         # LESO 观测状态与等效集总扰动项统计
|   |-- runtime_health_metrics.csv         # 底层硬件与通信调度健康指标
|   |-- figure_index.csv                   # 图件清单、数据源及坐标轴说明
|-- 04_raw_data/                  # 自包含的原始实验数据副本
|   |-- classical/                         # Classical 组原始运行数据与遥测 CSV/JSON
|   |-- ladrc/                             # 3-axis LADRC 组原始运行数据与遥测 CSV/JSON
|-- 05_analysis_scripts/          # 离线统计与自动出图的 Python 源码
    |-- process_payload_ab.py              # 数据处理、指标计算与绘图主脚本
```

---

## 5. CSV 表格用途说明
1. `controller_parameters.csv`：记录两组实验各通道控制律形式、增益配置（$K_p, K_d$ 或 $b_0, \omega_c, \omega_o$）及限幅；
2. `experiment_conditions.csv`：记录两组实验的运行标识、周期数、时长、接管时刻及异常计数；
3. `base_excitation_metrics.csv`：三轴 Base 输入激励的 RMS、P95 与 Peak 统计，说明两次外部晃动强度的客观差异；
4. `platform_error_metrics.csv`：ACTIVE 阶段动平台姿态误差的 RMS、MAE、P95、Peak 及相对变化率；
5. `control_output_metrics.csv`：两组控制器输出补偿角的 RMS、峰值及 $\pm 0.5^\circ$ 限幅饱和比例；
6. `ladrc_observer_metrics.csv`：LADRC 扩张状态观测器（LESO）的 $z_1$ 与 $z_2$（等效集总扰动项）统计；
7. `runtime_health_metrics.csv`：电机跟踪误差、相电流、数据延时及 UART 吞吐等底层指标（仅标注代码证实阈值）；
8. `figure_index.csv`：所有图件的文件名、数据源、横纵坐标量及客观事实描述清单。

---

## 6. PNG 图件用途说明
- `fig01_base_roll_comparison.png`：Base Roll 外部晃动角度时间序列对比；
- `fig02_base_pitch_comparison.png`：Base Pitch 外部晃动角度时间序列对比；
- `fig03_base_yaw_comparison.png`：Base Yaw 外部晃动角度时间序列对比；
- `fig04_roll_error_comparison.png`：动平台 Roll 姿态跟踪误差时间序列对比；
- `fig05_pitch_error_comparison.png`：动平台 Pitch 姿态跟踪误差时间序列对比；
- `fig06_yaw_error_comparison.png`：动平台 Yaw 姿态跟踪误差时间序列对比；
- `fig07_classical_control_output.png`：Classical 控制器各通道残差输出与 $\pm 0.5^\circ$ 限幅线对比；
- `fig08_ladrc_control_output.png`：3-axis LADRC 各通道残差输出与 $\pm 0.5^\circ$ 限幅线对比（显现 Pitch 饱和）；
- `fig09_ladrc_z2_states.png`：LADRC 观测器估计的各通道等效集总扰动项 $z_2$（deg/s）演变过程；
- `fig10_initial_static_bias_zoom.png`：$0 \sim 1.5\,\text{s}$ 静态阶段姿态误差放大图（展示偏心重力初始下垂）。

---

## 7. 已知实验边界（客观事实约束）
在查阅和引用本交接包数据时，须严格遵循以下客观边界：
1. **偏心负载质量未测量**：实验前未对装夹的重物进行精密称重；
2. **偏心距离未测量**：未对重物几何中心到平台原点的实际力臂进行精密物理量测；
3. **同一固定重物条件**：两组实验在相同的装夹物理状态下连续开展，重物未发生拆卸或位移；
4. **Base 为人工扰动**：基座晃动源于人工摇晃台面，属于外部施加的随机连续扰动；
5. **两组 Base 激励并不严格一致**：3-axis LADRC 组 Base Roll RMS amplitude 约为 Classical 组的 1.9 倍，Base Pitch RMS amplitude 约为 Classical 组的 1.5 倍，两组测试非严格同源输入；
6. **阶段性工程数据属性**：本数据集用于反映当前工程阶段真实机载控制行为与饱和现象，不能等同于受控严格统计学结论，严禁推演为“LADRC 全面优于 Classical”。
