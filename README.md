# HC-SR04 AX301 Project

本工程为 AX301 开发板上的超声波测距工程。根目录保留 Quartus 工程文件，RTL、约束和文档按功能分目录维护。

## 目录结构

```text
csb/
├─ csb.qpf                  Quartus 工程入口
├─ csb.qsf                  Quartus 工程配置、器件和引脚绑定
├─ rtl/
│  ├─ top/                  顶层集成模块
│  ├─ common/               通用时基、复位、跨模块公共逻辑
│  ├─ ultrasonic/           HC-SR04 超声波测距
│  ├─ ds18b20/              DS18B20 温度采集与补偿
│  ├─ seg/                  数码管驱动
│  ├─ uart/                 串口收发
│  ├─ led_bar/              LED 距离条，预留
│  └─ buzzer/               蜂鸣器报警，预留
├─ constraints/             SDC 时序约束
├─ docs/                    开发板手册、硬件说明
├─ output_files/            Quartus 输出文件
├─ db/                      Quartus 数据库，自动生成
├─ incremental_db/          Quartus 增量编译数据库，自动生成
└─ simulation/              Quartus/ModelSim 仿真输出
```

## 模块放置约定

- 新增顶层端口和子模块例化放在 `rtl/top/ultrasonic_ranging_system_top.v`。
- 串口收发模块放在 `rtl/uart/`。
- DS18B20 温度采集与声速补偿模块放在 `rtl/ds18b20/`。
- LED 距离条模块放在 `rtl/led_bar/`。
- 蜂鸣器报警模块放在 `rtl/buzzer/`。
- 新增 Verilog 文件后，需要同步在 `csb.qsf` 中添加 `set_global_assignment -name VERILOG_FILE ...`。
- 新增外设引脚后，需要在 `csb.qsf` 中添加位置约束和 `3.3-V LVTTL` 电平约束。
