# AX301板载50MHz有源晶振
create_clock -name clk -period 20.000 [get_ports {clk}]

# echo来自异步超声波模块，仅在第一级同步寄存器前切断时序分析
set_false_path -from [get_ports {echo}] -to [get_registers {*|r1_echo}]
set_false_path -from [get_ports {ds18b20_dq}] -to [get_registers {*|dq_meta}]
set_false_path -from [get_ports {uart_rx}] -to [get_registers {*|rx_d0}]

# rstn为板载异步复位按键
set_false_path -from [get_ports {rstn}]

# 数码管与HC-SR04 TRIG均为异步外设输出，不需要外部同步接口时序
set_false_path -to [get_ports {trig}]
set_false_path -to [get_ports {ds18b20_dq}]
set_false_path -to [get_ports {uart_tx}]
set_false_path -to [get_ports {sel[*]}]
set_false_path -to [get_ports {seg[*]}]

derive_clock_uncertainty
