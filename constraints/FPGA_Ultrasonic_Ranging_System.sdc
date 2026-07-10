# AX301板载50MHz有源晶振
create_clock -name clk -period 20.000 [get_ports {clk}]
derive_pll_clocks

# echo来自异步超声波模块，仅在第一级同步寄存器前切断时序分析
set_false_path -from [get_ports {echo}] -to [get_registers {*|r1_echo}]
set_false_path -from [get_ports {ds18b20_dq}] -to [get_registers {*|dq_meta}]
set_false_path -from [get_ports {uart_rx}] -to [get_registers {*|rx_d0}]

# 距离样点在 50MHz 域锁存后保持 100ms；LCD 的 9MHz 域先同步翻转标志，
# 再在额外一拍采样稳定的数据总线。因此这两条跨时钟路径不按单周期时序分析。
set_false_path -from [get_registers {*u_wave_sample_bridge|sample_data[*]}] -to [get_registers {*u_lcd_wave_display|sample_capture[*]}]
set_false_path -from [get_registers {*u_wave_sample_bridge|sample_toggle}] -to [get_registers {*u_lcd_wave_display|sample_toggle_meta}]

# rstn为板载异步复位按键
set_false_path -from [get_ports {rstn}]

# 数码管与HC-SR04 TRIG均为异步外设输出，不需要外部同步接口时序
set_false_path -to [get_ports {trig}]
set_false_path -to [get_ports {ds18b20_dq}]
set_false_path -to [get_ports {uart_tx}]
set_false_path -to [get_ports {sel[*]}]
set_false_path -to [get_ports {seg[*]}]
set_false_path -to [get_ports {lcd_dclk lcd_hs lcd_vs lcd_de}]
set_false_path -to [get_ports {lcd_r[*] lcd_g[*] lcd_b[*]}]

derive_clock_uncertainty
