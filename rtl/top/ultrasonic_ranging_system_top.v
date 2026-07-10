//============================================================
// 模块名称：ultrasonic_ranging_system_top
// 功能说明：
//   工程顶层模块，负责把时基、超声波测距、温度采集、板载显示、串口通信
//   与 LCD 波形显示等子模块连接为完整的超声波测距系统。
//
// 时钟与时基：
//   - clk 是 AX301 板载 50MHz 主时钟，除 LCD 像素扫描外的同步逻辑均使用它；
//   - clk_div 从主时钟生成单周期 clk_us 使能，不在工程内派生新的微秒时钟；
//   - lcd_video_pll 将 50MHz 转换为 9MHz video_clk，供 LCD 显示时序专用；
//   - wave_sample_bridge 通过“稳定数据总线 + 翻转标志”把 50MHz 域的距离
//     样点安全传递至 9MHz LCD 时钟域。
//
// 功能数据流：
//   1. trig_driver 每 100ms 输出一次 15us TRIG 高脉冲，启动 HC-SR04 测距；
//   2. ds18b20_ctrl 周期读取环境温度，为声速补偿提供 temp_c；
//   3. echo_driver 统计 ECHO 高电平宽度，并依据 temp_c 计算距离 data_o；
//   4. data_o 同时送往数码管、LED 距离条、蜂鸣器、串口与 LCD 波形显示；
//   5. uart_rx 仅保存最近一次接收字节，作为后续扩展上位机命令的接口。
//
// 设计边界：
//   本模块只承担顶层连接、时钟域衔接与预留数据锁存，不在此处实现具体外设时序。
//============================================================
module ultrasonic_ranging_system_top(
    //========================================================
    // 系统时钟与复位
    //========================================================
    input  wire        clk,        // AX301 板载 50MHz 系统时钟，所有同步逻辑都以它为主时钟
    input  wire        rstn,       // 板载低有效复位，0 表示复位，1 表示正常运行

    //========================================================
    // 外部传感器与串口输入
    //========================================================
    input  wire        echo,       // HC-SR04 回响输入，脉宽代表声波往返时间
    inout  wire        ds18b20_dq, // DS18B20 单总线 DQ，既要输出复位/命令，也要释放后读数据
    input  wire        uart_rx,    // 串口接收输入，当前作为后续命令控制预留

    //========================================================
    // 板载外设输出
    //========================================================
    output wire        trig,       // HC-SR04 触发输出，高脉冲启动一次测距
    output wire        uart_tx,    // 串口发送输出，向上位机周期发送距离和温度
    output wire [5:0]  sel,        // 6 位数码管位选，低电平有效
    output wire [7:0]  seg,        // 数码管段选，低电平有效，seg[7] 为小数点
    output wire [3:0]  led,        // AX301 板载 4 个 LED，高电平点亮，用作距离条
    output wire        buzzer,     // AX301 板载蜂鸣器，低电平有效，用作倒车雷达提示音

    //========================================================
    // AN430 RGB LCD 输出
    //========================================================
    output wire        lcd_dclk,   // AN430 LCD 像素时钟，输出为 video_clk 的反相
    output wire        lcd_hs,     // AN430 LCD 行同步，低有效
    output wire        lcd_vs,     // AN430 LCD 场同步，低有效
    output wire        lcd_de,     // AN430 LCD 数据有效
    output wire [7:0]  lcd_r,      // AN430 LCD 红色 8 位数据
    output wire [7:0]  lcd_g,      // AN430 LCD 绿色 8 位数据
    output wire [7:0]  lcd_b       // AN430 LCD 蓝色 8 位数据
);

    //========================================================
    // 测距、温度与公共时基
    //
    // data_o 是工程内部统一使用的距离格式：单位为 0.001cm，例如
    // 19'd123456 代表 123.456cm。所有需要距离数据的子模块均直接使用该总线。
    //========================================================
    wire [18:0] data_o;       // echo_driver 输出的温度补偿距离结果
    wire        clk_us;       // 1us 使能脉冲，宽度为 1 个 50MHz 时钟周期
    wire signed [7:0] temp_c; // DS18B20 输出的整数摄氏温度，用于声速补偿
    wire        temp_valid;   // 首次成功读取温度后置位；当前保留，便于后续增加温度异常处理

    //========================================================
    // 串口接收与 LCD 显示内部连线
    //========================================================
    wire [7:0]  uart_rx_data;       // UART 接收完成的 1 字节数据
    wire        uart_rx_valid;      // UART 接收完成标志；顶层将其锁存为最近一次接收事件
    wire        video_clk;          // PLL 生成的 9MHz LCD 像素时钟
    wire [18:0] wave_sample_data;   // 在 50MHz 域保持 100ms 的距离样点
    wire        wave_sample_toggle; // 每写入一个新样点翻转一次，作为跨时钟更新事件

    //========================================================
    // 串口接收预留寄存器
    //
    // 当前工程未定义上位机命令协议，因此只锁存最新字节和接收完成事件。
    // preserve/noprune/keep 是 Quartus 综合属性，使这些尚未参与功能控制的
    // 寄存器及其关联逻辑不会被优化删除；后续可在此扩展阈值设置、模式切换等命令。
    //========================================================
    reg [7:0] uart_rx_data_hold /* synthesis preserve noprune */;
    reg       uart_rx_valid_hold /* synthesis preserve noprune */;
    wire      uart_rx_reserved_keep /* synthesis keep */;

    // 异或归约使保留网络同时依赖有效标志和数据内容，避免仅保留孤立寄存器。
    assign uart_rx_reserved_keep = uart_rx_valid_hold ^ (^uart_rx_data_hold);

    //========================================================
    // LED 距离条
    //
    // 距离越近，点亮的 LED 数量越多；距离为 0 或超过报警范围时全灭。
    //========================================================
    led_bar_driver u_led_bar_driver(
        .distance   (data_o),
        .led        (led   )
    );

    //========================================================
    // 蜂鸣器倒车雷达提示
    //
    // 距离越近，蜂鸣器提示越急；近距离时持续鸣叫。
    //========================================================
    radar_buzzer u_radar_buzzer(
        .clk        (clk    ),
        .clk_us     (clk_us ),
        .rstn       (rstn   ),
        .distance   (data_o ),
        .buzzer     (buzzer )
    );

    //========================================================
    // 6 位数码管显示驱动
    //
    // data_o 的单位是 0.001cm，seg_driver 直接显示为 xxx.xxx。
    //========================================================
    seg_driver u_seg_driver(
        .clk        (clk    ),
        .rstn       (rstn   ),
        .data_in    (data_o ),
        .sel        (sel    ),
        .seg        (seg    )
    );

    //========================================================
    // 1us 时基产生模块
    //
    // clk_us 是同步时钟使能，统一供超声波触发、回响计时、DS18B20 时序、
    // 波形采样和串口周期发送使用；它不是独立的时钟网络。
    //========================================================
    clk_div u_clk_div(
        .clk        (clk    ),
        .rstn       (rstn   ),
        .clk_us     (clk_us )
    );

    //========================================================
    // AN430 LCD 实时距离波形显示
    //
    // 50MHz 时钟经 PLL 生成 9MHz 像素时钟；测距结果每 100ms 采样一次，
    // 屏幕保留 110 个样点，即约 11 秒的滑动时间窗。显示模块的纵轴为
    // 0~300cm，横轴每个网格代表 1 秒，纵轴每个网格代表 50cm。
    //
    // 下方三个实例依次完成像素时钟生成、跨时钟样点桥接和 RGB 时序输出。
    //========================================================
    lcd_video_pll u_lcd_video_pll(
        .inclk0 (clk      ),
        .c0     (video_clk)
    );

    // 在主时钟域按 10Hz 采样距离；sample_toggle 用于通知 LCD 域出现新样点。
    wave_sample_bridge u_wave_sample_bridge(
        .clk           (clk               ),
        .clk_us        (clk_us            ),
        .rstn          (rstn              ),
        .distance      (data_o            ),
        .sample_data   (wave_sample_data  ),
        .sample_toggle (wave_sample_toggle)
    );

    // LCD 显示模块接收跨时钟样点，维护历史缓存并输出 HS、VS、DE 与 RGB 数据。
    lcd_wave_display u_lcd_wave_display(
        .clk                 (video_clk         ),
        .rstn                (rstn              ),
        .sample_data_async   (wave_sample_data  ),
        .sample_toggle_async (wave_sample_toggle),
        .lcd_hs              (lcd_hs            ),
        .lcd_vs              (lcd_vs            ),
        .lcd_de              (lcd_de            ),
        .lcd_r               (lcd_r             ),
        .lcd_g               (lcd_g             ),
        .lcd_b               (lcd_b             )
    );

    // 面板在反相后的像素时钟边沿采样 RGB 数据；显示模块在 video_clk 上升沿
    // 更新时序和颜色数据，因此将 video_clk 反相后作为 lcd_dclk 输出。
    assign lcd_dclk = ~video_clk;

    //========================================================
    // DS18B20 温度采集模块
    //
    // 每约 1 秒完成一次温度读取。temp_c 在读数完成时更新，随后由 echo_driver
    // 用于计算当前声速；temp_valid 首次取得有效温度后保持置位。
    //========================================================
    ds18b20_ctrl u_ds18b20_ctrl(
        .clk        (clk        ),
        .clk_us     (clk_us     ),
        .rstn       (rstn       ),
        .dq         (ds18b20_dq ),
        .temp_c     (temp_c     ),
        .temp_valid (temp_valid )
    );

    //========================================================
    // HC-SR04 触发模块
    //
    // 周期性产生 TRIG 高脉冲，启动传感器发射超声波；本模块只负责发起测量，
    // 回波宽度的同步、计时与距离换算由下方 echo_driver 完成。
    //========================================================
    trig_driver u_trig_driver(
        .clk        (clk    ),
        .clk_us     (clk_us ),
        .rstn       (rstn   ),
        .trig       (trig   )
    );

    //========================================================
    // HC-SR04 回响测距模块
    //
    // 统计 echo 高电平持续时间，并使用 temp_c 做温度补偿。其输出 data_o 是
    // 整个系统的距离数据源，会被并行送至本文件中所有显示和通信功能。
    //========================================================
    echo_driver u_echo_driver(
        .clk        (clk    ),
        .clk_us     (clk_us ),
        .rstn       (rstn   ),
        .echo       (echo   ),
        .temp_c     (temp_c ),
        .data_o     (data_o )
    );

    //========================================================
    // 串口上报模块
    //
    // 每秒把当前 data_o 与 temp_c 转为 ASCII 文本并发送，例如：
    // "D=123.456cm,T=+020C\r\n"。发送期间会在模块内部锁存一帧数据，
    // 从而避免距离或温度更新造成同一帧报文前后不一致。
    //========================================================
    uart_report_tx u_uart_report_tx(
        .clk        (clk    ),
        .clk_us     (clk_us ),
        .rstn       (rstn   ),
        .distance   (data_o ),
        .temp_c     (temp_c ),
        .uart_tx    (uart_tx)
    );

    //========================================================
    // 串口接收模块
    //
    // 采用 115200bps、8N1 格式。当前作为功能预留，接收到的字节由下方
    // always 块锁存，rx_data_ready 固定为 1，表示顶层始终接受下一字节。
    //========================================================
    uart_rx #(
        .CLK_FRE   (50),
        .BAUD_RATE (115200)
    ) u_uart_rx (
        .clk           (clk),
        .rstn          (rstn),
        .rx_data       (uart_rx_data),
        .rx_data_valid (uart_rx_valid),
        .rx_data_ready (1'b1),
        .rx_pin        (uart_rx)
    );

    //========================================================
    // 串口接收预留锁存
    //
    // rx_data_ready 恒为 1，uart_rx_valid 因而只在接收完成后短暂有效。
    // 本 always 块以 50MHz 时钟锁存最近一次收到的数据，并同步保存该有效事件；
    // 后续增加命令解析时，可直接以 uart_rx_valid_hold 和 uart_rx_data_hold 为入口。
    //========================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            uart_rx_data_hold  <= 8'd0;
            uart_rx_valid_hold <= 1'b0;
        end else begin
            uart_rx_valid_hold <= uart_rx_valid;
            if (uart_rx_valid)
                uart_rx_data_hold <= uart_rx_data;
        end
    end

endmodule
