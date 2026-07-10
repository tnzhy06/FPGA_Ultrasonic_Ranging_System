//============================================================
// 模块名称：HC_SR04_TOP
// 功能说明：
//   工程顶层模块，负责把各个功能子模块连接成完整系统。
//
// 系统数据流：
//   1. clk_div 产生 1us 使能脉冲 clk_us；
//   2. trig_driver 按 100ms 周期驱动 HC-SR04 的 TRIG；
//   3. echo_driver 统计 ECHO 高电平宽度，并结合 DS18B20 温度做声速补偿；
//   4. seg_driver 将距离显示到 6 位数码管，格式为 123.456；
//   5. led_bar_driver 根据距离点亮 0~4 个板载 LED，形成距离条；
//   6. radar_buzzer 根据距离输出不同节奏/音调的蜂鸣器报警；
//   7. uart_report_tx 每秒向上位机发送距离和温度文本；
//   8. uart_rx 已预留接收通道，当前只锁存接收数据，不参与控制。
//
// 注意：
//   本模块只做顶层连线，不直接实现具体外设时序。
//============================================================
module HC_SR04_TOP(
    input  wire        clk,        // AX301 板载 50MHz 系统时钟，所有同步逻辑都以它为主时钟
    input  wire        rstn,       // 板载低有效复位，0 表示复位，1 表示正常运行

    input  wire        echo,       // HC-SR04 回响输入，脉宽代表声波往返时间
    inout  wire        ds18b20_dq, // DS18B20 单总线 DQ，既要输出复位/命令，也要释放后读数据
    input  wire        uart_rx,    // 串口接收输入，当前作为后续命令控制预留
    output wire        trig,       // HC-SR04 触发输出，高脉冲启动一次测距
    output wire        uart_tx,    // 串口发送输出，向上位机周期发送距离和温度
    output wire [5:0]  sel,        // 6 位数码管位选，低电平有效
    output wire [7:0]  seg,        // 数码管段选，低电平有效，seg[7] 为小数点
    output wire [3:0]  led,        // AX301 板载 4 个 LED，高电平点亮，用作距离条
    output wire        buzzer      // AX301 板载蜂鸣器，低电平有效，用作倒车雷达提示音
);

    //========================================================
    // 顶层内部连线
    //========================================================
    wire [18:0] data_o;       // 距离结果，单位为 0.001cm，例如 123456 表示 123.456cm
    wire        clk_us;       // 1us 使能脉冲，宽度为 1 个 50MHz 时钟周期
    wire signed [7:0] temp_c; // DS18B20 输出的整数摄氏温度，用于声速补偿
    wire        temp_valid;   // 温度有效标志，当前预留观察用，温度寄存器本身会保持最新值
    wire [7:0]  uart_rx_data; // 串口接收到的 1 字节数据，当前只保留，不解释
    wire        uart_rx_valid;// 串口接收完成脉冲，表示 uart_rx_data 中有新字节

    //========================================================
    // 串口接收预留寄存器
    // preserve/noprune 是给 Quartus 的保留属性，用于防止删除这些预留寄存器。
    // 当前工程还没有定义上位机命令协议，因此只把接收到的数据锁存下来，
    // 后续若要加阈值设置、模式切换等功能，可以从这里继续扩展。
    //========================================================
    reg [7:0] uart_rx_data_hold /* synthesis preserve noprune */;
    reg       uart_rx_valid_hold /* synthesis preserve noprune */;
    wire      uart_rx_reserved_keep /* synthesis keep */;

    assign uart_rx_reserved_keep = uart_rx_valid_hold ^ (^uart_rx_data_hold);

    //========================================================
    // LED 距离条
    // 距离越近，点亮的 LED 数量越多；距离为 0 或超过报警范围时全灭。
    //========================================================
    led_bar_driver u_led_bar_driver(
        .distance   (data_o),
        .led        (led   )
    );

    //========================================================
    // 蜂鸣器倒车雷达提示
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
    // 统一给超声波触发、回响计时、DS18B20 时序、串口周期发送使用。
    //========================================================
    clk_div u_clk_div(
        .clk        (clk    ),
        .rstn       (rstn   ),
        .clk_us     (clk_us )
    );

    //========================================================
    // DS18B20 温度采集模块
    // 每约 1 秒完成一次温度读取，temp_c 在读数完成时更新。
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
    // 周期性产生 TRIG 高脉冲，启动传感器发射超声波。
    //========================================================
    trig_driver u_trig_driver(
        .clk        (clk    ),
        .clk_us     (clk_us ),
        .rstn       (rstn   ),
        .trig       (trig   )
    );

    //========================================================
    // HC-SR04 回响测距模块
    // 统计 echo 高电平持续时间，并使用 temp_c 做温度补偿。
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
    // 周期发送类似 "D=123.456cm,T=+020C\r\n" 的 ASCII 文本。
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
    // 当前作为功能预留，接收到的字节由下方 always 块锁存。
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
    // rx_data_ready 恒为 1，所以 uart_rx_valid 只会短暂拉高一个接收完成周期。
    // 这里把最近一次收到的字节保存下来，方便后续增加命令解析。
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
