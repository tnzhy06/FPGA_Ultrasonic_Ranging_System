//============================================================
// 模块名称：echo_driver
// 功能说明：
//   统计 HC-SR04 的 ECHO 高电平宽度，并计算温度补偿后的距离。
//
// 输入输出单位：
//   echo 高电平宽度 cnt 的单位：us；
//   temp_c 的单位：摄氏度，整数；
//   data_o 的单位：0.001cm，例如 123456 表示 123.456cm。
//
// 距离公式：
//   声速 v = 331.4 + 0.607T，单位 m/s；
//   ECHO 时间是声波往返时间，所以距离 = time * v / 2；
//   换算成 0.001cm 后：
//       distance_0.001cm = time_us * (331400 + 607*T) / 20000
//
// 实现方式：
//   使用 Q12 定点系数代替运行时除法，避免综合出硬件除法器。
//============================================================
module echo_driver(
    input  wire              clk,    // 50MHz 系统时钟
    input  wire              clk_us, // 1us 使能脉冲，用于 ECHO 脉宽计数
    input  wire              rstn,   // 异步复位，低有效

    input  wire              echo,   // HC-SR04 ECHO 输入，高电平宽度代表往返时间
    input  wire signed [7:0] temp_c, // 温度，单位摄氏度，用于声速补偿
    output wire [18:0]       data_o  // 距离输出，单位 0.001cm
);

    // 最大计数 29999us，约对应 5.1m 量程。
    // 超过该时间后计数饱和，避免无回波时计数溢出。
    parameter T_MAX = 16'd29_999;

    // Q12 定点系数：
    //   BASE_COEFF_Q12 ≈ 331400 / 20000 * 4096
    //   TEMP_COEFF_Q12 ≈ 607 / 20000 * 4096
    // 最终 distance ≈ cnt * (BASE + temp_c*TEMP) >> 12。
    localparam signed [20:0] BASE_COEFF_Q12 = 21'sd67_871;
    localparam signed [10:0] TEMP_COEFF_Q12 = 11'sd124;

    // echo 先经过两级寄存器同步，降低异步输入带来的亚稳态风险。
    reg r1_echo;
    reg r2_echo;

    wire echo_pos; // 同步后的上升沿：开始计时
    wire echo_neg; // 同步后的下降沿：结束计时并锁存距离

    reg [15:0] cnt;    // ECHO 高电平持续时间，单位 us
    reg [18:0] data_r; // 距离结果寄存器

    // 温度补偿后的定点乘法链路。
    wire signed [18:0] temp_coeff_delta;
    wire signed [20:0] distance_coeff_q12;
    wire [36:0]        distance_product;
    wire [24:0]        distance_shifted;
    wire [18:0]        distance_compensated;

    assign temp_coeff_delta     = temp_c * TEMP_COEFF_Q12;
    assign distance_coeff_q12   = BASE_COEFF_Q12 + temp_coeff_delta;
    assign distance_product     = cnt * distance_coeff_q12[20:0];
    assign distance_shifted     = distance_product[36:12];
    assign distance_compensated = (|distance_shifted[24:19]) ? 19'h7ffff
                                                             : distance_shifted[18:0];

    // ECHO 输入同步。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            r1_echo <= 1'b0;
            r2_echo <= 1'b0;
        end else begin
            r1_echo <= echo;
            r2_echo <= r1_echo;
        end
    end

    assign echo_pos =  r1_echo & ~r2_echo;
    assign echo_neg = ~r1_echo &  r2_echo;

    // ECHO 高电平宽度计数。
    // 只在 clk_us 有效并且同步后的 echo 为高时计数，因此 cnt 单位就是 us。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt <= 16'd0;
        end else if (echo_pos) begin
            cnt <= 16'd0;
        end else if (clk_us && r2_echo) begin
            if (cnt < T_MAX)
                cnt <= cnt + 16'd1;
            else
                cnt <= T_MAX;
        end else if (!r2_echo) begin
            cnt <= 16'd0;
        end
    end

    // 下降沿说明一次测距结束，此时把组合计算出的距离锁存到输出寄存器。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            data_r <= 19'd0;
        else if (echo_neg)
            data_r <= distance_compensated;
    end

    assign data_o = data_r;

endmodule
