//============================================================
// 模块名称：radar_buzzer
// 功能说明：
//   根据超声波距离驱动 AX301 板载蜂鸣器，实现类似倒车雷达的声音提示。
//
// 设计思路：
//   1. 先把距离划分为 0~4 级报警等级；
//   2. 再根据报警等级选择不同的蜂鸣器音调频率；
//   3. 同时根据报警等级选择不同的间歇节奏；
//   4. 距离越近，音调更高、间隔更短；最近距离持续鸣叫。
//
// 距离单位：
//   distance 输入单位为 0.001cm，与 echo_driver / seg_driver / uart_report_tx 保持一致。
//
// 硬件极性：
//   AX301 板载蜂鸣器为低电平有效。
//   因此本模块输出 buzzer = 1 时关闭蜂鸣器，输出低电平脉冲时发声。
//
// 时基说明：
//   - clk 为 50MHz 系统时钟，用来产生蜂鸣器音调 PWM；
//   - clk_us 为 1us 使能脉冲，用来产生毫秒级间歇节奏。
//============================================================
module radar_buzzer(
    input  wire        clk,      // 50MHz 系统时钟
    input  wire        clk_us,   // 1us 使能脉冲
    input  wire        rstn,     // 低有效复位
    input  wire [18:0] distance, // 距离输入，单位 0.001cm
    output wire        buzzer    // 蜂鸣器输出，低电平有效
);

    //========================================================
    // 距离阈值
    // 与 led_bar_driver 保持同一套距离分段，方便 LED 与声音同步理解。
    //========================================================
    localparam [18:0] DIST_25CM  = 19'd25_000;
    localparam [18:0] DIST_50CM  = 19'd50_000;
    localparam [18:0] DIST_75CM  = 19'd75_000;
    localparam [18:0] DIST_100CM = 19'd100_000;

    //========================================================
    // 报警等级
    //   0：关闭
    //   1：远距离，慢速提示
    //   2：中远距离，中速提示
    //   3：中近距离，快速提示
    //   4：近距离，持续提示
    //========================================================
    reg [2:0] alarm_level;

    always @(*) begin
        if ((distance == 19'd0) || (distance > DIST_100CM))
            alarm_level = 3'd0;
        else if (distance > DIST_75CM)
            alarm_level = 3'd1;
        else if (distance > DIST_50CM)
            alarm_level = 3'd2;
        else if (distance > DIST_25CM)
            alarm_level = 3'd3;
        else
            alarm_level = 3'd4;
    end

    //========================================================
    // 音调 PWM 参数
    // 50MHz 时钟下，方波频率约等于：
    //   Fout = 50_000_000 / (2 * tone_half_period)
    //
    // 这里随报警等级提高音调：
    //   level 1：约 1.0kHz
    //   level 2：约 1.5kHz
    //   level 3：约 2.0kHz
    //   level 4：约 2.5kHz
    //========================================================
    reg [15:0] tone_half_period;

    always @(*) begin
        case (alarm_level)
            3'd1: tone_half_period = 16'd25_000; // 1000Hz
            3'd2: tone_half_period = 16'd16_667; // 约 1500Hz
            3'd3: tone_half_period = 16'd12_500; // 2000Hz
            3'd4: tone_half_period = 16'd10_000; // 2500Hz
            default: tone_half_period = 16'd25_000;
        endcase
    end

    // tone_cnt 用于累计一个半周期内的 50MHz 时钟数；计满后翻转一次
    // tone_square。连续翻转得到占空比接近 50% 的方波。
    reg [15:0] tone_cnt;
    reg        tone_square;

    //========================================================
    // 音调方波发生器
    // alarm_level 为 0 时关闭音调，避免无目标时输出脉冲。
    //========================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            tone_cnt    <= 16'd0;
            tone_square <= 1'b0;
        end else if (alarm_level == 3'd0) begin
            tone_cnt    <= 16'd0;
            tone_square <= 1'b0;
        end else if (tone_cnt >= tone_half_period - 16'd1) begin
            tone_cnt    <= 16'd0;
            tone_square <= ~tone_square;
        end else begin
            tone_cnt <= tone_cnt + 16'd1;
        end
    end

    //========================================================
    // 间歇节奏参数
    // envelope_period_ms：一个“响 + 停”周期的总时长；
    // beep_on_ms        ：每个周期内真正发声的时长。
    //
    // level 1：100ms 响，900ms 停；
    // level 2：100ms 响，400ms 停；
    // level 3：100ms 响，150ms 停；
    // level 4：持续响，不再间歇。
    //========================================================
    reg [9:0] envelope_period_ms;
    reg [9:0] beep_on_ms;

    always @(*) begin
        case (alarm_level)
            3'd1: begin
                envelope_period_ms = 10'd1000;
                beep_on_ms         = 10'd100;
            end
            3'd2: begin
                envelope_period_ms = 10'd500;
                beep_on_ms         = 10'd100;
            end
            3'd3: begin
                envelope_period_ms = 10'd250;
                beep_on_ms         = 10'd100;
            end
            default: begin
                envelope_period_ms = 10'd1;
                beep_on_ms         = 10'd1;
            end
        endcase
    end

    reg [9:0] us_cnt;        // 0~999，累计 1000 个 clk_us 得到 1ms
    reg [9:0] envelope_ms;   // 当前间歇周期内的毫秒位置
    reg [2:0] alarm_level_d; // 上一拍报警等级，用来在等级变化时重启提示节奏

    //========================================================
    // 毫秒节奏计数器
    // 只有在 clk_us 到来时才推进，因此不会额外生成新时钟域。
    // 当报警等级变化时，从周期起点重新开始，使声音节奏切换更干净，
    // 也避免把上一级报警的剩余停顿时间带入新一级报警。
    //========================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            us_cnt        <= 10'd0;
            envelope_ms   <= 10'd0;
            alarm_level_d <= 3'd0;
        end else begin
            alarm_level_d <= alarm_level;

            if ((alarm_level == 3'd0) || (alarm_level != alarm_level_d)) begin
                us_cnt      <= 10'd0;
                envelope_ms <= 10'd0;
            end else if (clk_us) begin
                if (us_cnt == 10'd999) begin
                    us_cnt <= 10'd0;

                    if (envelope_ms >= envelope_period_ms - 10'd1)
                        envelope_ms <= 10'd0;
                    else
                        envelope_ms <= envelope_ms + 10'd1;
                end else begin
                    us_cnt <= us_cnt + 10'd1;
                end
            end
        end
    end

    //========================================================
    // 发声包络
    // level 4 持续响；level 1~3 只在 beep_on_ms 时间内允许 PWM 输出。
    // beep_enable 只是音调方波的门控信号，不改变 tone_square 的频率；
    // 因此每次进入响铃阶段时仍可保持自然的方波相位。
    //========================================================
    wire beep_enable;

    assign beep_enable = (alarm_level == 3'd4) ? 1'b1 :
                         (alarm_level == 3'd0) ? 1'b0 :
                         (envelope_ms < beep_on_ms);

    //========================================================
    // 蜂鸣器低有效输出
    // tone_square & beep_enable 为 1 时输出低电平，让蜂鸣器发声；
    // 其他时间输出高电平，蜂鸣器关闭。
    //========================================================
    assign buzzer = ~(tone_square & beep_enable);

endmodule
