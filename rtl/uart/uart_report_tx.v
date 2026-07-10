//============================================================
// 模块名称：uart_report_tx
// 功能说明：
//   把当前距离和温度转换为 ASCII 文本，并通过 UART 周期发送到上位机。
//
// 串口格式：
//   115200bps，8N1，由内部 uart_tx 模块完成实际串行发送。
//
// 报文格式：
//   D=123.456cm,T=+020C\r\n
//
// 输入单位：
//   distance：单位 0.001cm，直接转换成 xxx.xxxcm；
//   temp_c  ：有符号整数摄氏度，发送为 +020C 或 -005C。
//
// 资源优化：
//   距离和温度的十进制转换使用串行 Double-Dabble 算法；
//   不使用除法和取模，避免综合出大面积组合除法器。
//============================================================
module uart_report_tx(
    input  wire              clk,      // 50MHz 系统时钟
    input  wire              clk_us,   // 1us 使能脉冲，用于 1 秒发送周期计数
    input  wire              rstn,     // 异步复位，低有效

    input  wire [18:0]       distance, // 距离，单位 0.001cm
    input  wire signed [7:0] temp_c,   // 温度，单位摄氏度

    output wire              uart_tx   // UART TX 输出
);

    // 每 1 秒启动一次报文发送。
    // 计数 0~999999，共 1000000us。
    localparam [19:0] REPORT_INTERVAL_US = 20'd999_999;

    // 状态机：
    //   S_WAIT     ：等待 1 秒周期到达；
    //   S_DIST_BCD ：把距离 distance 转为 6 位 BCD；
    //   S_TEMP_BCD ：把温度绝对值转为 3 位 BCD；
    //   S_SEND     ：逐字节发送固定格式报文。
    localparam [2:0] S_WAIT     = 3'd0,
                     S_DIST_BCD = 3'd1,
                     S_TEMP_BCD = 3'd2,
                     S_SEND     = 3'd3;

    // 报文最后一个字节索引。
    // 0~20 共 21 字节，最后两个字节为 \r\n。
    localparam [4:0] LAST_BYTE = 5'd20;

    reg [2:0]  state;
    reg [19:0] interval_cnt;    // 1 秒周期计数，单位 us

    reg signed [7:0] temp_latch;     // 报文开始时锁存温度，保证一帧内不变化
    reg [7:0]  temp_abs_latch;       // 温度绝对值，用于 BCD 转换

    // 距离 BCD 转换寄存器。
    // dist_bcd 对应 6 个十进制数字：百位、十位、个位、0.1、0.01、0.001。
    reg [18:0] dist_shift;
    reg [23:0] dist_bcd_work;
    reg [23:0] dist_bcd;
    reg [4:0]  dist_bcd_cnt;
    reg [23:0] dist_bcd_adjust;

    // 温度 BCD 转换寄存器。
    // temp_bcd 对应 3 个十进制数字：百位、十位、个位。
    reg [7:0]  temp_shift;
    reg [11:0] temp_bcd_work;
    reg [11:0] temp_bcd;
    reg [3:0]  temp_bcd_cnt;
    reg [11:0] temp_bcd_adjust;

    // 串口发送控制。
    reg [4:0]  send_index;     // 当前发送报文中的第几个字符
    reg [7:0]  report_char;    // 根据 send_index 组合出的当前字符
    reg [7:0]  tx_data;        // 送给 uart_tx 的字节
    reg        tx_data_valid;  // 送给 uart_tx 的 valid
    wire       tx_data_ready;  // uart_tx 空闲标志

    //========================================================
    // 距离 Double-Dabble 加 3 修正
    //========================================================
    always @(*) begin
        dist_bcd_adjust = dist_bcd_work;

        if (dist_bcd_adjust[3:0] >= 4'd5)
            dist_bcd_adjust[3:0] = dist_bcd_adjust[3:0] + 4'd3;
        if (dist_bcd_adjust[7:4] >= 4'd5)
            dist_bcd_adjust[7:4] = dist_bcd_adjust[7:4] + 4'd3;
        if (dist_bcd_adjust[11:8] >= 4'd5)
            dist_bcd_adjust[11:8] = dist_bcd_adjust[11:8] + 4'd3;
        if (dist_bcd_adjust[15:12] >= 4'd5)
            dist_bcd_adjust[15:12] = dist_bcd_adjust[15:12] + 4'd3;
        if (dist_bcd_adjust[19:16] >= 4'd5)
            dist_bcd_adjust[19:16] = dist_bcd_adjust[19:16] + 4'd3;
        if (dist_bcd_adjust[23:20] >= 4'd5)
            dist_bcd_adjust[23:20] = dist_bcd_adjust[23:20] + 4'd3;
    end

    //========================================================
    // 温度 Double-Dabble 加 3 修正
    //========================================================
    always @(*) begin
        temp_bcd_adjust = temp_bcd_work;

        if (temp_bcd_adjust[3:0] >= 4'd5)
            temp_bcd_adjust[3:0] = temp_bcd_adjust[3:0] + 4'd3;
        if (temp_bcd_adjust[7:4] >= 4'd5)
            temp_bcd_adjust[7:4] = temp_bcd_adjust[7:4] + 4'd3;
        if (temp_bcd_adjust[11:8] >= 4'd5)
            temp_bcd_adjust[11:8] = temp_bcd_adjust[11:8] + 4'd3;
    end

    //========================================================
    // 报文生成主状态机
    //========================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state          <= S_WAIT;
            interval_cnt   <= 20'd0;
            temp_latch     <= 8'sd20;
            temp_abs_latch <= 8'd20;
            dist_shift     <= 19'd0;
            dist_bcd_work  <= 24'd0;
            dist_bcd       <= 24'd0;
            dist_bcd_cnt   <= 5'd0;
            temp_shift     <= 8'd0;
            temp_bcd_work  <= 12'd0;
            temp_bcd       <= 12'd0;
            temp_bcd_cnt   <= 4'd0;
            send_index     <= 5'd0;
            tx_data        <= 8'd0;
            tx_data_valid  <= 1'b0;
        end else begin
            case (state)
                // 等待 1 秒周期到达。
                // 到达后锁存 distance/temp_c，并启动十进制转换。
                S_WAIT: begin
                    tx_data_valid <= 1'b0;

                    if (clk_us) begin
                        if (interval_cnt == REPORT_INTERVAL_US) begin
                            interval_cnt   <= 20'd0;
                            temp_latch     <= temp_c;
                            temp_abs_latch <= temp_c[7] ? (~temp_c + 8'd1) : temp_c;
                            dist_shift     <= distance;
                            dist_bcd_work  <= 24'd0;
                            dist_bcd_cnt   <= 5'd0;
                            state          <= S_DIST_BCD;
                        end else begin
                            interval_cnt <= interval_cnt + 20'd1;
                        end
                    end
                end

                // 距离二进制转 BCD。
                // distance 为 19 位，所以转换 19 次。
                S_DIST_BCD: begin
                    dist_shift    <= {dist_shift[17:0], 1'b0};
                    dist_bcd_work <= {dist_bcd_adjust[22:0], dist_shift[18]};

                    if (dist_bcd_cnt == 5'd18) begin
                        dist_bcd      <= {dist_bcd_adjust[22:0], dist_shift[18]};
                        temp_shift    <= temp_abs_latch;
                        temp_bcd_work <= 12'd0;
                        temp_bcd_cnt  <= 4'd0;
                        state         <= S_TEMP_BCD;
                    end else begin
                        dist_bcd_cnt <= dist_bcd_cnt + 5'd1;
                    end
                end

                // 温度绝对值转 BCD。
                // 温度绝对值为 8 位，所以转换 8 次。
                S_TEMP_BCD: begin
                    temp_shift    <= {temp_shift[6:0], 1'b0};
                    temp_bcd_work <= {temp_bcd_adjust[10:0], temp_shift[7]};

                    if (temp_bcd_cnt == 4'd7) begin
                        temp_bcd   <= {temp_bcd_adjust[10:0], temp_shift[7]};
                        send_index <= 5'd0;
                        state      <= S_SEND;
                    end else begin
                        temp_bcd_cnt <= temp_bcd_cnt + 4'd1;
                    end
                end

                // 逐字节发送报文。
                // tx_data_valid 拉高后等待 uart_tx 的 ready，再推进到下一个字符。
                S_SEND: begin
                    if (!tx_data_valid) begin
                        tx_data       <= report_char;
                        tx_data_valid <= 1'b1;
                    end else if (tx_data_ready) begin
                        tx_data_valid <= 1'b0;
                        if (send_index == LAST_BYTE) begin
                            state      <= S_WAIT;
                            send_index <= 5'd0;
                        end else begin
                            send_index <= send_index + 5'd1;
                        end
                    end
                end

                default:
                    state <= S_WAIT;
            endcase
        end
    end

    // 单个 BCD 数字转 ASCII 字符。
    function [7:0] bcd_ascii;
        input [3:0] value;
        begin
            bcd_ascii = 8'd48 + value;
        end
    endfunction

    // 根据 send_index 组合出当前要发送的报文字符。
    // 报文例子：D=123.456cm,T=+020C\r\n
    always @(*) begin
        case (send_index)
            5'd0:  report_char = "D";
            5'd1:  report_char = "=";
            5'd2:  report_char = bcd_ascii(dist_bcd[23:20]);
            5'd3:  report_char = bcd_ascii(dist_bcd[19:16]);
            5'd4:  report_char = bcd_ascii(dist_bcd[15:12]);
            5'd5:  report_char = ".";
            5'd6:  report_char = bcd_ascii(dist_bcd[11:8]);
            5'd7:  report_char = bcd_ascii(dist_bcd[7:4]);
            5'd8:  report_char = bcd_ascii(dist_bcd[3:0]);
            5'd9:  report_char = "c";
            5'd10: report_char = "m";
            5'd11: report_char = ",";
            5'd12: report_char = "T";
            5'd13: report_char = "=";
            5'd14: report_char = temp_latch[7] ? "-" : "+";
            5'd15: report_char = bcd_ascii(temp_bcd[11:8]);
            5'd16: report_char = bcd_ascii(temp_bcd[7:4]);
            5'd17: report_char = bcd_ascii(temp_bcd[3:0]);
            5'd18: report_char = "C";
            5'd19: report_char = 8'h0d;
            5'd20: report_char = 8'h0a;
            default: report_char = 8'h20;
        endcase
    end

    // 实际 UART 发送器。
    uart_tx #(
        .CLK_FRE   (50),
        .BAUD_RATE (115200)
    ) u_uart_tx (
        .clk           (clk),
        .rstn          (rstn),
        .tx_data       (tx_data),
        .tx_data_valid (tx_data_valid),
        .tx_data_ready (tx_data_ready),
        .tx_pin        (uart_tx)
    );

endmodule
