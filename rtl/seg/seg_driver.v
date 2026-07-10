//============================================================
// 模块名称：seg_driver
// 功能说明：
//   6 位共阳极数码管动态扫描驱动。
//
// 显示格式：
//   data_in 单位为 0.001cm；
//   显示为 xxx.xxx，例如 data_in=123456 时显示 123.456。
//
// 资源优化：
//   不使用 /10、%10 这类组合除法；
//   使用串行 Double-Dabble 算法将二进制转为 BCD；
//   每次转换只需 19 个 50MHz 周期，远小于显示刷新周期。
//
// 硬件说明：
//   AX301 板载 6 位共阳极数码管；
//   sel 低有效，选择当前点亮的位；
//   seg 低有效，seg[7] 为小数点 DP。
//============================================================
module seg_driver(
    input   wire        clk,     // 50MHz 系统时钟
    input   wire        rstn,    // 异步复位，低有效

    input   wire [18:0] data_in, // 距离数据，单位 0.001cm

    output  reg  [5:0]  sel,     // 数码管位选，低电平有效
    output  reg  [7:0]  seg      // 数码管段选，低电平有效，seg[7] 为小数点
);

    // 共阳极段码，位序为 {dp,g,f,e,d,c,b,a}。
    // 低电平点亮，所以数字 0 的 a~f 为 0，g 和 dp 为 1。
    localparam  NUM_0   = 8'b1100_0000,
                NUM_1   = 8'b1111_1001,
                NUM_2   = 8'b1010_0100,
                NUM_3   = 8'b1011_0000,
                NUM_4   = 8'b1001_1001,
                NUM_5   = 8'b1001_0010,
                NUM_6   = 8'b1000_0010,
                NUM_7   = 8'b1111_1000,
                NUM_8   = 8'b1000_0000,
                NUM_9   = 8'b1001_0000,
                LIT_OUT = 8'b1111_1111; // 全灭

    // 单个位保持时间：
    //   50MHz 下 8333 个周期约 166.66us；
    //   6 位完整扫描约 1ms；
    //   每位刷新率约 1kHz，人眼不会明显闪烁。
    localparam [13:0] SCAN_CNT_MAX = 14'd8_332;

    //========================================================
    // 二进制转 BCD：Double-Dabble 算法
    //
    // bcd_work 用 6 个 BCD 位保存十进制结果：
    //   [23:20] 百位 cm
    //   [19:16] 十位 cm
    //   [15:12] 个位 cm
    //   [11:8]  0.1cm
    //   [7:4]   0.01cm
    //   [3:0]   0.001cm
    //
    // 算法步骤：
    //   1. 对每个 BCD 位，如果 >=5 就加 3；
    //   2. 整体左移一位，并移入二进制最高位；
    //   3. 重复 19 次，得到 19 位输入对应的十进制 BCD。
    //========================================================
    reg  [18:0] bin_shift;   // 正在移位处理的二进制输入副本
    reg  [23:0] bcd_work;    // 转换过程中的 BCD 工作寄存器
    reg  [23:0] bcd_value;   // 转换完成后供显示扫描使用的稳定 BCD 值
    reg  [23:0] bcd_adjust;  // 加 3 修正后的组合结果
    reg  [4:0]  bcd_count;   // 已处理的 bit 数
    reg         bcd_busy;    // 1 表示正在转换

    // Double-Dabble 的“>=5 加 3”组合修正。
    always @(*) begin
        bcd_adjust = bcd_work;

        if (bcd_adjust[3:0] >= 4'd5)
            bcd_adjust[3:0] = bcd_adjust[3:0] + 4'd3;
        if (bcd_adjust[7:4] >= 4'd5)
            bcd_adjust[7:4] = bcd_adjust[7:4] + 4'd3;
        if (bcd_adjust[11:8] >= 4'd5)
            bcd_adjust[11:8] = bcd_adjust[11:8] + 4'd3;
        if (bcd_adjust[15:12] >= 4'd5)
            bcd_adjust[15:12] = bcd_adjust[15:12] + 4'd3;
        if (bcd_adjust[19:16] >= 4'd5)
            bcd_adjust[19:16] = bcd_adjust[19:16] + 4'd3;
        if (bcd_adjust[23:20] >= 4'd5)
            bcd_adjust[23:20] = bcd_adjust[23:20] + 4'd3;
    end

    // 串行 BCD 转换控制。
    // 每次转换结束后立即重新采样 data_in，因此显示延迟只有几十个 clk 周期。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            bin_shift <= 19'd0;
            bcd_work   <= 24'd0;
            bcd_value  <= 24'd0;
            bcd_count  <= 5'd0;
            bcd_busy   <= 1'b0;
        end else if (!bcd_busy) begin
            bin_shift <= data_in;
            bcd_work  <= 24'd0;
            bcd_count <= 5'd0;
            bcd_busy  <= 1'b1;
        end else begin
            bin_shift <= {bin_shift[17:0], 1'b0};
            bcd_work  <= {bcd_adjust[22:0], bin_shift[18]};

            if (bcd_count == 5'd18) begin
                bcd_value <= {bcd_adjust[22:0], bin_shift[18]};
                bcd_busy  <= 1'b0;
            end else begin
                bcd_count <= bcd_count + 5'd1;
            end
        end
    end

    //========================================================
    // 6 位动态扫描
    //
    // 扫描顺序从小数最低位到百位：
    //   scan_index=0 -> 0.001cm
    //   scan_index=1 -> 0.01cm
    //   scan_index=2 -> 0.1cm
    //   scan_index=3 -> 个位 cm，并点亮小数点
    //   scan_index=4 -> 十位 cm
    //   scan_index=5 -> 百位 cm
    //
    // 位选是低有效，每次只拉低一个 sel 位，避免多个数码管串亮。
    //========================================================
    reg [13:0] scan_cnt;
    reg [2:0]  scan_index;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            scan_cnt   <= 14'd0;
            scan_index <= 3'd0;
            sel        <= 6'b11_1111;
            seg        <= LIT_OUT;
        end else if (scan_cnt == SCAN_CNT_MAX) begin
            scan_cnt <= 14'd0;

            case (scan_index)
                3'd0: begin
                    sel <= 6'b11_1110;
                    seg <= hex_data(bcd_value[3:0]);
                end
                3'd1: begin
                    sel <= 6'b11_1101;
                    seg <= hex_data(bcd_value[7:4]);
                end
                3'd2: begin
                    sel <= 6'b11_1011;
                    seg <= hex_data(bcd_value[11:8]);
                end
                3'd3: begin
                    sel <= 6'b11_0111;
                    seg <= hex_data(bcd_value[15:12]) & 8'b0111_1111;
                end
                3'd4: begin
                    sel <= 6'b10_1111;
                    seg <= hex_data(bcd_value[19:16]);
                end
                3'd5: begin
                    sel <= 6'b01_1111;
                    seg <= hex_data(bcd_value[23:20]);
                end
                default: begin
                    sel <= 6'b11_1111;
                    seg <= LIT_OUT;
                end
            endcase

            if (scan_index == 3'd5)
                scan_index <= 3'd0;
            else
                scan_index <= scan_index + 3'd1;
        end else begin
            scan_cnt <= scan_cnt + 14'd1;
        end
    end

    // BCD 数字到共阳极段码的译码函数。
    function [7:0] hex_data;
        input [3:0] data_i;
        begin
            case (data_i)
                4'd0: hex_data = NUM_0;
                4'd1: hex_data = NUM_1;
                4'd2: hex_data = NUM_2;
                4'd3: hex_data = NUM_3;
                4'd4: hex_data = NUM_4;
                4'd5: hex_data = NUM_5;
                4'd6: hex_data = NUM_6;
                4'd7: hex_data = NUM_7;
                4'd8: hex_data = NUM_8;
                4'd9: hex_data = NUM_9;
                default: hex_data = LIT_OUT;
            endcase
        end
    endfunction

endmodule
