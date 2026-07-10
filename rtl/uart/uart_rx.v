//============================================================
// 模块名称：uart_rx
// 功能说明：
//   UART 串口接收器，接收格式为 8N1：
//     1 个起始位；
//     8 个数据位，低位先收；
//     1 个停止位；
//     无校验位。
//
// 当前工程用途：
//   RX 已经接入顶层并可接收数据；
//   目前只作为预留接口，接收到的字节由顶层锁存，暂不参与控制。
//============================================================
module uart_rx
#(
    parameter CLK_FRE = 50,       // 输入时钟频率，单位 MHz
    parameter BAUD_RATE = 115200  // 串口波特率
)
(
    input        clk,             // 系统时钟
    input        rstn,            // 异步复位，低有效
    output reg [7:0] rx_data,     // 接收到的 8 位数据
    output reg   rx_data_valid,   // 接收完成标志
    input        rx_data_ready,   // 上层准备好接收数据
    input        rx_pin           // UART RX 引脚
);

    // 一个 UART bit 对应的系统时钟周期数。
    localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;

    // 接收状态机。
    localparam S_IDLE     = 3'd1, // 等待起始位下降沿
               S_START    = 3'd2, // 起始位确认
               S_REC_BYTE = 3'd3, // 接收 8 个数据位
               S_STOP     = 3'd4, // 接收停止位
               S_DATA     = 3'd5; // 等待上层取走数据

    reg [2:0]  state;
    reg [2:0]  next_state;
    reg        rx_d0;        // 第一级同步/延迟
    reg        rx_d1;        // 第二级同步/延迟
    reg [7:0]  rx_bits;      // 接收过程中的临时数据
    reg [15:0] cycle_cnt;    // 一个 bit 内部的时钟计数
    reg [2:0]  bit_cnt;      // 当前接收到第几个数据位

    wire rx_negedge;         // 起始位下降沿检测

    // UART 空闲为高电平，检测从 1 到 0 的变化作为一帧开始。
    assign rx_negedge = rx_d1 && !rx_d0;

    // RX 输入同步。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rx_d0 <= 1'b1;
            rx_d1 <= 1'b1;
        end else begin
            rx_d0 <= rx_pin;
            rx_d1 <= rx_d0;
        end
    end

    // 状态寄存器。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // 下一状态组合逻辑。
    // 数据位在 bit 中点采样，状态切换仍按完整 bit 周期推进。
    always @(*) begin
        case (state)
            S_IDLE:
                next_state = rx_negedge ? S_START : S_IDLE;
            S_START:
                next_state = (cycle_cnt == CYCLE - 1) ? S_REC_BYTE : S_START;
            S_REC_BYTE:
                next_state = (cycle_cnt == CYCLE - 1 && bit_cnt == 3'd7) ? S_STOP : S_REC_BYTE;
            S_STOP:
                next_state = (cycle_cnt == CYCLE / 2 - 1) ? S_DATA : S_STOP;
            S_DATA:
                next_state = rx_data_ready ? S_IDLE : S_DATA;
            default:
                next_state = S_IDLE;
        endcase
    end

    // 接收完成后拉高 rx_data_valid；
    // 当上层 rx_data_ready=1 时清除 valid 并准备接收下一帧。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            rx_data_valid <= 1'b0;
        else if (state == S_STOP && next_state != state)
            rx_data_valid <= 1'b1;
        else if (state == S_DATA && rx_data_ready)
            rx_data_valid <= 1'b0;
    end

    // 停止位阶段结束时，把临时接收值输出到 rx_data。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            rx_data <= 8'd0;
        else if (state == S_STOP && next_state != state)
            rx_data <= rx_bits;
    end

    // 数据位计数，低位先收。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            bit_cnt <= 3'd0;
        else if (state == S_REC_BYTE) begin
            if (cycle_cnt == CYCLE - 1)
                bit_cnt <= bit_cnt + 3'd1;
        end else begin
            bit_cnt <= 3'd0;
        end
    end

    // 波特率计数器。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            cycle_cnt <= 16'd0;
        else if ((state == S_REC_BYTE && cycle_cnt == CYCLE - 1) || next_state != state)
            cycle_cnt <= 16'd0;
        else
            cycle_cnt <= cycle_cnt + 16'd1;
    end

    // 在每个数据 bit 的中点采样，可以最大限度避开边沿抖动。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            rx_bits <= 8'd0;
        else if (state == S_REC_BYTE && cycle_cnt == CYCLE / 2 - 1)
            rx_bits[bit_cnt] <= rx_pin;
    end

endmodule
