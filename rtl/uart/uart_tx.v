//============================================================
// 模块名称：uart_tx
// 功能说明：
//   UART 串口发送器，发送格式为常见的 8N1：
//     1 个起始位 0；
//     8 个数据位，低位先发；
//     1 个停止位 1；
//     无校验位。
//
// 握手接口：
//   tx_data_valid=1 表示上层有 1 字节要发送；
//   tx_data_ready=1 表示本模块空闲，可以接收新字节；
//   当二者同时有效时，tx_data 会被锁存并开始发送。
//============================================================
module uart_tx
#(
    parameter CLK_FRE = 50,       // 输入时钟频率，单位 MHz
    parameter BAUD_RATE = 115200  // 串口波特率
)
(
    input        clk,             // 系统时钟
    input        rstn,            // 异步复位，低有效
    input  [7:0] tx_data,         // 待发送的 8 位数据
    input        tx_data_valid,   // 待发送数据有效
    output reg   tx_data_ready,   // 发送器空闲/可接收新数据
    output       tx_pin           // UART TX 引脚
);

    // 每个串口 bit 对应多少个系统时钟周期。
    // 50MHz / 115200 ≈ 434，即每 434 个 clk 输出一个串口 bit。
    localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;

    // 发送状态机。
    localparam S_IDLE      = 3'd1, // 空闲，TX 保持高电平
               S_START     = 3'd2, // 发送起始位 0
               S_SEND_BYTE = 3'd3, // 发送 8 个数据位
               S_STOP      = 3'd4; // 发送停止位 1

    reg [2:0]  state;
    reg [2:0]  next_state;
    reg [15:0] cycle_cnt;      // 一个 bit 内部的时钟计数
    reg [2:0]  bit_cnt;        // 当前发送到第几个数据位
    reg [7:0]  tx_data_latch;  // 锁存待发送数据，防止上层数据变化影响发送
    reg        tx_reg;         // 串口输出寄存器

    assign tx_pin = tx_reg;

    // 状态寄存器。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // 下一状态组合逻辑。
    always @(*) begin
        case (state)
            S_IDLE:
                next_state = tx_data_valid ? S_START : S_IDLE;
            S_START:
                next_state = (cycle_cnt == CYCLE - 1) ? S_SEND_BYTE : S_START;
            S_SEND_BYTE:
                next_state = (cycle_cnt == CYCLE - 1 && bit_cnt == 3'd7) ? S_STOP : S_SEND_BYTE;
            S_STOP:
                next_state = (cycle_cnt == CYCLE - 1) ? S_IDLE : S_STOP;
            default:
                next_state = S_IDLE;
        endcase
    end

    // ready 信号：
    //   空闲且没有新 valid 时为 1；
    //   一旦开始发送就拉低；
    //   停止位发送完成后重新拉高。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            tx_data_ready <= 1'b0;
        else if (state == S_IDLE)
            tx_data_ready <= ~tx_data_valid;
        else if (state == S_STOP && cycle_cnt == CYCLE - 1)
            tx_data_ready <= 1'b1;
    end

    // 在空闲状态接收到 valid 时锁存数据。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            tx_data_latch <= 8'd0;
        else if (state == S_IDLE && tx_data_valid)
            tx_data_latch <= tx_data;
    end

    // 数据位计数，范围 0~7。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            bit_cnt <= 3'd0;
        else if (state == S_SEND_BYTE) begin
            if (cycle_cnt == CYCLE - 1)
                bit_cnt <= bit_cnt + 3'd1;
        end else begin
            bit_cnt <= 3'd0;
        end
    end

    // 波特率计数器。
    // 状态切换时清零，保证每个 start/data/stop bit 都从完整 bit 周期开始。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            cycle_cnt <= 16'd0;
        else if ((state == S_SEND_BYTE && cycle_cnt == CYCLE - 1) || next_state != state)
            cycle_cnt <= 16'd0;
        else
            cycle_cnt <= cycle_cnt + 16'd1;
    end

    // 串口输出：
    //   空闲和停止位为 1；
    //   起始位为 0；
    //   数据位按 tx_data_latch[0] 到 tx_data_latch[7] 依次发送。
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            tx_reg <= 1'b1;
        else begin
            case (state)
                S_IDLE,
                S_STOP:
                    tx_reg <= 1'b1;
                S_START:
                    tx_reg <= 1'b0;
                S_SEND_BYTE:
                    tx_reg <= tx_data_latch[bit_cnt];
                default:
                    tx_reg <= 1'b1;
            endcase
        end
    end

endmodule
