//============================================================
// 模块名称：ds18b20_ctrl
// 功能说明：
//   DS18B20 温度传感器单总线控制器。
//
// 通信流程：
//   1. 复位总线，等待 DS18B20 存在脉冲；
//   2. 发送 Skip ROM(0xCC) + Convert T(0x44)，启动温度转换；
//   3. 等待 750ms，保证 12 位温度转换完成；
//   4. 再次复位总线，等待存在脉冲；
//   5. 发送 Skip ROM(0xCC) + Read Scratchpad(0xBE)；
//   6. 读取前 16 位温度原始数据；
//   7. 转换为整数摄氏度 temp_c，并保持约 1 秒更新一次。
//
// 硬件连接：
//   dq 为开漏/三态单总线，FPGA 只主动拉低或释放总线；
//   外部需要 4.7k 左右电阻上拉到 3.3V，不能上拉到 5V。
//
// 输出说明：
//   temp_c      ：有符号整数温度，单位摄氏度；
//   temp_valid  ：读到一次有效温度后拉高并保持。
//============================================================
module ds18b20_ctrl(
    input  wire              clk,        // 50MHz 系统时钟
    input  wire              clk_us,     // 1us 使能脉冲，所有单总线时间都以它计数
    input  wire              rstn,       // 异步复位，低有效

    inout  wire              dq,         // DS18B20 单总线数据脚

    output reg  signed [7:0] temp_c,     // 当前温度，整数摄氏度
    output reg               temp_valid  // 温度有效标志
);

    // 上电或未读到传感器前使用 20℃ 作为默认温度。
    localparam signed [7:0] DEFAULT_TEMP_C = 8'sd20;

    // 状态机定义。
    // S_INIT_CONV ：第一次复位，准备发送温度转换命令；
    // S_WR_CONV   ：写 0xCC + 0x44，启动转换；
    // S_WAIT_CONV ：等待转换完成；
    // S_INIT_READ ：第二次复位，准备读取温度寄存器；
    // S_WR_READ   ：写 0xCC + 0xBE，发送读暂存器命令；
    // S_RD_TEMP   ：读取 16 位温度原始值；
    // S_INTERVAL  ：补足采样周期，使 temp_c 约每 1 秒更新一次。
    localparam [3:0] S_INIT_CONV  = 4'd0,
                     S_WR_CONV    = 4'd1,
                     S_WAIT_CONV  = 4'd2,
                     S_INIT_READ  = 4'd3,
                     S_WR_READ    = 4'd4,
                     S_RD_TEMP    = 4'd5,
                     S_INTERVAL   = 4'd6;

    // DS18B20 命令低位先发。
    // 16'h44cc 实际发送顺序为 0xCC、0x44；
    // 16'hbecc 实际发送顺序为 0xCC、0xBE。
    localparam [15:0] CMD_CONVERT = 16'h44cc;
    localparam [15:0] CMD_READ    = 16'hbecc;

    // 单总线关键时间参数，单位 us。
    localparam [19:0] RESET_US    = 20'd999;     // 复位完整窗口约 1000us
    localparam [19:0] RESET_LOW   = 20'd499;     // 主机拉低约 500us
    localparam [19:0] SAMPLE_PRES = 20'd570;     // 释放后采样存在脉冲
    localparam [19:0] SLOT_US     = 20'd64;      // 每个读/写时隙约 65us
    localparam [19:0] WAIT_US     = 20'd750_000; // 12 位转换最长约 750ms
    localparam [19:0] INTERVAL_US = 20'd244_879; // 补齐到约 1s 更新周期

    // dq_meta/dq_sync 用于把异步单总线输入同步到 clk 时钟域。
    reg         dq_meta;
    reg         dq_sync;

    // dq_drive_low=1 时 FPGA 主动把 DQ 拉低；
    // dq_drive_low=0 时 FPGA 输出高阻，由外部上拉电阻拉高。
    reg         dq_drive_low;

    reg  [3:0]  state;          // 当前状态
    reg  [19:0] cnt_us;         // 当前状态内的微秒计数
    reg  [3:0]  bit_cnt;        // 命令/温度数据 bit 序号，低位先处理
    reg  [15:0] temp_raw;       // DS18B20 原始温度数据，补码格式
    reg         presence_seen;  // 是否检测到存在脉冲

    wire [15:0] cmd_data;       // 当前需要发送的 16 位命令
    wire        write_zero;     // 当前命令 bit 是否为 0，写 0 需要长时间拉低
    wire signed [15:0] temp_raw_signed;
    wire signed [15:0] temp_integer;

    // 三态总线控制。
    // 注意这里没有主动输出 1，因为 DS18B20 单总线依靠上拉得到高电平。
    assign dq = dq_drive_low ? 1'b0 : 1'bz;

    assign cmd_data        = (state == S_WR_CONV) ? CMD_CONVERT : CMD_READ;
    assign write_zero      = ~cmd_data[bit_cnt];
    assign temp_raw_signed = temp_raw;

    // DS18B20 12 位温度格式低 4 位是小数部分。
    // 右移 4 位后得到整数摄氏度，负数自动做算术右移。
    assign temp_integer    = temp_raw_signed >>> 4;

    // DQ 输入同步。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            dq_meta <= 1'b1;
            dq_sync <= 1'b1;
        end else begin
            dq_meta <= dq;
            dq_sync <= dq_meta;
        end
    end

    // 主状态机。
    // 只有 clk_us 有效时才推进状态内部计数，因此 cnt_us 单位为 us。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state         <= S_INIT_CONV;
            cnt_us        <= 20'd0;
            bit_cnt       <= 4'd0;
            temp_raw      <= 16'd0;
            temp_c        <= DEFAULT_TEMP_C;
            temp_valid    <= 1'b0;
            presence_seen <= 1'b0;
        end else if (clk_us) begin
            case (state)
                // 复位阶段：
                //   cnt_us=0 时清除存在标志；
                //   SAMPLE_PRES 时刻如果 DQ 为低，说明 DS18B20 已响应；
                //   RESET_US 到达后，根据本次复位用途进入写转换命令或写读取命令。
                S_INIT_CONV,
                S_INIT_READ: begin
                    if (cnt_us == 20'd0)
                        presence_seen <= 1'b0;
                    else if (cnt_us == SAMPLE_PRES && !dq_sync)
                        presence_seen <= 1'b1;

                    if (cnt_us == RESET_US) begin
                        cnt_us <= 20'd0;
                        if (presence_seen) begin
                            state   <= (state == S_INIT_CONV) ? S_WR_CONV : S_WR_READ;
                            bit_cnt <= 4'd0;
                        end
                    end else begin
                        cnt_us <= cnt_us + 20'd1;
                    end
                end

                // 写命令阶段：
                //   每个 bit 使用一个约 65us 的时隙；
                //   写完 16 bit 后，转换命令进入等待，读取命令进入读温度。
                S_WR_CONV,
                S_WR_READ: begin
                    if (cnt_us == SLOT_US) begin
                        cnt_us <= 20'd0;
                        if (bit_cnt == 4'd15) begin
                            bit_cnt <= 4'd0;
                            state   <= (state == S_WR_CONV) ? S_WAIT_CONV : S_RD_TEMP;
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                        end
                    end else begin
                        cnt_us <= cnt_us + 20'd1;
                    end
                end

                // 温度转换等待阶段。
                // DS18B20 在 12 位分辨率下最大转换时间约 750ms。
                S_WAIT_CONV: begin
                    if (cnt_us == WAIT_US) begin
                        cnt_us <= 20'd0;
                        state  <= S_INIT_READ;
                    end else begin
                        cnt_us <= cnt_us + 20'd1;
                    end
                end

                // 读温度阶段：
                //   主机先拉低 1~2us 发起读时隙；
                //   在约 13us 处采样 DQ；
                //   DS18B20 低位先出，所以用 {dq_sync, temp_raw[15:1]} 移入。
                S_RD_TEMP: begin
                    if (cnt_us == 20'd13)
                        temp_raw <= {dq_sync, temp_raw[15:1]};

                    if (cnt_us == SLOT_US) begin
                        cnt_us <= 20'd0;
                        if (bit_cnt == 4'd15) begin
                            bit_cnt    <= 4'd0;
                            temp_c     <= temp_integer[7:0];
                            temp_valid <= 1'b1;
                            state      <= S_INTERVAL;
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                        end
                    end else begin
                        cnt_us <= cnt_us + 20'd1;
                    end
                end

                // 采样间隔阶段。
                // 前面转换和读取已经消耗约 755ms，这里再等待约 245ms，
                // 使 temp_c 的更新间隔约为 1 秒。
                S_INTERVAL: begin
                    if (cnt_us == INTERVAL_US) begin
                        cnt_us <= 20'd0;
                        state  <= S_INIT_CONV;
                    end else begin
                        cnt_us <= cnt_us + 20'd1;
                    end
                end

                default: begin
                    state  <= S_INIT_CONV;
                    cnt_us <= 20'd0;
                end
            endcase
        end
    end

    // 单总线输出时序组合逻辑。
    // 这里只决定“是否拉低”，真正的状态推进在上面的时序 always 中完成。
    always @(*) begin
        case (state)
            // 复位脉冲：前 RESET_LOW 微秒主动拉低，其余时间释放总线。
            S_INIT_CONV,
            S_INIT_READ:
                dq_drive_low = (cnt_us < RESET_LOW);

            // 写时隙：
            //   写 1：只在时隙开始短暂拉低，然后释放；
            //   写 0：拉低到约 62us。
            S_WR_CONV,
            S_WR_READ:
                dq_drive_low = (cnt_us <= 20'd1) || (write_zero && cnt_us <= 20'd62);

            // 读时隙：主机短暂拉低后释放，由 DS18B20 驱动数据位。
            S_RD_TEMP:
                dq_drive_low = (cnt_us <= 20'd1);

            default:
                dq_drive_low = 1'b0;
        endcase
    end

endmodule
