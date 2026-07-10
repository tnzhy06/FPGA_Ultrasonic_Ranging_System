//============================================================
// 模块名称：trig_driver
// 功能说明：
//   产生 HC-SR04 超声波模块的 TRIG 触发信号。
//
// HC-SR04 使用方式：
//   TRIG 拉高至少 10us 后，模块会发射一组超声波；
//   本工程输出 15us 高脉冲，留出一定余量；
//   每 100ms 触发一次，避免回波未结束就开始下一次测量。
//
// 时基说明：
//   本模块不直接分频时钟，只在 clk_us=1 时更新微秒计数。
//============================================================
module trig_driver (
    input  wire clk,     // 50MHz 系统时钟
    input  wire clk_us,  // 1us 使能脉冲，来自 clk_div
    input  wire rstn,    // 异步复位，低有效

    output wire trig     // HC-SR04 TRIG 输出，高电平触发测距
);

    // 100ms = 100000us，计数 0~99999。
    parameter CYCLE_MAX = 19'd99_999;

    // 微秒计数器：
    //   只在 clk_us 有效时加 1；
    //   计满 100ms 后回到 0；
    //   计数前 15us 输出 TRIG 高电平。
    reg [18:0] cnt;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt <= 19'd0;
        end else if (clk_us) begin
            if (cnt == CYCLE_MAX) begin
                cnt <= 19'd0;
            end else begin
                cnt <= cnt + 19'd1;
            end
        end
    end

    // cnt 为 0~14 时输出高电平，共 15us。
    assign trig = (cnt < 19'd15) ? 1'b1 : 1'b0;

endmodule
