//============================================================
// 模块名称：clk_div
// 功能说明：
//   将 AX301 板载 50MHz 时钟转换为 1us 使能脉冲。
//
// 设计要点：
//   1. clk_us 不是新的慢时钟，而是一个单周期使能脉冲；
//   2. 所有下游模块仍然使用 50MHz clk 作为时钟；
//   3. 这样可以避免派生时钟带来的跨时钟域和时序约束问题。
//
// 时序关系：
//   50MHz 时钟周期为 20ns，50 个周期为 1us。
//   cnt 从 0 计到 49，共 50 个 clk 周期。
//============================================================
module clk_div (
    input  wire clk,    // 50MHz 系统时钟
    input  wire rstn,   // 异步复位，低有效
    output wire clk_us  // 1us 使能脉冲，高电平宽度为 1 个 clk 周期
);

    // 50MHz 下 1us 需要 50 个周期，因此最大计数值为 49。
    parameter CNT_MAX = 19'd49;

    // 计数范围 0~49，6 位足够容纳。
    reg [5:0] cnt;

    // 计数器循环计数：
    //   rstn=0 时清零；
    //   计到 CNT_MAX 后回到 0；
    //   否则每个 50MHz 时钟加 1。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt <= 6'd0;
        end else if (cnt == CNT_MAX) begin
            cnt <= 6'd0;
        end else begin
            cnt <= cnt + 6'd1;
        end
    end

    // 当计数器等于 CNT_MAX 时输出高电平。
    // 由于下一拍 cnt 会清零，所以 clk_us 只持续一个 clk 周期。
    assign clk_us = (cnt == CNT_MAX);

endmodule
