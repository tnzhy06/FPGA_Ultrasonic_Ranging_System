//============================================================
// 模块名称：wave_sample_bridge
// 功能说明：
//   在 50MHz 测距时钟域中每隔 100ms 锁存一次距离，并把样点交给
//   9MHz LCD 像素时钟域，用于绘制实时波形。
//
// 跨时钟处理方式：
//   sample_data 在两次采样之间保持不变；每产生一个新样点就翻转一次
//   sample_toggle。LCD 域同步该翻转标志后，再读取已稳定的数据总线，
//   因而不需要把 19 位距离数据逐位同步。与传递单周期脉冲相比，翻转
//   标志会一直保持到 LCD 域检测到电平变化，避免慢时钟域遗漏更新事件。
//============================================================
module wave_sample_bridge (
    input  wire        clk,           // 50MHz 系统时钟
    input  wire        clk_us,        // 1us 使能脉冲
    input  wire        rstn,          // 低有效异步复位
    input  wire [18:0] distance,      // 当前距离，单位 0.001cm
    output reg  [18:0] sample_data,   // 保持到下一次采样的波形样点
    output reg         sample_toggle  // 新样点事件的翻转标志
);

    // 100ms = 100000us。计数范围为 0~99999；计到 99999 的那个 clk_us
    // 同时生成样点，所以下一个采样周期重新从 0 开始累计。
    localparam [16:0] SAMPLE_INTERVAL_US = 17'd99_999;
    reg [16:0] interval_cnt; // 以 clk_us 为单位的采样周期计数器

    // 仅在 clk_us 到达时推进计数，clk 本身仍是本模块唯一的时钟，
    // clk_us 仅作为同步时钟使能使用。到达周期末尾时，将当前距离锁存
    // 并翻转事件标志，通知 LCD 域读取新样点。
    //
    // sample_data 与 sample_toggle 在同一个时钟沿更新。LCD 域检测到
    // toggle 的变化时，数据总线已经保持了多个 LCD 像素时钟周期，
    // 再由 lcd_wave_display 把该样点送入历史缓存。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            // 复位时清空第一个样点并把事件标志置为 0，避免 LCD 刚退出
            // 复位便把不确定的旧数据误当作一次有效更新。
            interval_cnt  <= 17'd0;
            sample_data   <= 19'd0;
            sample_toggle <= 1'b0;
        end else if (clk_us) begin
            if (interval_cnt == SAMPLE_INTERVAL_US) begin
                interval_cnt  <= 17'd0;
                sample_data   <= distance;
                sample_toggle <= ~sample_toggle;
            end else begin
                interval_cnt <= interval_cnt + 17'd1;
            end
        end
    end

endmodule
