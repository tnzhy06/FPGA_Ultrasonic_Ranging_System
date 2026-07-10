//============================================================
// 模块名称：lcd_wave_display
// 功能说明：
//   产生 AN430 480x272 RGB LCD 的 9MHz 显示时序，并在屏幕上绘制
//   实时距离波形。波形以 10Hz 更新，保存 110 个样点，对应约 11 秒
//   的滑动时间窗；纵轴显示范围为 0~300cm。
//
// 显示效果：
//   - 黑色背景；
//   - 尽量占满屏幕高度的网格与波形区域；
//   - 亮绿色曲线表示距离随时间的变化。
//   - 横轴每个网格代表 1 秒，纵轴每个网格代表 50cm。
//============================================================
module lcd_wave_display (
    input  wire        clk,                 // 9MHz LCD 像素时钟
    input  wire        rstn,                // 低有效异步复位
    input  wire [18:0] sample_data_async,   // 来自 50MHz 域的稳定距离样点
    input  wire        sample_toggle_async, // 来自 50MHz 域的新样点翻转标志
    output wire        lcd_hs,              // 行同步，低有效
    output wire        lcd_vs,              // 场同步，低有效
    output wire        lcd_de,              // RGB 数据有效，高有效
    output reg  [7:0]  lcd_r,               // 红色分量
    output reg  [7:0]  lcd_g,               // 绿色分量
    output reg  [7:0]  lcd_b                // 蓝色分量
);

    //========================================================
    // AN430 显示时序参数
    //
    // 一行先经历 2 个像素前肩、41 个像素同步、2 个像素后肩，随后输出
    // 480 个有效像素；一帧的垂直时序与此相同。总像素数为 525×286，
    // 在 9MHz 像素时钟下刷新率约为 59.94Hz。
    //========================================================
    localparam [9:0] H_ACTIVE = 10'd480;
    localparam [9:0] H_FP     = 10'd2;
    localparam [9:0] H_SYNC   = 10'd41;
    localparam [9:0] H_BP     = 10'd2;
    localparam [9:0] H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;

    localparam [8:0] V_ACTIVE = 9'd272;
    localparam [8:0] V_FP     = 9'd2;
    localparam [8:0] V_SYNC   = 9'd10;
    localparam [8:0] V_BP     = 9'd2;
    localparam [8:0] V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;

    // 波形区宽 440 像素，每个样点占 4 像素，共显示 110 个历史样点。
    // 移除标题栏后，图表纵向范围扩展为 y=8~239，实际高度为 231 像素。
    localparam [7:0] SAMPLE_COUNT = 8'd110;
    localparam [9:0] GRAPH_X0     = 10'd20;
    localparam [9:0] GRAPH_X1     = 10'd459;
    localparam [8:0] GRAPH_Y0     = 9'd8;
    localparam [8:0] GRAPH_Y1     = 9'd239;

    reg [9:0] h_cnt; // 当前一行内的像素位置，范围 0~524
    reg [8:0] v_cnt; // 当前一帧内的行位置，范围 0~285

    wire video_active; // 当前计数位置是否处于 480x272 有效显示区
    wire hs_raw;       // 未延迟的行同步
    wire vs_raw;       // 未延迟的场同步
    wire [9:0] pixel_x;// 有效显示区内的横坐标
    wire [8:0] pixel_y;// 有效显示区内的纵坐标
    reg        lcd_hs_d1;  // 与 RGB 数据对齐的一拍行同步
    reg        lcd_vs_d1;  // 与 RGB 数据对齐的一拍场同步
    reg        lcd_de_d1;  // 与 RGB 数据对齐的一拍数据有效
    reg [9:0]  pixel_x_d1; // 与历史样点读出对齐的一拍横坐标
    reg [8:0]  pixel_y_d1; // 与历史样点读出对齐的一拍纵坐标

    // h_cnt/v_cnt 统计的是包含消隐区在内的完整时序位置。只有同时越过
    // 水平与垂直后肩、且尚未到达总长度时，当前时钟才输出有效 RGB。
    // pixel_x/pixel_y 则把有效区左上角重新映射为坐标原点 (0,0)。
    assign video_active = (h_cnt >= (H_FP + H_SYNC + H_BP)) &&
                          (h_cnt < H_TOTAL) &&
                          (v_cnt >= (V_FP + V_SYNC + V_BP)) &&
                          (v_cnt < V_TOTAL);
    assign pixel_x = h_cnt - (H_FP + H_SYNC + H_BP);
    assign pixel_y = v_cnt - (V_FP + V_SYNC + V_BP);

    // AN430 面板采用低有效 HS/VS、高有效 DE。RGB 波形数据包含一拍
    // RAM 读延迟，因此同步信号与有效坐标也统一延迟一拍。
    assign hs_raw = !((h_cnt >= H_FP) && (h_cnt < (H_FP + H_SYNC)));
    assign vs_raw = !((v_cnt >= V_FP) && (v_cnt < (V_FP + V_SYNC)));
    assign lcd_hs = lcd_hs_d1;
    assign lcd_vs = lcd_vs_d1;
    assign lcd_de = lcd_de_d1;

    //========================================================
    // 行、场扫描计数器
    //
    // h_cnt 每个像素时钟加 1；行末回到 0 时才推进 v_cnt。这样一帧结束
    // 后两个计数器都会回零，并从下一帧左上角重新开始扫描。
    //========================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            h_cnt <= 10'd0;
            v_cnt <= 9'd0;
        end else begin
            if (h_cnt == H_TOTAL - 10'd1) begin
                h_cnt <= 10'd0;
                if (v_cnt == V_TOTAL - 9'd1)
                    v_cnt <= 9'd0;
                else
                    v_cnt <= v_cnt + 9'd1;
            end else begin
                h_cnt <= h_cnt + 10'd1;
            end
        end
    end

    // 延迟视频时序一个像素周期，使 HS、VS、DE、像素坐标与下方寄存器化
    // 的历史样点读取结果严格对齐，避免波形相对边框横向偏移。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            lcd_hs_d1  <= 1'b1;
            lcd_vs_d1  <= 1'b1;
            lcd_de_d1  <= 1'b0;
            pixel_x_d1 <= 10'd0;
            pixel_y_d1 <= 9'd0;
        end else begin
            lcd_hs_d1  <= hs_raw;
            lcd_vs_d1  <= vs_raw;
            lcd_de_d1  <= video_active;
            pixel_x_d1 <= pixel_x;
            pixel_y_d1 <= pixel_y;
        end
    end

    //========================================================
    // 距离样点跨时钟接收与环形历史缓冲
    //
    // 环形历史缓冲可避免每次采样时移动全部 110 个寄存器，并会被 Quartus
    // 推断为片上 RAM。distance[18:10] 约等于厘米值，先限制到 300cm，
    // 再缩放到 231 像素的图表高度。
    reg [7:0] history [0:SAMPLE_COUNT-1]; // 已缩放的波形高度值
    reg [7:0] write_ptr;                  // 下一次写入位置；也表示最旧样点位置
    reg [7:0] valid_count;                // 已写入有效样点数，最大 SAMPLE_COUNT
    reg       sample_toggle_meta;         // 跨时钟翻转标志的第一级同步寄存器
    reg       sample_toggle_sync;         // 跨时钟翻转标志的第二级同步寄存器
    reg       sample_toggle_seen;         // 已处理的翻转值，防止重复写入同一样点
    reg       capture_pending;            // 标志变化后延迟一拍再读取稳定数据总线
    reg [18:0] sample_capture;            // LCD 域锁存的原始距离样点
    reg [7:0] history_value_d1;           // 片上 RAM 读出的样点，延迟一拍
    reg       history_valid_d1;           // 与 history_value_d1 对齐的有效标志
    wire [16:0] sample_scale_product;     // 距离缩放时的定点乘法结果
    wire [8:0]  sample_plot_height;       // 换算后的 0~231 像素高度
    wire [9:0] plot_column_wide;          // 当前像素对应的样点列号（未截断）
    wire [7:0] plot_column;               // 当前像素对应的 0~109 样点列号
    wire [8:0] history_sum;               // 环形地址相加的中间结果
    wire [8:0] history_wrapped;           // 超过缓冲区尾部后回绕的地址
    wire [7:0] history_addr;              // 当前像素需要读取的历史样点地址
    wire       history_valid;             // 缓冲区相应位置是否已经写入有效数据

    // 用 198/256 近似 231/300：300cm 对应新的 231 像素图表高度。
    // 例如距离 150cm 时高度约为 150×198/256=116 像素，显示位置为
    // GRAPH_Y1-116。取乘积高 9 位相当于右移 8 位，避免综合出硬件除法器。
    assign sample_scale_product = sample_capture[18:10] * 8'd198;
    assign sample_plot_height   = sample_scale_product[16:8];

    // 以下 always 同时完成三项工作：
    //   1. 两级同步 sample_toggle_async，检测 50MHz 域是否有新样点；
    //   2. 把稳定的距离总线缩放后写入环形 RAM，并推进写指针；
    //   3. 对当前显示像素发起同步 RAM 读，为下一个像素周期准备数据。
    // valid_count 在上电后的前 110 个样点内递增；未填满的左侧区域
    // 不读取未初始化 RAM，而是按 0cm 基线显示。
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            write_ptr          <= 8'd0;
            valid_count        <= 8'd0;
            sample_toggle_meta <= 1'b0;
            sample_toggle_sync <= 1'b0;
            sample_toggle_seen <= 1'b0;
            capture_pending    <= 1'b0;
            sample_capture     <= 19'd0;
            history_value_d1   <= 8'd0;
            history_valid_d1   <= 1'b0;
        end else begin
            // 桥接模块会将样点总线保持 100ms。翻转标志经过两级同步后，
            // 再额外等待一拍才读取 19 位总线，保证总线已稳定。
            sample_toggle_meta <= sample_toggle_async;
            sample_toggle_sync <= sample_toggle_meta;

            // 同步后的翻转值与已处理值不同，说明出现一个新的 100ms 样点。
            // 这里仅锁存原始数据；下一拍才执行缩放和写 RAM。
            if (sample_toggle_sync != sample_toggle_seen) begin
                sample_toggle_seen <= sample_toggle_sync;
                sample_capture     <= sample_data_async;
                capture_pending    <= 1'b1;
            end else if (capture_pending) begin
                // 超过量程时固定绘制在顶端；缩放乘法的近似误差若使高度
                // 超过 231 像素，也同样钳位，防止波形越过图表边框。
                if (sample_capture[18:10] > 9'd300)
                    history[write_ptr] <= 8'd231;
                else if (sample_plot_height > 9'd231)
                    history[write_ptr] <= 8'd231;
                else
                    history[write_ptr] <= sample_plot_height[7:0];

                // 写到最后一个单元后回到 0，后续数据会覆盖最旧样点。
                // 因而 write_ptr 在写入完成后始终指向当前最旧样点。
                if (write_ptr == SAMPLE_COUNT - 8'd1)
                    write_ptr <= 8'd0;
                else
                    write_ptr <= write_ptr + 8'd1;

                // 缓冲区填满后 valid_count 保持 110，环形覆盖不再改变它。
                if (valid_count < SAMPLE_COUNT)
                    valid_count <= valid_count + 8'd1;

                capture_pending <= 1'b0;
            end

            // 对片上 RAM 的读取寄存器化：把可变地址访问和 RGB 颜色判断
            // 分到两个像素周期，缩短 9MHz 像素时钟的关键组合路径。
            history_value_d1 <= history[history_addr];
            history_valid_d1 <= history_valid;
        end
    end

    //========================================================
    // 像素坐标到历史样点地址的映射
    //
    // 横向每 4 个像素代表一个样点。write_ptr 始终指向最旧样点，
    // 因而图表左侧显示最早数据，右侧显示最新距离。
    //
    // history_addr = (write_ptr + plot_column) mod SAMPLE_COUNT。
    // 当缓冲区尚未写满时，只有右侧 valid_count 个样点有效；左侧区域
    // 用 0cm 基线补齐。缓冲区写满后，110 个列号全部映射为有效历史值。
    //========================================================
    wire in_graph;
    wire [7:0] history_value;
    wire [8:0] waveform_y;
    wire graph_border;
    wire graph_grid;
    wire waveform_pixel;

    // in_graph 限定当前像素位于图表内部；图表外保持黑色背景。
    assign in_graph = lcd_de_d1 && (pixel_x_d1 >= GRAPH_X0) &&
                      (pixel_x_d1 <= GRAPH_X1) && (pixel_y_d1 >= GRAPH_Y0) &&
                      (pixel_y_d1 <= GRAPH_Y1);
    // 右移 2 位相当于除以 4，得到 0~109 的样点列号。图表外强制为 0，
    // 避免坐标相减下溢时形成越界的 RAM 地址。
    assign plot_column_wide = ((pixel_x >= GRAPH_X0) && (pixel_x <= GRAPH_X1)) ?
                               ((pixel_x - GRAPH_X0) >> 2) : 10'd0;
    assign plot_column  = plot_column_wide[7:0];
    // 额外保留 1 位用于检测 write_ptr+plot_column 是否越过缓冲区末尾；
    // 一旦越界，history_wrapped 减去 SAMPLE_COUNT 得到回绕地址。
    assign history_sum  = {1'b0, write_ptr} + {1'b0, plot_column};
    assign history_wrapped = history_sum - {1'b0, SAMPLE_COUNT};
    assign history_addr = (history_sum >= {1'b0, SAMPLE_COUNT}) ?
                          history_wrapped[7:0] : history_sum[7:0];
    assign history_valid = (valid_count == SAMPLE_COUNT) ||
                           (plot_column >= SAMPLE_COUNT - valid_count);
    // RAM 输出与像素坐标相差一拍，故用 history_valid_d1 共同判定。
    // 无效样点输出 0，使波形在刚上电时从底部逐步向左扩展。
    assign history_value = history_valid_d1 ? history_value_d1 : 8'd0;
    assign waveform_y    = GRAPH_Y1 - {1'b0, history_value};

    // 图表四边绘制亮蓝色边框。内部网格只绘制 1~10 秒和 50~250cm，
    // 0 秒/11 秒与 0cm/300cm 由边框本身表示，不再重复画线。
    assign graph_border = in_graph && ((pixel_x_d1 == GRAPH_X0) ||
                                       (pixel_x_d1 == GRAPH_X1) ||
                                       (pixel_y_d1 == GRAPH_Y0) ||
                                       (pixel_y_d1 == GRAPH_Y1));
    // 横轴：10Hz 采样且每个样点占 4 像素，因此 40 像素正好为 1 秒；
    // 纵轴：根据 0~300cm 到 231 像素的缩放结果，依次标出 50、100、
    // 150、200、250cm。0cm 与 300cm 由底部和顶部边框表示。
    assign graph_grid = in_graph && ((pixel_x_d1 == 10'd60)  || (pixel_x_d1 == 10'd100) ||
                                     (pixel_x_d1 == 10'd140) || (pixel_x_d1 == 10'd180) ||
                                     (pixel_x_d1 == 10'd220) || (pixel_x_d1 == 10'd260) ||
                                     (pixel_x_d1 == 10'd300) || (pixel_x_d1 == 10'd340) ||
                                     (pixel_x_d1 == 10'd380) || (pixel_x_d1 == 10'd420) ||
                                     (pixel_y_d1 == 9'd201)  || (pixel_y_d1 == 9'd162) ||
                                     (pixel_y_d1 == 9'd123)  || (pixel_y_d1 == 9'd85)  ||
                                     (pixel_y_d1 == 9'd46));
    // 以 waveform_y 为中心绘制 3 像素厚的绿色轨迹。使用相邻像素加粗
    // 可避免单像素曲线在 LCD 上显得过细，同时不需要额外的线段插值逻辑。
    assign waveform_pixel = in_graph && (pixel_y_d1 >= waveform_y - 9'd1) &&
                            (pixel_y_d1 <= waveform_y + 9'd1);

    //========================================================
    // RGB 颜色生成
    //
    // 像素颜色优先级：黑色背景 < 网格 < 边框 < 绿色波形。
    // 使用连续的 if 覆盖赋值可直接表达层叠关系：波形与网格重合时，
    // 波形保持绿色；与边框重合时，也优先显示绿色样点。
    //========================================================
    always @(*) begin
        lcd_r = 8'h00;
        lcd_g = 8'h00;
        lcd_b = 8'h00;

        if (lcd_de_d1) begin
            if (graph_grid) begin
                lcd_r = 8'h10;
                lcd_g = 8'h28;
                lcd_b = 8'h40;
            end

            if (graph_border) begin
                lcd_r = 8'h50;
                lcd_g = 8'h80;
                lcd_b = 8'ha0;
            end

            if (waveform_pixel) begin
                lcd_r = 8'h00;
                lcd_g = 8'hff;
                lcd_b = 8'h70;
            end
        end
    end

endmodule
