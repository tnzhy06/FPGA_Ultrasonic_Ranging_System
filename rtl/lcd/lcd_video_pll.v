//============================================================
// 模块名称：lcd_video_pll
// 功能说明：
//   使用 Cyclone IV E 的 ALTPLL，把 AX301 板载 50MHz 时钟转换为
//   AN430（480x272）RGB LCD 所需的 9MHz 像素时钟。
//
// 参数关系：
//   Fout = 50MHz × 9 / 50 = 9MHz。
//   输出时钟保持 50% 占空比，供 LCD 行、场扫描逻辑统一使用。
//============================================================
module lcd_video_pll (
    input  wire inclk0, // 50MHz 输入时钟
    output wire c0     // 9MHz 像素时钟输出
);

    // ALTPLL 的 inclk 端口为两位总线：inclk[0] 接实际时钟，
    // inclk[1] 未使用，固定为 0。
    wire [0:0] pll_in_unused = 1'b0;
    wire [1:0] pll_in = {pll_in_unused, inclk0};
    wire [4:0] pll_clk; // ALTPLL 最多可提供 5 路时钟，本工程仅使用 clk[0]

    // 将 PLL 的第 0 路输出作为 LCD 像素时钟。
    assign c0 = pll_clk[0];

    // ALTPLL 原语实例。未使用的动态重配置、时钟切换与扫描接口被固定：
    // clkena/extclkena/pllena 保持使能，areset 始终不触发复位；
    // 扫描与相位步进接口不参与运行，因此 PLL 始终以 NORMAL 模式稳定输出 c0。
    altpll altpll_component (
        .inclk               (pll_in),
        .clk                 (pll_clk),
        .areset              (1'b0),
        .clkena              ({6{1'b1}}),
        .clkswitch           (1'b0),
        .configupdate        (1'b0),
        .extclkena           ({4{1'b1}}),
        .fbin                (1'b1),
        .pfdena              (1'b1),
        .phasecounterselect  ({4{1'b1}}),
        .phasestep           (1'b1),
        .phaseupdown         (1'b1),
        .pllena              (1'b1),
        .scanaclr            (1'b0),
        .scanclk             (1'b0),
        .scanclkena          (1'b1),
        .scandata            (1'b0),
        .scanread            (1'b0),
        .scanwrite           (1'b0)
    );

    // PLL 静态参数：输入时钟周期 20ns（20000ps），倍频 9、分频 50。
    // 得到的 c0 周期约为 111.111ns，即频率为 9MHz；相移为 0，
    // 因而扫描模块直接以 c0 上升沿更新水平、垂直坐标。
    // port_inclk0 与 port_clk0 明确声明实际使用的输入和输出端口。
    defparam
        altpll_component.bandwidth_type          = "AUTO",
        altpll_component.clk0_divide_by          = 50,
        altpll_component.clk0_duty_cycle         = 50,
        altpll_component.clk0_multiply_by        = 9,
        altpll_component.clk0_phase_shift        = "0",
        altpll_component.compensate_clock        = "CLK0",
        altpll_component.inclk0_input_frequency  = 20000,
        altpll_component.intended_device_family  = "Cyclone IV E",
        altpll_component.lpm_hint                 = "CBX_MODULE_PREFIX=lcd_video_pll",
        altpll_component.lpm_type                 = "altpll",
        altpll_component.operation_mode           = "NORMAL",
        altpll_component.pll_type                 = "AUTO",
        altpll_component.port_inclk0              = "PORT_USED",
        altpll_component.port_clk0                = "PORT_USED",
        altpll_component.width_clock              = 5;

endmodule
