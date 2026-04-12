// -----------------------------------------------------------------------------
// Single TDC channel
// -----------------------------------------------------------------------------
// 每个通道包含:
//   1) 延迟线
//   2) 前导位计数器/编码器
//   3) LUT 查表
//   4) 粗计数 + deskew 合成
//   5) 在线标定环振
// -----------------------------------------------------------------------------
module tdc_channel #(
    parameter integer g_CARRY4_COUNT = 1,
    parameter integer g_RAW_COUNT    = 9,
    parameter integer g_FP_COUNT     = 13,
    parameter integer g_COARSE_COUNT = 25,
    parameter integer g_RO_LENGTH    = 31
) (
    input  wire                               clk_i,
    input  wire                               reset_i,
    input  wire [g_COARSE_COUNT-1:0]          coarse_i,
    input  wire [g_COARSE_COUNT+g_FP_COUNT-1:0] deskew_i,
    input  wire                               signal_i,
    input  wire                               calib_i,
    input  wire                               calib_sel_i,
    output reg                                detect_o,
    output wire                               polarity_o,
    output wire [g_RAW_COUNT-1:0]             raw_o,
    output reg  [g_COARSE_COUNT+g_FP_COUNT-1:0] fp_o,
    input  wire [g_RAW_COUNT-1:0]             lut_a_i,
    input  wire                               lut_we_i,
    input  wire [g_FP_COUNT-1:0]              lut_d_i,
    output wire [g_FP_COUNT-1:0]              lut_d_o,
    input  wire                               ro_en_i,
    output wire                               ro_clk_o
);

reg                                calib_sel_d;
wire                               muxed_signal;
wire                               inv_signal;
wire [4*g_CARRY4_COUNT-1:0]        taps;
wire                               ipolarity;
wire                               polarity;
reg                                polarity_d1;
reg                                polarity_d2;
wire                               detect_d1;
wire [g_RAW_COUNT-1:0]             raw;
reg  [g_RAW_COUNT-1:0]             raw_d1;
reg  [g_RAW_COUNT-1:0]             raw_d2;
wire [g_FP_COUNT-1:0]              lut;
wire                               ro_en;
wire [g_COARSE_COUNT+g_FP_COUNT-1:0] coarse_ext;
wire [g_COARSE_COUNT+g_FP_COUNT-1:0] lut_ext;

// 对校准选择信号打一拍，避免直接切换数据源时产生毛刺。
always @(posedge clk_i) begin
    calib_sel_d <= calib_sel_i;
end

assign muxed_signal = calib_sel_d ? calib_i : signal_i;
// ipolarity 由编码器给出，用于在不同极性之间统一成“前导 1”问题。
assign inv_signal   = muxed_signal ^ (~ipolarity);

tdc_delayline #(
    .g_WIDTH(g_CARRY4_COUNT)
) cmp_delayline (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .signal_i(inv_signal),
    .taps_o(taps)
);

tdc_lbc #(
    .g_N(g_RAW_COUNT),
    .g_NIN(g_CARRY4_COUNT*4),
    .g_IGNORE(2)
) cmp_lbc (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .d_i(taps),
    .ipolarity_o(ipolarity),
    .polarity_o(polarity),
    .count_o(raw)
);

// LUT 双口 RAM:
//   A 口: 由原始码 raw 自动读取时间细分值
//   B 口: 外部控制器/调试接口写入或读取校准 LUT
generic_dpram #(
    .g_data_width(g_FP_COUNT),
    .g_size(2**g_RAW_COUNT),
    .g_with_byte_enable(0),
    .g_addr_conflict_resolution("read_first"),
    .g_init_file(""),
    .g_dual_clock(0)
) cmp_lut (
    .clka_i(clk_i),
    .clkb_i(1'b0),
    .wea_i(1'b0),
    .bwea_i(1'b0),
    .aa_i(raw),
    .da_i({g_FP_COUNT{1'b0}}),
    .qa_o(lut),
    .web_i(lut_we_i),
    .bweb_i(1'b0),
    .ab_i(lut_a_i),
    .db_i(lut_d_i),
    .qb_o(lut_d_o)
);

tdc_ringosc #(
    .g_LENGTH(g_RO_LENGTH)
) cmp_ringosc (
    .en_i(ro_en),
    .clk_o(ro_clk_o)
);

assign ro_en     = ro_en_i & ~reset_i;
assign detect_d1 = polarity_d1 ^ polarity_d2;
assign polarity_o = polarity_d2;
assign raw_o      = raw_d2;
assign coarse_ext = {coarse_i, {g_FP_COUNT{1'b0}}};
assign lut_ext    = {{g_COARSE_COUNT{1'b0}}, lut};

// 检测边沿后，锁存对应的极性和原始码。
always @(posedge clk_i) begin
    if (reset_i) begin
        detect_o     <= 1'b0;
        polarity_d1  <= 1'b1;
        polarity_d2  <= 1'b1;
        raw_d1       <= {g_RAW_COUNT{1'b0}};
        raw_d2       <= {g_RAW_COUNT{1'b0}};
    end else begin
        detect_o    <= detect_d1;
        polarity_d1 <= polarity;
        raw_d1      <= raw;
        if (detect_d1) begin
            polarity_d2 <= polarity_d1;
            raw_d2      <= raw_d1;
        end
    end
end

// 细时间 = 粗计数扩展值 - LUT 修正值 + deskew。
always @(posedge clk_i) begin
    if (reset_i) begin
        fp_o <= {(g_COARSE_COUNT+g_FP_COUNT){1'b0}};
    end else if (detect_d1) begin
        fp_o <= coarse_ext - lut_ext + deskew_i;
    end
end

endmodule
