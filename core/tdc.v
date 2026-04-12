// -----------------------------------------------------------------------------
// TDC top module
// -----------------------------------------------------------------------------
// 顶层模块，连接 channel bank 与 controller，并在冻结调试模式下把内部控制口
// 切换到外部调试接口。
// -----------------------------------------------------------------------------
module tdc #(
    parameter integer g_CHANNEL_COUNT  = 1,
    parameter integer g_CARRY4_COUNT   = 124,
    parameter integer g_RAW_COUNT      = 9,
    parameter integer g_FP_COUNT       = 13,
    parameter integer g_EXHIS_COUNT    = 4,
    parameter integer g_COARSE_COUNT   = 25,
    parameter integer g_RO_LENGTH      = 31,
    parameter integer g_FCOUNTER_WIDTH = 13,
    parameter integer g_FTIMER_WIDTH   = 14
) (
    input  wire                                      clk_i,
    input  wire                                      reset_i,
    output wire                                      ready_o,

    input  wire                                      cc_rst_i,
    output wire                                      cc_cy_o,

    input  wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] deskew_i,

    input  wire [g_CHANNEL_COUNT-1:0]                signal_i,
    input  wire [g_CHANNEL_COUNT-1:0]                calib_i,

    output wire [g_CHANNEL_COUNT-1:0]                detect_o,
    output wire [g_CHANNEL_COUNT-1:0]                polarity_o,
    output wire [g_CHANNEL_COUNT*g_RAW_COUNT-1:0]    raw_o,
    output wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] fp_o,

    input  wire                                      freeze_req_i,
    output wire                                      freeze_ack_o,
    input  wire                                      cs_next_i,
    output wire                                      cs_last_o,
    input  wire                                      calib_sel_i,
    input  wire [g_RAW_COUNT-1:0]                    lut_a_i,
    output wire [g_FP_COUNT-1:0]                     lut_d_o,
    input  wire [g_RAW_COUNT-1:0]                    his_a_i,
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_o,
    input  wire                                      oc_start_i,
    output wire                                      oc_ready_o,
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_freq_o,
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_sfreq_o
);

wire                                 cs_next;
wire                                 cs_next_c;
wire                                 cs_last;
wire                                 calib_sel;
wire                                 calib_sel_c;

wire [g_RAW_COUNT-1:0]               lut_a;
wire [g_RAW_COUNT-1:0]               lut_a_c;
wire                                 lut_we;
wire [g_FP_COUNT-1:0]                lut_d_w;
wire [g_FP_COUNT-1:0]                lut_d_r;

wire                                 c_detect;
wire [g_RAW_COUNT-1:0]               c_raw;
wire [g_RAW_COUNT-1:0]               his_a;
wire [g_RAW_COUNT-1:0]               his_a_c;
wire                                 his_we;
wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]  his_d_w;
wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]  his_d_r;

wire                                 oc_start;
wire                                 oc_start_c;
wire                                 oc_ready;
wire [g_FCOUNTER_WIDTH-1:0]          oc_freq;
wire                                 oc_store;
wire [g_FCOUNTER_WIDTH-1:0]          oc_sfreq;

wire                                 freeze_ack;

// 通道阵列与测量相关逻辑。
tdc_channelbank #(
    .g_CHANNEL_COUNT(g_CHANNEL_COUNT),
    .g_CARRY4_COUNT(g_CARRY4_COUNT),
    .g_RAW_COUNT(g_RAW_COUNT),
    .g_FP_COUNT(g_FP_COUNT),
    .g_EXHIS_COUNT(g_EXHIS_COUNT),
    .g_COARSE_COUNT(g_COARSE_COUNT),
    .g_RO_LENGTH(g_RO_LENGTH),
    .g_FCOUNTER_WIDTH(g_FCOUNTER_WIDTH),
    .g_FTIMER_WIDTH(g_FTIMER_WIDTH)
) cmp_channelbank (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .cc_rst_i(cc_rst_i),
    .cc_cy_o(cc_cy_o),
    .next_i(cs_next),
    .last_o(cs_last),
    .calib_sel_i(calib_sel),
    .deskew_i(deskew_i),
    .signal_i(signal_i),
    .calib_i(calib_i),
    .detect_o(detect_o),
    .polarity_o(polarity_o),
    .raw_o(raw_o),
    .fp_o(fp_o),
    .lut_a_i(lut_a),
    .lut_we_i(lut_we),
    .lut_d_i(lut_d_w),
    .lut_d_o(lut_d_r),
    .c_detect_o(c_detect),
    .c_raw_o(c_raw),
    .his_a_i(his_a),
    .his_we_i(his_we),
    .his_d_i(his_d_w),
    .his_d_o(his_d_r),
    .oc_start_i(oc_start),
    .oc_ready_o(oc_ready),
    .oc_freq_o(oc_freq),
    .oc_store_i(oc_store),
    .oc_sfreq_o(oc_sfreq)
);

// 控制器负责启动标定、在线标定、LUT 写入以及冻结调试切换。
tdc_controller #(
    .g_RAW_COUNT(g_RAW_COUNT),
    .g_FP_COUNT(g_FP_COUNT),
    .g_EXHIS_COUNT(g_EXHIS_COUNT),
    .g_FCOUNTER_WIDTH(g_FCOUNTER_WIDTH)
) cmp_controller (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .ready_o(ready_o),
    .next_o(cs_next_c),
    .last_i(cs_last),
    .calib_sel_o(calib_sel_c),
    .lut_a_o(lut_a_c),
    .lut_we_o(lut_we),
    .lut_d_o(lut_d_w),
    .c_detect_i(c_detect),
    .c_raw_i(c_raw),
    .his_a_o(his_a_c),
    .his_we_o(his_we),
    .his_d_o(his_d_w),
    .his_d_i(his_d_r),
    .oc_start_o(oc_start_c),
    .oc_ready_i(oc_ready),
    .oc_freq_i(oc_freq),
    .oc_store_o(oc_store),
    .oc_sfreq_i(oc_sfreq),
    .freeze_req_i(freeze_req_i),
    .freeze_ack_o(freeze_ack)
);

// 冻结模式下，调试口直接接管内部控制信号；否则由控制器驱动。
assign cs_next      = freeze_ack ? cs_next_i    : cs_next_c;
assign calib_sel    = freeze_ack ? calib_sel_i  : calib_sel_c;
assign lut_a        = freeze_ack ? lut_a_i      : lut_a_c;
assign his_a        = freeze_ack ? his_a_i      : his_a_c;
assign oc_start     = freeze_ack ? oc_start_i   : oc_start_c;
assign freeze_ack_o = freeze_ack;
assign cs_last_o    = cs_last;
assign lut_d_o      = lut_d_r;
assign his_d_o      = his_d_r;
assign oc_ready_o   = oc_ready;
assign oc_freq_o    = oc_freq;
assign oc_sfreq_o   = oc_sfreq;

endmodule
