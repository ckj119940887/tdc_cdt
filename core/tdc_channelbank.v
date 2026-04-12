// -----------------------------------------------------------------------------
// Automatic channel-bank wrapper
// -----------------------------------------------------------------------------
// 根据 g_CHANNEL_COUNT 自动在单通道实现和多通道实现之间切换。
// -----------------------------------------------------------------------------
module tdc_channelbank #(
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
    input  wire                                      cc_rst_i,
    output wire                                      cc_cy_o,
    input  wire                                      next_i,
    output wire                                      last_o,
    input  wire                                      calib_sel_i,
    input  wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] deskew_i,
    input  wire [g_CHANNEL_COUNT-1:0]                signal_i,
    input  wire [g_CHANNEL_COUNT-1:0]                calib_i,
    output wire [g_CHANNEL_COUNT-1:0]                detect_o,
    output wire [g_CHANNEL_COUNT-1:0]                polarity_o,
    output wire [g_CHANNEL_COUNT*g_RAW_COUNT-1:0]    raw_o,
    output wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] fp_o,
    input  wire [g_RAW_COUNT-1:0]                    lut_a_i,
    input  wire                                      lut_we_i,
    input  wire [g_FP_COUNT-1:0]                     lut_d_i,
    output wire [g_FP_COUNT-1:0]                     lut_d_o,
    output wire                                      c_detect_o,
    output wire [g_RAW_COUNT-1:0]                    c_raw_o,
    input  wire [g_RAW_COUNT-1:0]                    his_a_i,
    input  wire                                      his_we_i,
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_i,
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_o,
    input  wire                                      oc_start_i,
    output wire                                      oc_ready_o,
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_freq_o,
    input  wire                                      oc_store_i,
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_sfreq_o
);

generate
    if (g_CHANNEL_COUNT == 1) begin : g_single
        tdc_channelbank_single #(
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
            .next_i(next_i),
            .last_o(last_o),
            .calib_sel_i(calib_sel_i),
            .deskew_i(deskew_i),
            .signal_i(signal_i[0]),
            .calib_i(calib_i[0]),
            .detect_o(detect_o[0]),
            .polarity_o(polarity_o[0]),
            .raw_o(raw_o),
            .fp_o(fp_o),
            .lut_a_i(lut_a_i),
            .lut_we_i(lut_we_i),
            .lut_d_i(lut_d_i),
            .lut_d_o(lut_d_o),
            .c_detect_o(c_detect_o),
            .c_raw_o(c_raw_o),
            .his_a_i(his_a_i),
            .his_we_i(his_we_i),
            .his_d_i(his_d_i),
            .his_d_o(his_d_o),
            .oc_start_i(oc_start_i),
            .oc_ready_o(oc_ready_o),
            .oc_freq_o(oc_freq_o),
            .oc_store_i(oc_store_i),
            .oc_sfreq_o(oc_sfreq_o)
        );
    end else begin : g_multi
        tdc_channelbank_multi #(
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
            .next_i(next_i),
            .last_o(last_o),
            .calib_sel_i(calib_sel_i),
            .deskew_i(deskew_i),
            .signal_i(signal_i),
            .calib_i(calib_i),
            .detect_o(detect_o),
            .polarity_o(polarity_o),
            .raw_o(raw_o),
            .fp_o(fp_o),
            .lut_a_i(lut_a_i),
            .lut_we_i(lut_we_i),
            .lut_d_i(lut_d_i),
            .lut_d_o(lut_d_o),
            .c_detect_o(c_detect_o),
            .c_raw_o(c_raw_o),
            .his_a_i(his_a_i),
            .his_we_i(his_we_i),
            .his_d_i(his_d_i),
            .his_d_o(his_d_o),
            .oc_start_i(oc_start_i),
            .oc_ready_o(oc_ready_o),
            .oc_freq_o(oc_freq_o),
            .oc_store_i(oc_store_i),
            .oc_sfreq_o(oc_sfreq_o)
        );
    end
endgenerate

endmodule
