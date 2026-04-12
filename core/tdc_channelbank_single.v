// -----------------------------------------------------------------------------
// Single-channel bank
// -----------------------------------------------------------------------------
// 单通道情形下的简化实现:
//   - 只有一个 channel
//   - histogram 为单通道 RAM
//   - 频率计数器直接测该通道环振
// -----------------------------------------------------------------------------
module tdc_channelbank_single #(
    parameter integer g_CARRY4_COUNT   = 124,
    parameter integer g_RAW_COUNT      = 9,
    parameter integer g_FP_COUNT       = 13,
    parameter integer g_EXHIS_COUNT    = 4,
    parameter integer g_COARSE_COUNT   = 25,
    parameter integer g_RO_LENGTH      = 31,
    parameter integer g_FCOUNTER_WIDTH = 13,
    parameter integer g_FTIMER_WIDTH   = 14
) (
    input  wire                               clk_i,
    input  wire                               reset_i,
    input  wire                               cc_rst_i,
    output reg                                cc_cy_o,
    input  wire                               next_i,
    output wire                               last_o,
    input  wire                               calib_sel_i,
    input  wire [g_COARSE_COUNT+g_FP_COUNT-1:0] deskew_i,
    input  wire                               signal_i,
    input  wire                               calib_i,
    output wire                               detect_o,
    output wire                               polarity_o,
    output wire [g_RAW_COUNT-1:0]             raw_o,
    output wire [g_COARSE_COUNT+g_FP_COUNT-1:0] fp_o,
    input  wire [g_RAW_COUNT-1:0]             lut_a_i,
    input  wire                               lut_we_i,
    input  wire [g_FP_COUNT-1:0]              lut_d_i,
    output wire [g_FP_COUNT-1:0]              lut_d_o,
    output wire                               c_detect_o,
    output wire [g_RAW_COUNT-1:0]             c_raw_o,
    input  wire [g_RAW_COUNT-1:0]             his_a_i,
    input  wire                               his_we_i,
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_i,
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_o,
    input  wire                               oc_start_i,
    output wire                               oc_ready_o,
    output wire [g_FCOUNTER_WIDTH-1:0]        oc_freq_o,
    input  wire                               oc_store_i,
    output wire [g_FCOUNTER_WIDTH-1:0]        oc_sfreq_o
);

wire [g_RAW_COUNT-1:0]                raw;
wire                                  detect;
reg  [g_COARSE_COUNT-1:0]             coarse_counter;
wire                                  ro_clk;
wire [g_FCOUNTER_WIDTH-1:0]           freq;
reg  [g_FCOUNTER_WIDTH-1:0]           sfreq_s;

// 单通道处理链。
tdc_channel #(
    .g_CARRY4_COUNT(g_CARRY4_COUNT),
    .g_RAW_COUNT(g_RAW_COUNT),
    .g_FP_COUNT(g_FP_COUNT),
    .g_COARSE_COUNT(g_COARSE_COUNT),
    .g_RO_LENGTH(g_RO_LENGTH)
) cmp_channel (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .coarse_i(coarse_counter),
    .deskew_i(deskew_i),
    .signal_i(signal_i),
    .calib_i(calib_i),
    .calib_sel_i(calib_sel_i),
    .detect_o(detect),
    .polarity_o(polarity_o),
    .raw_o(raw),
    .fp_o(fp_o),
    .lut_a_i(lut_a_i),
    .lut_we_i(lut_we_i),
    .lut_d_i(lut_d_i),
    .lut_d_o(lut_d_o),
    .ro_en_i(1'b1),
    .ro_clk_o(ro_clk)
);

assign detect_o   = detect;
assign raw_o      = raw;
assign c_detect_o = detect;
assign c_raw_o    = raw;

// 单端口 RAM 保存该通道的 histogram。
generic_spram #(
    .g_data_width(g_FP_COUNT+g_EXHIS_COUNT),
    .g_size(2**g_RAW_COUNT),
    .g_with_byte_enable(0),
    .g_init_file(""),
    .g_addr_conflict_resolution("read_first")
) cmp_histogram (
    .rst_n_i(1'b1),
    .clk_i(clk_i),
    .bwe_i(1'b0),
    .we_i(his_we_i),
    .a_i(his_a_i),
    .d_i(his_d_i),
    .q_o(his_d_o)
);

// 测量当前环振频率。
tdc_freqc #(
    .g_COUNTER_WIDTH(g_FCOUNTER_WIDTH),
    .g_TIMER_WIDTH(g_FTIMER_WIDTH)
) cmp_freqc (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .clk_m_i(ro_clk),
    .start_i(oc_start_i),
    .ready_o(oc_ready_o),
    .freq_o(freq)
);

assign oc_freq_o = freq;

// 粗计数器：自由运行计数，用于与细时间拼接。
always @(posedge clk_i) begin
    if (reset_i || cc_rst_i) begin
        coarse_counter <= {g_COARSE_COUNT{1'b0}};
        cc_cy_o        <= 1'b0;
    end else begin
        coarse_counter <= coarse_counter + 1'b1;
        cc_cy_o        <= &coarse_counter;
    end
end

// 在控制器发出 oc_store_i 时，保存当前频率测量值。
always @(posedge clk_i) begin
    if (oc_store_i) begin
        sfreq_s <= freq;
    end
end

assign oc_sfreq_o = sfreq_s;
// 单通道情况下，当前通道永远是最后一个通道。
assign last_o = 1'b1;

endmodule
