// -----------------------------------------------------------------------------
// Multi-channel bank
// -----------------------------------------------------------------------------
// 多通道实现特点:
//   - 所有 channel 并行实例化
//   - 通过 one-hot 选择当前被控制/被标定的通道
//   - histogram RAM 在各通道之间共享
//   - 频率计数器共享，仅当前通道的环振接入测量
// -----------------------------------------------------------------------------
module tdc_channelbank_multi #(
    parameter integer g_CHANNEL_COUNT  = 2,
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
    output reg                                       cc_cy_o,
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
    output reg  [g_FP_COUNT-1:0]                     lut_d_o,
    output reg                                       c_detect_o,
    output reg  [g_RAW_COUNT-1:0]                    c_raw_o,
    input  wire [g_RAW_COUNT-1:0]                    his_a_i,
    input  wire                                      his_we_i,
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_i,
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_o,
    input  wire                                      oc_start_i,
    output wire                                      oc_ready_o,
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_freq_o,
    input  wire                                      oc_store_i,
    output reg  [g_FCOUNTER_WIDTH-1:0]               oc_sfreq_o
);

function integer f_log2_size;
    input a;
    integer a;
    integer i;
    begin
        f_log2_size = 1;
        for (i = 1; i <= 64; i = i + 1) begin
            if ((2**i) >= a) begin
                f_log2_size = i;
                i = 65;
            end
        end
    end
endfunction

localparam integer CHANNEL_INDEX_W = f_log2_size(g_CHANNEL_COUNT);

wire [g_CHANNEL_COUNT-1:0]                 detect;
wire [g_CHANNEL_COUNT*g_RAW_COUNT-1:0]     raw;
reg  [g_COARSE_COUNT-1:0]                  coarse_counter;
reg  [g_CHANNEL_COUNT-1:0]                 current_channel_onehot;
reg  [CHANNEL_INDEX_W-1:0]                 current_channel;
wire [g_CHANNEL_COUNT*g_FP_COUNT-1:0]      lut_d_o_s;
wire [CHANNEL_INDEX_W+g_RAW_COUNT-1:0]     his_full_a;
wire [g_CHANNEL_COUNT-1:0]                 ro_clk_s;
wire                                       ro_clk;
wire [g_FCOUNTER_WIDTH-1:0]                freq;
reg  [g_CHANNEL_COUNT*g_FCOUNTER_WIDTH-1:0] sfreq_s;

integer idx;

genvar i;
generate
    for (i = 0; i < g_CHANNEL_COUNT; i = i + 1) begin : g_channels
        wire this_calib_sel;
        wire this_lut_we;
        assign this_calib_sel = current_channel_onehot[i] & calib_sel_i;
        assign this_lut_we    = current_channel_onehot[i] & lut_we_i;

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
            .deskew_i(deskew_i[(i+1)*(g_COARSE_COUNT+g_FP_COUNT)-1 -: (g_COARSE_COUNT+g_FP_COUNT)]),
            .signal_i(signal_i[i]),
            .calib_i(calib_i[i]),
            .calib_sel_i(this_calib_sel),
            .detect_o(detect[i]),
            .polarity_o(polarity_o[i]),
            .raw_o(raw[(i+1)*g_RAW_COUNT-1 -: g_RAW_COUNT]),
            .fp_o(fp_o[(i+1)*(g_COARSE_COUNT+g_FP_COUNT)-1 -: (g_COARSE_COUNT+g_FP_COUNT)]),
            .lut_a_i(lut_a_i),
            .lut_we_i(this_lut_we),
            .lut_d_i(lut_d_i),
            .lut_d_o(lut_d_o_s[(i+1)*g_FP_COUNT-1 -: g_FP_COUNT]),
            .ro_en_i(current_channel_onehot[i]),
            .ro_clk_o(ro_clk_s[i])
        );
    end
endgenerate

assign detect_o   = detect;
assign raw_o      = raw;
assign his_full_a = {current_channel, his_a_i};

// 多通道共享 histogram RAM，地址高位为当前通道号，低位为原始码。
generic_spram #(
    .g_data_width(g_FP_COUNT+g_EXHIS_COUNT),
    .g_size(g_CHANNEL_COUNT*(2**g_RAW_COUNT)),
    .g_with_byte_enable(0),
    .g_init_file(""),
    .g_addr_conflict_resolution("read_first")
) cmp_histogram (
    .rst_n_i(1'b1),
    .clk_i(clk_i),
    .bwe_i(1'b0),
    .we_i(his_we_i),
    .a_i(his_full_a),
    .d_i(his_d_i),
    .q_o(his_d_o)
);

// 共享频率计数器，仅测当前通道的环振输出。
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

// 粗计数器由所有通道共享。
always @(posedge clk_i) begin
    if (reset_i || cc_rst_i) begin
        coarse_counter <= {g_COARSE_COUNT{1'b0}};
        cc_cy_o        <= 1'b0;
    end else begin
        coarse_counter <= coarse_counter + 1'b1;
        cc_cy_o        <= &coarse_counter;
    end
end

// 选择当前通道的 LUT 输出。
always @* begin
    lut_d_o = {g_FP_COUNT{1'b0}};
    for (idx = 0; idx < g_CHANNEL_COUNT; idx = idx + 1) begin
        if (current_channel_onehot[idx])
            lut_d_o = lut_d_o | lut_d_o_s[(idx+1)*g_FP_COUNT-1 -: g_FP_COUNT];
    end
end

// 选择当前通道的 detect/raw，用于 histogram 更新。
always @* begin
    c_detect_o = 1'b0;
    c_raw_o    = {g_RAW_COUNT{1'b0}};
    for (idx = 0; idx < g_CHANNEL_COUNT; idx = idx + 1) begin
        if (current_channel_onehot[idx]) begin
            c_detect_o = c_detect_o | detect[idx];
            c_raw_o    = c_raw_o | raw[(idx+1)*g_RAW_COUNT-1 -: g_RAW_COUNT];
        end
    end
end

// 未使能的环振输出恒为 0，因此所有环振输出取 OR 即可得到当前测量时钟。
assign ro_clk = |ro_clk_s;

// 保存各通道的环振频率结果。
always @(posedge clk_i) begin
    if (oc_store_i) begin
        for (idx = 0; idx < g_CHANNEL_COUNT; idx = idx + 1) begin
            if (current_channel_onehot[idx])
                sfreq_s[(idx+1)*g_FCOUNTER_WIDTH-1 -: g_FCOUNTER_WIDTH] <= freq;
        end
    end
end

// 输出当前通道对应的已保存频率。
always @* begin
    oc_sfreq_o = {g_FCOUNTER_WIDTH{1'b0}};
    for (idx = 0; idx < g_CHANNEL_COUNT; idx = idx + 1) begin
        if (current_channel_onehot[idx])
            oc_sfreq_o = oc_sfreq_o | sfreq_s[(idx+1)*g_FCOUNTER_WIDTH-1 -: g_FCOUNTER_WIDTH];
    end
end

// one-hot 通道轮转控制。
always @(posedge clk_i) begin
    if (reset_i) begin
        current_channel_onehot <= {{(g_CHANNEL_COUNT-1){1'b0}}, 1'b1};
    end else if (next_i) begin
        current_channel_onehot <= {current_channel_onehot[g_CHANNEL_COUNT-2:0], current_channel_onehot[g_CHANNEL_COUNT-1]};
    end
end

assign last_o = current_channel_onehot[g_CHANNEL_COUNT-1];

// 把 one-hot 编码转换成通道编号，供共享 histogram RAM 寻址使用。
always @* begin
    current_channel = {CHANNEL_INDEX_W{1'b0}};
    for (idx = 0; idx < g_CHANNEL_COUNT; idx = idx + 1) begin
        if (current_channel_onehot[idx])
            current_channel = current_channel | idx;
    end
end

endmodule
