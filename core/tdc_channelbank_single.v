// -----------------------------------------------------------------------------
// Single-channel bank
// -----------------------------------------------------------------------------
// 单通道情形下的简化实现:
//   - 只有一个 channel
//   - histogram 为单通道 RAM
//   - 频率计数器直接测该通道环振
// 这个模块本质上把以下几部分连接到一起:
//   1) 一个 tdc_channel 单通道测量核心
//   2) 一块单端口 histogram RAM
//   3) 一个在线校准频率计数器 tdc_freqc
//   4) 一个自由运行的 coarse counter
// -----------------------------------------------------------------------------
module tdc_channelbank_single #(
    parameter integer g_CARRY4_COUNT   = 124,                         // 单通道延迟线中 CARRY4 的个数
    parameter integer g_RAW_COUNT      = 9,                           // 原始细时间码 raw 的位宽
    parameter integer g_FP_COUNT       = 13,                          // 细时间/LUT 的位宽
    parameter integer g_EXHIS_COUNT    = 4,                           // histogram 数据额外扩展位宽
    parameter integer g_COARSE_COUNT   = 25,                          // 粗计数器位宽
    parameter integer g_RO_LENGTH      = 31,                          // 环振长度参数
    parameter integer g_FCOUNTER_WIDTH = 13,                          // 频率计数器输出位宽
    parameter integer g_FTIMER_WIDTH   = 14                           // 频率测量窗口定时器位宽
) (
    input  wire                               clk_i,                  // 系统时钟输入
    input  wire                               reset_i,                // 系统复位输入，高有效
    input  wire                               cc_rst_i,               // 粗计数器复位输入
    output reg                                cc_cy_o,                // 粗计数器回卷标志输出
    input  wire                               next_i,                 // 下一通道选择输入（单通道下基本无实际作用）
    output wire                               last_o,                 // 是否为最后一个通道的标志输出
    input  wire                               calib_sel_i,            // 选择正常输入还是校准输入
    input  wire [g_COARSE_COUNT+g_FP_COUNT-1:0] deskew_i,             // 本通道的 deskew 补偿值
    input  wire                               signal_i,               // 本通道正常待测输入
    input  wire                               calib_i,                // 本通道校准输入
    output wire                               detect_o,               // 本通道事件检测输出
    output wire                               polarity_o,             // 本通道极性输出
    output wire [g_RAW_COUNT-1:0]             raw_o,                  // 本通道 raw 输出
    output wire [g_COARSE_COUNT+g_FP_COUNT-1:0] fp_o,                 // 本通道最终时间输出
    input  wire [g_RAW_COUNT-1:0]             lut_a_i,                // LUT 地址输入（controller/调试接口使用）
    input  wire                               lut_we_i,               // LUT 写使能输入
    input  wire [g_FP_COUNT-1:0]              lut_d_i,                // LUT 写数据输入
    output wire [g_FP_COUNT-1:0]              lut_d_o,                // LUT 读数据输出
    output wire                               c_detect_o,             // 提供给 controller 的当前通道 detect 输出
    output wire [g_RAW_COUNT-1:0]             c_raw_o,                // 提供给 controller 的当前通道 raw 输出
    input  wire [g_RAW_COUNT-1:0]             his_a_i,                // histogram RAM 地址输入
    input  wire                               his_we_i,               // histogram RAM 写使能输入
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_i,               // histogram RAM 写数据输入
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_o,               // histogram RAM 读数据输出
    input  wire                               oc_start_i,             // 在线校准频率测量启动输入
    output wire                               oc_ready_o,             // 在线校准频率测量就绪输出
    output wire [g_FCOUNTER_WIDTH-1:0]        oc_freq_o,              // 当前测得频率输出
    input  wire                               oc_store_i,             // 保存参考频率 sfreq 的控制输入
    output wire [g_FCOUNTER_WIDTH-1:0]        oc_sfreq_o              // 已保存的参考频率 sfreq 输出
);

wire [g_RAW_COUNT-1:0]                raw;                            // 从单通道测量核心输出的 raw 码
wire                                  detect;                         // 从单通道测量核心输出的 detect 脉冲
reg  [g_COARSE_COUNT-1:0]             coarse_counter;                 // 自由运行的粗计数器
wire                                  ro_clk;                         // 本通道环振输出时钟
wire [g_FCOUNTER_WIDTH-1:0]           freq;                           // 当前测得的环振频率计数值
reg  [g_FCOUNTER_WIDTH-1:0]           sfreq_s;                        // 保存下来的参考频率 sfreq

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
        cc_cy_o        <= &coarse_counter; // 当 coarse_counter 当前全为 1 时，下一拍会回卷，因此这里输出回卷标志
    end
end

// 在控制器发出 oc_store_i 时，保存当前频率测量值。
always @(posedge clk_i) begin
    if (oc_store_i) begin
        sfreq_s <= freq; // 把当前测得频率写入 sfreq_s
    end
end

assign oc_sfreq_o = sfreq_s; // 将保存的参考频率输出到模块外
// 单通道情况下，当前通道永远是最后一个通道。
assign last_o = 1'b1;

endmodule
