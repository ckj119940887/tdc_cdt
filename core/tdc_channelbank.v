// -----------------------------------------------------------------------------
// Automatic channel-bank wrapper
// -----------------------------------------------------------------------------
// 根据 g_CHANNEL_COUNT 自动在单通道实现和多通道实现之间切换。
// -----------------------------------------------------------------------------
module tdc_channelbank #(
    parameter integer g_CHANNEL_COUNT  = 1,                            // 通道数量参数：决定实例化单通道版还是多通道版
    parameter integer g_CARRY4_COUNT   = 124,                          // 每个通道延迟线中使用的 CARRY4 个数
    parameter integer g_RAW_COUNT      = 9,                            // 原始细时间码 raw 的位宽
    parameter integer g_FP_COUNT       = 13,                           // 细时间/LUT 的位宽
    parameter integer g_EXHIS_COUNT    = 4,                            // histogram 额外扩展位宽
    parameter integer g_COARSE_COUNT   = 25,                           // 粗计数位宽
    parameter integer g_RO_LENGTH      = 31,                           // 环振长度参数
    parameter integer g_FCOUNTER_WIDTH = 13,                           // 频率计数器输出位宽
    parameter integer g_FTIMER_WIDTH   = 14                            // 频率测量窗口计时器位宽
) (
    input  wire                                      clk_i,             // 系统时钟输入
    input  wire                                      reset_i,           // 系统复位输入，高有效
    input  wire                                      cc_rst_i,          // 粗计数器复位输入
    output wire                                      cc_cy_o,           // 粗计数器回卷/进位输出
    input  wire                                      next_i,            // 多通道时切换到下一通道的控制输入
    output wire                                      last_o,            // 当前是否已经到最后一个通道的标志输出
    input  wire                                      calib_sel_i,       // 输入选择控制：正常信号还是校准信号
    input  wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] deskew_i, // 所有通道的 deskew 拼接输入
    input  wire [g_CHANNEL_COUNT-1:0]                signal_i,          // 所有通道的正常待测输入信号
    input  wire [g_CHANNEL_COUNT-1:0]                calib_i,           // 所有通道的校准输入信号
    output wire [g_CHANNEL_COUNT-1:0]                detect_o,          // 所有通道的事件检测输出
    output wire [g_CHANNEL_COUNT-1:0]                polarity_o,        // 所有通道的极性输出
    output wire [g_CHANNEL_COUNT*g_RAW_COUNT-1:0]    raw_o,             // 所有通道的 raw 输出拼接总线
    output wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] fp_o, // 所有通道的最终时间输出拼接总线
    input  wire [g_RAW_COUNT-1:0]                    lut_a_i,           // LUT 访问地址输入（供 controller/调试接口使用）
    input  wire                                      lut_we_i,          // LUT 写使能输入
    input  wire [g_FP_COUNT-1:0]                     lut_d_i,           // LUT 写数据输入
    output wire [g_FP_COUNT-1:0]                     lut_d_o,           // LUT 读数据输出
    output wire                                      c_detect_o,        // 当前被 controller 关注通道的 detect 输出
    output wire [g_RAW_COUNT-1:0]                    c_raw_o,           // 当前被 controller 关注通道的 raw 输出
    input  wire [g_RAW_COUNT-1:0]                    his_a_i,           // histogram RAM 地址输入
    input  wire                                      his_we_i,          // histogram RAM 写使能输入
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_i,           // histogram RAM 写数据输入
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_o,           // histogram RAM 读数据输出
    input  wire                                      oc_start_i,        // 在线校准频率测量启动输入
    output wire                                      oc_ready_o,        // 在线校准频率测量就绪输出
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_freq_o,         // 当前测得的 ring oscillator 频率输出
    input  wire                                      oc_store_i,        // 保存参考频率 sfreq 的控制输入
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_sfreq_o         // 已保存的参考频率 sfreq 输出
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
