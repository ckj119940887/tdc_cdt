// -----------------------------------------------------------------------------
// TDC top module
// -----------------------------------------------------------------------------
// 顶层模块，连接 channel bank 与 controller，并在冻结调试模式下把内部控制口
// 切换到外部调试接口。
// 本模块本身不直接做延迟链测量，也不直接做 LUT 算法计算；
// 它的作用是：
//   1) 实例化测量通路 tdc_channelbank
//   2) 实例化控制通路 tdc_controller
//   3) 在正常模式下让 controller 接管内部控制口
//   4) 在 freeze/debug 模式下让外部调试口接管这些控制口
// -----------------------------------------------------------------------------
module tdc #(
    parameter integer g_CHANNEL_COUNT  = 1,                              // 通道数量参数
    parameter integer g_CARRY4_COUNT   = 124,                            // 每个通道延迟线中 CARRY4 的个数
    parameter integer g_RAW_COUNT      = 9,                              // 原始细时间码 raw 的位宽
    parameter integer g_FP_COUNT       = 13,                             // 细时间/LUT 的位宽
    parameter integer g_EXHIS_COUNT    = 4,                              // histogram 数据额外扩展位宽
    parameter integer g_COARSE_COUNT   = 25,                             // 粗计数位宽
    parameter integer g_RO_LENGTH      = 31,                             // 环振长度参数
    parameter integer g_FCOUNTER_WIDTH = 13,                             // 频率计数器输出位宽
    parameter integer g_FTIMER_WIDTH   = 14                              // 频率测量窗口计时器位宽
) (
    input  wire                                      clk_i,              // 系统时钟输入
    input  wire                                      reset_i,            // 系统复位输入，高有效
    output wire                                      ready_o,            // 控制器 ready 输出：表示启动校准完成、系统可正常工作

    input  wire                                      cc_rst_i,           // 粗计数器复位输入
    output wire                                      cc_cy_o,            // 粗计数器回卷/进位标志输出

    input  wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] deskew_i, // 所有通道的 deskew 拼接输入

    input  wire [g_CHANNEL_COUNT-1:0]                signal_i,           // 所有通道的正常待测输入
    input  wire [g_CHANNEL_COUNT-1:0]                calib_i,            // 所有通道的校准输入

    output wire [g_CHANNEL_COUNT-1:0]                detect_o,           // 所有通道的 detect 输出
    output wire [g_CHANNEL_COUNT-1:0]                polarity_o,         // 所有通道的 polarity 输出
    output wire [g_CHANNEL_COUNT*g_RAW_COUNT-1:0]    raw_o,              // 所有通道的 raw 输出拼接总线
    output wire [g_CHANNEL_COUNT*(g_COARSE_COUNT+g_FP_COUNT)-1:0] fp_o,  // 所有通道的最终时间输出拼接总线

    input  wire                                      freeze_req_i,       // 外部冻结请求输入：请求切换到调试接管模式
    output wire                                      freeze_ack_o,       // 冻结确认输出：为 1 表示当前调试口已接管内部控制口
    input  wire                                      cs_next_i,          // 调试模式下：外部手动切换下一通道输入
    output wire                                      cs_last_o,          // 当前是否已到最后一个通道输出
    input  wire                                      calib_sel_i,        // 调试模式下：外部手动选择正常输入/校准输入
    input  wire [g_RAW_COUNT-1:0]                    lut_a_i,            // 调试模式下：外部手动给出的 LUT 地址
    output wire [g_FP_COUNT-1:0]                     lut_d_o,            // LUT 读数据输出给外部
    input  wire [g_RAW_COUNT-1:0]                    his_a_i,            // 调试模式下：外部手动给出的 histogram 地址
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]       his_d_o,            // histogram 读数据输出给外部
    input  wire                                      oc_start_i,         // 调试模式下：外部手动启动频率测量
    output wire                                      oc_ready_o,         // 频率测量模块 ready 输出给外部
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_freq_o,          // 当前测得频率输出给外部
    output wire [g_FCOUNTER_WIDTH-1:0]               oc_sfreq_o          // 当前保存的参考频率 sfreq 输出给外部
);

wire                                 cs_next;                           // 实际送入 channelbank 的“切换下一通道”控制
wire                                 cs_next_c;                         // 由 controller 产生的 next 控制
wire                                 cs_last;                           // 由 channelbank 返回的“最后一个通道”标志
wire                                 calib_sel;                         // 实际送入 channelbank 的输入选择控制
wire                                 calib_sel_c;                       // 由 controller 产生的 calib_sel 控制

wire [g_RAW_COUNT-1:0]               lut_a;                             // 实际送入 channelbank 的 LUT 地址
wire [g_RAW_COUNT-1:0]               lut_a_c;                           // 由 controller 产生的 LUT 地址
wire                                 lut_we;                            // LUT 写使能，由 controller 直接驱动
wire [g_FP_COUNT-1:0]                lut_d_w;                           // 写入 LUT 的数据，由 controller 直接驱动
wire [g_FP_COUNT-1:0]                lut_d_r;                           // 从 channelbank/LUT RAM 读回的数据

wire                                 c_detect;                          // 当前被 controller 关注通道的 detect 输出
wire [g_RAW_COUNT-1:0]               c_raw;                             // 当前被 controller 关注通道的 raw 输出
wire [g_RAW_COUNT-1:0]               his_a;                             // 实际送入 channelbank 的 histogram 地址
wire [g_RAW_COUNT-1:0]               his_a_c;                           // 由 controller 产生的 histogram 地址
wire                                 his_we;                            // histogram 写使能，由 controller 直接驱动
wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]  his_d_w;                           // 写入 histogram 的数据，由 controller 直接驱动
wire [g_FP_COUNT+g_EXHIS_COUNT-1:0]  his_d_r;                           // 从 histogram RAM 读回的数据

wire                                 oc_start;                          // 实际送入 channelbank 的频率测量启动控制
wire                                 oc_start_c;                        // 由 controller 产生的频率测量启动控制
wire                                 oc_ready;                          // 频率测量模块 ready 输出
wire [g_FCOUNTER_WIDTH-1:0]          oc_freq;                           // 当前测得的频率值
wire                                 oc_store;                          // 保存参考频率 sfreq 的控制，由 controller 产生
wire [g_FCOUNTER_WIDTH-1:0]          oc_sfreq;                          // 已保存的参考频率值

wire                                 freeze_ack;                        // controller 返回的冻结确认内部信号

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
// freeze_ack = 0 时：
//   内部控制信号全部来自 controller，系统正常自动运行。
// freeze_ack = 1 时：
//   cs_next / calib_sel / lut_a / his_a / oc_start 改由外部调试端口接管，
//   便于手动检查 LUT、histogram、频率测量等内部状态。
assign cs_next      = freeze_ack ? cs_next_i    : cs_next_c;             // 选择实际的 next 控制来源：外部调试口或 controller
assign calib_sel    = freeze_ack ? calib_sel_i  : calib_sel_c;           // 选择实际的输入选择控制来源
assign lut_a        = freeze_ack ? lut_a_i      : lut_a_c;               // 选择实际的 LUT 地址来源
assign his_a        = freeze_ack ? his_a_i      : his_a_c;               // 选择实际的 histogram 地址来源
assign oc_start     = freeze_ack ? oc_start_i   : oc_start_c;            // 选择实际的频率测量启动来源
assign freeze_ack_o = freeze_ack;                                        // 将内部 freeze_ack 直接输出到模块外
assign cs_last_o    = cs_last;                                           // 将“是否最后一个通道”输出到模块外
assign lut_d_o      = lut_d_r;                                           // 将 LUT 读数据直接输出到模块外
assign his_d_o      = his_d_r;                                           // 将 histogram 读数据直接输出到模块外
assign oc_ready_o   = oc_ready;                                          // 将频率测量 ready 直接输出到模块外
assign oc_freq_o    = oc_freq;                                           // 将当前频率测量值直接输出到模块外
assign oc_sfreq_o   = oc_sfreq;                                          // 将已保存的参考频率直接输出到模块外

endmodule
