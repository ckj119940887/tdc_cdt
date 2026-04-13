// -----------------------------------------------------------------------------
// Single TDC channel
// -----------------------------------------------------------------------------
// 每个通道包含:
//   1) 延迟线
//   2) 前导位计数器/编码器
//   3) LUT 查表
//   4) 粗计数 + deskew 合成
//   5) 在线标定环振
// 本模块是单通道测量核心：
// 输入事件 -> 延迟线采样 -> 编码得到 raw -> 查 LUT 得 fine time -> 与 coarse 合成。
// -----------------------------------------------------------------------------
module tdc_channel #(
    parameter integer g_CARRY4_COUNT = 1,              // 延迟线中 CARRY4 的个数，总 tap 数 = 4 * g_CARRY4_COUNT
    parameter integer g_RAW_COUNT    = 9,              // 原始细时间码 raw 的位宽
    parameter integer g_FP_COUNT     = 13,             // 细时间/查表值 LUT 的位宽
    parameter integer g_COARSE_COUNT = 25,             // 粗计数 coarse 的位宽
    parameter integer g_RO_LENGTH    = 31              // 环振长度参数
) (
    input  wire                               clk_i,
    input  wire                               reset_i,
    input  wire [g_COARSE_COUNT-1:0]          coarse_i,    // 外部输入的粗计数值
    input  wire [g_COARSE_COUNT+g_FP_COUNT-1:0] deskew_i,  // 静态 deskew 补偿值，用于通道间对齐
    input  wire                               signal_i,    // 正常待测输入信号
    input  wire                               calib_i,     // 校准输入信号
    input  wire                               calib_sel_i, // 输入选择：1 选 calib_i，0 选 signal_i
    output reg                                detect_o,    // 事件检测输出脉冲
    output wire                               polarity_o,  // 锁存后的边沿极性输出
    output wire [g_RAW_COUNT-1:0]             raw_o,       // 锁存后的原始细时间码输出
    output reg  [g_COARSE_COUNT+g_FP_COUNT-1:0] fp_o,      // 最终 coarse/fine 合成后的时间输出
    input  wire [g_RAW_COUNT-1:0]             lut_a_i,     // LUT 的 B 口地址：供控制器/调试接口访问
    input  wire                               lut_we_i,    // LUT 的 B 口写使能
    input  wire [g_FP_COUNT-1:0]              lut_d_i,     // LUT 的 B 口写数据
    output wire [g_FP_COUNT-1:0]              lut_d_o,     // LUT 的 B 口读数据
    input  wire                               ro_en_i,     // 环振使能输入
    output wire                               ro_clk_o     // 环振输出时钟
);

reg                                calib_sel_d;           // 对 calib_sel_i 打一拍后的版本，降低输入选择切换毛刺风险
wire                               muxed_signal;          // 经过输入多路选择后的信号：signal_i 或 calib_i
wire                               inv_signal;            // 可能经过反相后的输入信号，送入延迟线
wire [4*g_CARRY4_COUNT-1:0]        taps;                  // 延迟线采样后的 tap 向量
wire                               ipolarity;             // 编码器内部极性状态，用于控制是否反相输入
wire                               polarity;              // 编码器输出的当前极性状态
reg                                polarity_d1;           // polarity 的第一级寄存
reg                                polarity_d2;           // polarity 的第二级寄存，用于形成 detect
wire                               detect_d1;             // 由 polarity 翻转得到的一拍检测脉冲
wire [g_RAW_COUNT-1:0]             raw;                   // 编码器当前输出的原始细时间码
reg  [g_RAW_COUNT-1:0]             raw_d1;                // raw 的第一级寄存
reg  [g_RAW_COUNT-1:0]             raw_d2;                // 在 detect 时锁存后的 raw
wire [g_FP_COUNT-1:0]              lut;                   // 从 LUT A 口按 raw 查到的细时间修正值
wire                               ro_en;                 // 实际送入环振的使能信号
wire [g_COARSE_COUNT+g_FP_COUNT-1:0] coarse_ext;          // 对 coarse 扩展后的总位宽版本
wire [g_COARSE_COUNT+g_FP_COUNT-1:0] lut_ext;             // 对 lut 扩展后的总位宽版本

// 对校准选择信号打一拍，避免直接切换数据源时产生毛刺。
always @(posedge clk_i) begin
    calib_sel_d <= calib_sel_i;
end

assign muxed_signal = calib_sel_d ? calib_i : signal_i;
// ipolarity 由编码器给出，用于在不同极性之间统一成“前导 1”问题。
// 这里的做法是：根据当前内部极性状态，对输入做条件反相，
// 从而让后面的 delayline + lbc 始终尽量面对一致形式的编码问题。
// 根据 ipolarity 决定是否对输入翻转
/*
但真实输入 signal_i 可能既有上升沿，也有下降沿。
如果你什么都不做，那么：
有一类边沿会很好测
另一类边沿就不一定能用同一套逻辑自然地编码
*/
assign inv_signal   = muxed_signal ^ (~ipolarity);

// 实例化延迟线模块
tdc_delayline #(
    .g_WIDTH(g_CARRY4_COUNT)
) cmp_delayline (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .signal_i(inv_signal),
    .taps_o(taps)
);

// 实例化前导位计数/编码器模块
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
    .g_data_width(g_FP_COUNT),                            // RAM 数据宽度 = LUT 项位宽
    .g_size(2**g_RAW_COUNT),                              // RAM 深度 = raw 码空间大小
    .g_with_byte_enable(0),                               // 不使用 byte enable
    .g_addr_conflict_resolution("read_first"),            // 地址冲突时采用 read_first 策略
    .g_init_file(""),                                     // 不使用初始化文件
    .g_dual_clock(0)                                      // 单时钟双口模式
) cmp_lut (
    .clka_i(clk_i),
    .clkb_i(1'b0),
    .wea_i(1'b0),                                         // A 口不写，只读
    .bwea_i(1'b0),                                        // A 口不使用字节写使能
    .aa_i(raw),                                           // A 口地址来自当前 raw
    .da_i({g_FP_COUNT{1'b0}}),                            // A 口写数据无效，填 0 占位
    .qa_o(lut),                                           // A 口读出当前 raw 对应的 LUT 值
    .web_i(lut_we_i),                                     // B 口写使能由外部控制器给出
    .bweb_i(1'b0),                                        // B 口不使用字节写使能
    .ab_i(lut_a_i),                                       // B 口地址由外部控制器/调试接口给出
    .db_i(lut_d_i),                                       // B 口写数据由外部控制器给出
    .qb_o(lut_d_o)                                        // B 口读数据输出给外部
);

tdc_ringosc #(
    .g_LENGTH(g_RO_LENGTH)
) cmp_ringosc (
    .en_i(ro_en),
    .clk_o(ro_clk_o)
);

assign ro_en     = ro_en_i & ~reset_i; // 复位期间关闭环振，非复位时由 ro_en_i 控制
assign detect_d1 = polarity_d1 ^ polarity_d2; // polarity 前后两拍不同时，认为检测到一个新事件
assign polarity_o = polarity_d2; // 输出锁存后的极性状态
assign raw_o      = raw_d2; // 输出锁存后的原始码
assign coarse_ext = {coarse_i, {g_FP_COUNT{1'b0}}}; // 将 coarse 左移 g_FP_COUNT 位，与 fine 对齐
assign lut_ext    = {{g_COARSE_COUNT{1'b0}}, lut}; // 将 LUT 细时间值扩展到与 coarse_ext 相同位宽

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
    	// 这个事件距离当前采样边界还差多少 / 或者距离当前采样边界之前有多少细分偏移, 所以要从 coarse 边界往回减。deskew_i 只表示通道固定修正。
    	// “一个 cycle 之间的变化”本质上是两个时间戳之差： T_period = fp_o[n] - fp_o[n-1]
        fp_o <= coarse_ext - lut_ext + deskew_i;
    end
end

endmodule
