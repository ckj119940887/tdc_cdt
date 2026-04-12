// -----------------------------------------------------------------------------
// TDC controller
// -----------------------------------------------------------------------------
// 控制器负责:
//   1) 上电启动标定
//   2) histogram 清零/累计
//   3) 保存各通道基准频率 F0
//   4) 在线标定时计算 LUT = accumulated_histogram * sfreq / freq
//   5) 冻结模式下把控制权交给调试接口
// -----------------------------------------------------------------------------
module tdc_controller #(
    parameter integer g_RAW_COUNT      = 9, // raw 编码位宽，也决定 histogram/LUT 地址位宽
    parameter integer g_FP_COUNT       = 13, // 最终 fine-time / LUT 数据位宽
    parameter integer g_EXHIS_COUNT    = 4, // histogram 额外扩展位宽，用于防止统计计数溢出
    parameter integer g_FCOUNTER_WIDTH = 13 // 频率计数器输出位宽
) (
    input  wire                               clk_i,
    input  wire                               reset_i,
    output reg                                ready_o, // TDC 控制器整体 ready 标志，一旦校准完成后置 1
    output reg                                next_o, // 切换到下一个通道的脉冲输出
    input  wire                               last_i, // 当前通道是否是最后一个通道
    output reg                                calib_sel_o, // 校准输入选择控制：1 表示用校准信号，0 表示正常信号
    output wire [g_RAW_COUNT-1:0]             lut_a_o, // LUT RAM 地址输出
    output reg                                lut_we_o, // LUT RAM 写使能
    output wire [g_FP_COUNT-1:0]              lut_d_o, // LUT RAM 写数据
    input  wire                               c_detect_i, // 当前通道检测到一次校准事件
    input  wire [g_RAW_COUNT-1:0]             c_raw_i, // 当前校准事件的 raw 编码
    output wire [g_RAW_COUNT-1:0]             his_a_o, // histogram RAM 地址输出
    output reg                                his_we_o, // histogram RAM 写使能
    output wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_o, // histogram RAM 写数据
    input  wire [g_FP_COUNT+g_EXHIS_COUNT-1:0] his_d_i, // histogram RAM 读数据
    output reg                                oc_start_o, // 启动 ring-osc 频率测量
    input  wire                               oc_ready_i, // ring-osc 频率测量完成 / 模块空闲
    input  wire [g_FCOUNTER_WIDTH-1:0]        oc_freq_i, // 当前测得的 ring-osc 频率
    output reg                                oc_store_o, // 将当前频率保存为参考频率 sfreq
    input  wire [g_FCOUNTER_WIDTH-1:0]        oc_sfreq_i, // 先前保存的参考频率 sfreq
    input  wire                               freeze_req_i, // 冻结请求，进入手动调试/冻结模式
    output reg                                freeze_ack_o // 冻结应答，表示当前已进入冻结模式
);

localparam integer HC_W = g_FP_COUNT + g_EXHIS_COUNT; // histogram 计数宽度 = fine-time 位宽 + 扩展位宽
localparam integer DIV_W = g_FP_COUNT + g_FCOUNTER_WIDTH; // 除法器宽度 = LUT 位宽 + 频率位宽

localparam [3:0]           // 主状态机状态编码，4 bit 足够表示所有状态
    SC_NEWCHANNEL = 4'd0,  // Startup Calibration: 切换到新通道
    SC_CLEARHIST  = 4'd1,  // Startup Calibration: 清空 histogram
    SC_READ       = 4'd2,  // Startup Calibration: 等待校准事件
    SC_UPDATE     = 4'd3,  // Startup Calibration: histogram 对应 bin 加 1
    SC_STOREF0    = 4'd4,  // Startup Calibration: 记录当前通道参考频率 F0
    OC_STARTM     = 4'd5,  // Online Calibration: 启动当前通道 ring-osc 频率测量
    OC_WAITM      = 4'd6,  // Online Calibration: 等待频率测量完成
    OC_WAITMUL1   = 4'd7,  // Online Calibration: 乘法流水第 1 等待拍
    OC_WAITMUL2   = 4'd8,  // Online Calibration: 乘法流水第 2 等待拍
    OC_STARTDIV   = 4'd9,  // Online Calibration: 启动除法器
    OC_WAITDIV    = 4'd10, // Online Calibration: 等待除法器完成
    OC_WRITELUT   = 4'd11, // Online Calibration: 把除法结果写回 LUT，并推进 histogram 地址
    OC_NEXTCHANNEL= 4'd12, // Online Calibration: 切换到下一个通道
    FREEZE        = 4'd13; // 冻结模式：停止自动流程，把内部控制权交给外部

reg                                ready_p;       // ready_o 的脉冲置位信号；一旦拉高，ready_o 保持为 1
reg  [HC_W-1:0]                    hc_count;      // startup calibration 阶段剩余要记录的 histogram 事件个数
reg                                hc_reset;      // hc_count 清零/装载控制
reg                                hc_dec;        // hc_count 减 1 控制
wire                               hc_zero;       // hc_count 是否已经减到 0
reg  [g_RAW_COUNT-1:0]             ha_count;      // histogram / LUT 地址计数器
reg                                ha_reset;      // ha_count 复位到 0
reg                                ha_inc;        // ha_count 自增
wire                               ha_last;       // ha_count 是否已到最后一个地址
reg                                ha_sel;        // histogram 地址选择：1 用 ha_count，0 用 c_raw_i
reg  [HC_W-1:0]                    acc;           // histogram 累加器，在线标定时计算累计直方图
reg                                acc_reset;     // acc 清零控制
reg                                acc_en;        // acc 累加使能
reg  [DIV_W-1:0]                   mul;           // 乘法结果寄存器：acc * sfreq
reg  [DIV_W-1:0]                   mul_d1;        // mul 的一拍延迟，用于对齐除法器输入
reg                                div_start;     // 启动除法器脉冲
wire                               div_ready;     // 除法器完成标志
wire [DIV_W-1:0]                   div_divisor;   // 除法器除数输入
wire [DIV_W-1:0]                   div_quotient;  // 除法器商输出
reg  [g_FP_COUNT-1:0]              div_qsat;      // 除法结果饱和裁剪后的 LUT 写数据
reg  [3:0]                         state;         // 主状态机当前状态

// ready_o 一旦被置位，保持为 1，直到复位。
always @(posedge clk_i) begin
    if (reset_i)
        ready_o <= 1'b0;
    else if (ready_p)
        ready_o <= 1'b1;
end

// 启动标定期间，需要统计还要记录多少个 histogram 事件。
always @(posedge clk_i) begin
    if (hc_reset)
        hc_count <= {HC_W{1'b1}};
    else if (hc_dec)
        hc_count <= hc_count - 1'b1;
end
assign hc_zero = (hc_count == {HC_W{1'b0}});

// histogram 地址计数器：用于清 RAM / 回读直方图 / 写 LUT。
always @(posedge clk_i) begin
    if (ha_reset)
        ha_count <= {g_RAW_COUNT{1'b0}};
    else if (ha_inc)
        ha_count <= ha_count + 1'b1;
end
assign ha_last = &ha_count;
assign his_a_o = ha_sel ? ha_count : c_raw_i;
assign his_d_o = ha_sel ? {HC_W{1'b0}} : (his_d_i + 1'b1);

// 在线标定时，对直方图做累加。
always @(posedge clk_i) begin
    if (acc_reset)
        acc <= {HC_W{1'b0}};
    else if (acc_en)
        acc <= acc + his_d_i;
end

// 乘法阶段: accumulated_histogram * saved_frequency。
always @(posedge clk_i) begin
    mul    <= acc[HC_W-1:g_EXHIS_COUNT] * oc_sfreq_i;
    mul_d1 <= mul;
end

// 除法阶段: (acc * sfreq) / freq。
tdc_divider #(
    .g_WIDTH(DIV_W)
) cmp_divider (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .start_i(div_start),
    .dividend_i(mul_d1),
    .divisor_i(div_divisor),
    .ready_o(div_ready),
    .quotient_o(div_quotient),
    .remainder_o()
);

assign div_divisor = {{g_FP_COUNT{1'b0}}, oc_freq_i};

always @* begin
    if (div_quotient[DIV_W-1:g_FP_COUNT] == {g_FCOUNTER_WIDTH{1'b0}})
        div_qsat = div_quotient[g_FP_COUNT-1:0];
    else
        div_qsat = {g_FP_COUNT{1'b1}};
end

assign lut_a_o = ha_count;
assign lut_d_o = div_qsat;

// 主状态机。
always @(posedge clk_i) begin
    if (reset_i) begin
        if (freeze_req_i) // 若复位时外部就请求冻结
            state <= FREEZE; // 则直接进入冻结模式
        else
            state <= SC_NEWCHANNEL; // 从 startup calibration 的新通道状态开始
    end else begin
        case (state)
            SC_NEWCHANNEL: begin // 启动标定：切换到一个新通道后
                state <= SC_CLEARHIST; // 下一步先清空该通道对应 histogram
            end
            SC_CLEARHIST: begin // 启动标定：逐地址清 histogram RAM
                if (ha_last) // 如果已经清到最后一个地址
                    state <= SC_READ; // 则进入等待校准事件状态
            end
            SC_READ: begin // 启动标定：等待当前通道产生校准事件
                if (c_detect_i) // 一旦检测到一次事件
                    state <= SC_UPDATE; // 则转到 histogram bin 更新状态
            end
            SC_UPDATE: begin // 启动标定：把对应 raw bin 计数加 1
                if (hc_zero) // 若事件计数器已减到 0，表示采样足够多
                    state <= SC_STOREF0; // 则转到存储参考频率状态
                else // 否则 histogram 采样还未结束
                    state <= SC_READ; // 返回继续等待下一次校准事件
            end
            SC_STOREF0: begin // 启动标定：记录当前通道参考频率 F0
                if (oc_ready_i) begin // 等 ring-osc 测频模块完成本次测量
                    if (last_i) // 若当前已是最后一个通道
                        state <= OC_STARTM; // 则 startup calibration 全部完成，转入在线标定
                    else // 否则还没轮完所有通道
                        state <= SC_NEWCHANNEL; // 切到下一个通道继续 startup calibration
                end
            end
            OC_STARTM:     state <= OC_WAITM; // 在线标定：发起测频后，下一拍进入等待测频完成
            OC_WAITM:      if (oc_ready_i) state <= OC_WAITMUL1; // 在线标定：测频完成后开始等待乘法流水结果
            OC_WAITMUL1:   state <= OC_WAITMUL2; // 在线标定：乘法流水第 1 拍
            OC_WAITMUL2:   state <= OC_STARTDIV; // 在线标定：乘法流水第 2 拍后启动除法
            OC_STARTDIV:   state <= OC_WAITDIV; // 在线标定：启动除法器后等待除法完成
            OC_WAITDIV:    if (div_ready) state <= OC_WRITELUT; // 在线标定：除法完成后进入写 LUT 状态
            OC_WRITELUT: begin // 在线标定：把当前地址对应 LUT 写入
                if (ha_last) // 若已写到 LUT 最后一个地址
                    state <= OC_NEXTCHANNEL; // 则本通道 LUT 更新结束，准备切换通道
                else // 否则 LUT 还没写完
                    state <= OC_WAITMUL1; // 继续处理下一个 histogram/LUT 地址
            end
            OC_NEXTCHANNEL: begin // 在线标定：切换到下一个通道
                if (freeze_req_i) // 若此时有冻结请求
                    state <= FREEZE; // 则进入冻结模式
                else // 否则继续自动在线标定
                    state <= OC_STARTM; // 启动下一个通道的新一轮测频与 LUT 更新
            end
            FREEZE: begin
                if (!freeze_req_i)
                    state <= OC_STARTM;
            end
            default: state <= SC_NEWCHANNEL;
        endcase
    end
end

// 输出控制逻辑。
always @* begin
    ready_p      = 1'b0;                           // 默认不置位 ready_o
    hc_reset     = 1'b0;                           // 默认不重装 histogram 事件计数器
    hc_dec       = 1'b0;                           // 默认不减少事件计数器
    ha_reset     = 1'b0;                           // 默认不复位 histogram/LUT 地址计数器
    ha_inc       = 1'b0;                           // 默认不递增地址计数器
    ha_sel       = 1'b0;                           // 默认 histogram 地址选择 c_raw_i
    acc_reset    = 1'b0;                           // 默认不清零累计器
    acc_en       = 1'b0;                           // 默认不做累计 histogram 加法
    div_start    = 1'b0;                           // 默认不启动除法器
    next_o       = 1'b0;                           // 默认不切换通道
    calib_sel_o  = 1'b0;                           // 默认选择正常输入而非校准输入
    lut_we_o     = 1'b0;                           // 默认不写 LUT
    his_we_o     = 1'b0;                           // 默认不写 histogram
    oc_start_o   = 1'b0;                           // 默认不启动 ring-osc 测频
    oc_store_o   = 1'b0;                           // 默认不存储参考频率
    freeze_ack_o = 1'b0;                           // 默认不应答冻结

    case (state)                                   // 根据当前状态产生相应控制输出
        SC_NEWCHANNEL: begin                       // 刚切换到新通道
            hc_reset <= 1'b1;                      // 重装 startup histogram 事件计数器
            ha_reset <= 1'b1;                      // 地址计数器回到 0，为清 histogram 做准备
        end                                        // SC_NEWCHANNEL 输出结束
        SC_CLEARHIST: begin                        // 清空当前通道 histogram RAM
            calib_sel_o <= 1'b1;                   // 使用校准输入，而非正常测量输入
            ha_inc      <= 1'b1;                   // 每拍递增 histogram 地址
            ha_sel      <= 1'b1;                   // histogram 地址选择 ha_count
            his_we_o    <= 1'b1;                   // 使能 histogram 写入，且写入值为 0
        end                                        // SC_CLEARHIST 输出结束
        SC_READ: begin                             // 等待一次新的校准事件
            calib_sel_o <= 1'b1;                   // 仍然保持使用校准输入
            if (c_detect_i)                        // 若这拍捕获到一次事件
                hc_dec <= 1'b1;                    // 则将剩余事件计数减 1
        end                                        // SC_READ 输出结束
        SC_UPDATE: begin                           // 对刚检测到的 raw bin 做 histogram +1
            calib_sel_o <= 1'b1;                   // 仍然走校准输入路径
            his_we_o    <= 1'b1;                   // 写 histogram RAM，对应地址为 c_raw_i
            if (hc_zero)                           // 若这次更新后事件数已满足要求
                oc_start_o <= 1'b1;                // 则立即启动一次频率测量，为保存 F0 做准备
        end                                        // SC_UPDATE 输出结束
        SC_STOREF0: begin                          // 等待频率测量完成并保存参考频率
            if (oc_ready_i) begin                  // 若测频模块已经完成
                oc_store_o <= 1'b1;                // 则把当前频率存入 sfreq
                next_o     <= 1'b1;                // 同时给出切换到下一通道的脉冲
            end                                    // SC_STOREF0 条件控制结束
        end                                        // SC_STOREF0 输出结束
        OC_STARTM: begin                           // 在线标定开始：准备更新当前通道整张 LUT
            oc_start_o <= 1'b1;                    // 启动当前通道 ring-osc 测频
            ha_reset   <= 1'b1;                    // 地址计数器复位到 0，准备从第 0 个 histogram bin 开始扫描
            acc_reset  <= 1'b1;                    // 累计 histogram 清零
            ha_sel     <= 1'b1;                    // histogram 地址选择 ha_count
        end                                        // OC_STARTM 输出结束
        OC_WAITM,                                  // 在线标定：等待测频完成
        OC_WAITMUL1,                               // 在线标定：等待乘法流水第 1 拍
        OC_WAITMUL2,                               // 在线标定：等待乘法流水第 2 拍
        OC_WAITDIV: begin                          // 在线标定：等待除法器完成
            ha_sel <= 1'b1;                        // 这些阶段都保持 histogram 地址由 ha_count 驱动
        end                                        // 上述合并状态的输出结束
        OC_STARTDIV: begin                         // 在线标定：启动除法器
            div_start <= 1'b1;                     // 发出除法启动脉冲
            ha_sel    <= 1'b1;                     // 保持 histogram/LUT 地址仍由 ha_count 提供
        end                                        // OC_STARTDIV 输出结束
        OC_WRITELUT: begin                         // 在线标定：写当前地址对应 LUT 项
            lut_we_o <= 1'b1;                      // 使能 LUT 写入，数据来自 div_qsat
            acc_en   <= 1'b1;                      // 同时把当前 histogram bin 加入累计 histogram
            ha_inc   <= 1'b1;                      // 地址加 1，准备处理下一个 bin
            ha_sel   <= 1'b1;                      // histogram 地址保持选择 ha_count
        end                                        // OC_WRITELUT 输出结束
        OC_NEXTCHANNEL: begin                      // 在线标定：当前通道 LUT 已全部更新完成
            next_o <= 1'b1;                        // 发出切换到下一个通道的脉冲
            if (last_i)                            // 如果刚刚处理的是最后一个通道
                ready_p <= 1'b1;                   // 则说明整个系统已完成首轮完整在线校准，置位 ready
            ha_sel <= 1'b1;                        // 保持地址选择为 ha_count
        end                                        // OC_NEXTCHANNEL 输出结束
        FREEZE: begin                              // 冻结模式输出
            freeze_ack_o <= 1'b1;                  // 向外部应答：当前已进入冻结模式
        end                                        // FREEZE 输出结束
        default: begin                             // 默认分支
        end                                        // 默认分支不做额外控制
    endcase                                        // 输出控制 case 结束
end

endmodule
