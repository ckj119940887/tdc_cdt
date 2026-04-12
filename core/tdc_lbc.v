// -----------------------------------------------------------------------------
// Leading-bit counter
// -----------------------------------------------------------------------------
// 功能:
// - 把延迟线 tap 编码成原始码 raw,把延迟线采样结果转换为原始细时间码
// - 统计从 MSB 开始连续的 1 的数量,从最高位开始统计连续为 1 的位数
// - 根据最高位翻转情况维护当前极性 polarity,根据最高位事件来更新极性状态
// -----------------------------------------------------------------------------
module tdc_lbc #(
    parameter integer g_N      = 9, // 参数 g_N：输出计数值 count_o 的位宽，默认 9 位
    parameter integer g_NIN    = 496, // 参数 g_NIN：输入 tap 向量 d_i 的位宽，默认 496 位
    parameter integer g_IGNORE = 2 // 参数 g_IGNORE：极性翻转后忽略后续若干周期的长度，默认 2
) (
    input  wire                 clk_i,
    input  wire                 reset_i,
    input  wire [g_NIN-1:0]     d_i, // 输入 tap 向量，来自延迟线采样结果
    output wire                 ipolarity_o, // 输出内部极性当前值
    output wire                 polarity_o, // 输出延迟一拍后的极性值，便于与延迟后的 count 对齐
    output wire [g_N-1:0]       count_o // 输出延迟一拍后的前导 1 计数值
);

// 局部常量：补齐后的比较向量宽度，通常等于最大可编码原始码范围
localparam integer DCOMP_W = (2**g_N) - 1;

// 定义函数 f_cls，用于计算从 MSB 开始连续 1 的个数
function integer f_cls;
    input [DCOMP_W-1:0] d; // 函数输入 d：待统计的位向量
    integer k;
    integer cnt; // 计数变量 cnt：记录连续 1 的数量
    reg stop_flag; // 停止标志：遇到第一个 0 后停止继续累计
    begin
        cnt = 0;
        stop_flag = 1'b0;
        for (k = DCOMP_W-1; k >= 0; k = k - 1) begin
            if (!stop_flag) begin
                if (d[k])
                    cnt = cnt + 1; // 则连续 1 的数量加 1
                else
                    stop_flag = 1'b1; // 则置停止标志，后续不再计数
            end
        end
        f_cls = cnt; // 将统计得到的连续 1 个数作为函数返回值
    end
endfunction

reg                    polarity; // 寄存器：保存当前内部极性状态
reg                    polarity_d1; // 寄存器：保存延迟一拍后的极性状态，用于输出对齐
reg  [g_N-1:0]         count; // 寄存器：保存当前拍计算出的前导 1 计数
reg  [g_N-1:0]         count_d1; // 寄存器：保存延迟一拍后的计数结果，用于输出对齐
wire [DCOMP_W-1:0]     d_completed; // 线网：补齐到 DCOMP_W 位宽后的输入向量
wire                   ignore; // 线网：忽略标志，高电平时禁止再次触发极性翻转

// 根据输入宽度决定是否补零扩展
generate
    if (g_NIN < DCOMP_W) begin : g_expand
        assign d_completed = {d_i, {(DCOMP_W-g_NIN){1'b0}}};
    end else begin : g_dontexpand
        assign d_completed = d_i;
    end
endgenerate

/*
polarity 代表当前 TDC 通道处于哪一种“相位状态”。
每次检测到一次完整有效事件后，它翻转一次。

这个翻转有两个用途：
让后级通过“前后是否变化”产生 detect
让输入路径根据当前状态决定是否反相，从而把不同方向的边沿统一成同一种编码形式

它是一个随着有效事件交替翻转的内部状态位。
*/
always @(posedge clk_i) begin
    if (reset_i) begin
        polarity    <= 1'b1;
        polarity_d1 <= 1'b1;
        count       <= {g_N{1'b0}};
        count_d1    <= {g_N{1'b0}};
    end else begin
        if (d_completed[DCOMP_W-1] && !ignore) // 当延迟线编码结果的最高位变成 1，而且当前不在忽略窗口里，就把 polarity 翻转一次。
            polarity <= ~polarity;
        polarity_d1 <= ~polarity;
        count       <= f_cls(d_completed);
        count_d1    <= count;
    end
end

/*
ignore 是给 polarity 配的“保护机制”。
如果没有 ignore，会发生一个很严重的问题：
假设某次事件来了，导致 d_completed[DCOMP_W-1] = 1

如果这个最高位不是只高一个时钟，而是连续高了两三个时钟，那么这段代码：
if (d_completed[DCOMP_W-1])
    polarity <= ~polarity;
    
就会在每个时钟都翻一次 polarity：
第 1 拍翻一次
第 2 拍又翻回来
第 3 拍再翻一次
这样就等于把同一次物理事件错误地当成了多次事件。    

后果就是：
detect 会冒出多个脉冲
raw 对齐会乱
后面 LUT 查表和时间戳都会错

ignore 怎么避免这个问题
也就是说：
第一次看到最高位变 1，允许翻转
同时开启一个 ignore 窗口
在这个窗口持续期间，即使最高位还保持 1，也不允许再次翻转
所以 ignore 本质上就是一个短死区或防重触发窗口。
*/
generate
    if (g_IGNORE > 1) begin : g_ignoresr_many // 情况1：需要忽略多个周期
        reg [g_IGNORE-1:0] ignore_sr; // 移位寄存器：用来维持一个长度为 g_IGNORE 的忽略窗口
        always @(posedge clk_i) begin
            if (reset_i) begin
                ignore_sr <= {g_IGNORE{1'b0}};
            end else begin
                if (d_completed[DCOMP_W-1] && !ignore) begin // 如果最高位触发且当前不在忽略窗口中
                    ignore_sr <= {1'b1, {(g_IGNORE-1){1'b0}}}; // 则装载一个新的忽略窗口：最高位置 1，其余位置 0
                end else begin
                    ignore_sr <= {1'b0, ignore_sr[g_IGNORE-1:1]};
                end
            end
        end
        assign ignore = (ignore_sr == {g_IGNORE{1'b0}}) ? 1'b0 : 1'b1;
    end else if (g_IGNORE == 1) begin : g_ignoresr_one
        reg ignore_sr;
        always @(posedge clk_i) begin
            if (reset_i)
                ignore_sr <= 1'b0;
            else if (d_completed[DCOMP_W-1] && !ignore) 
                ignore_sr <= 1'b1;
            else
                ignore_sr <= 1'b0;
        end
        assign ignore = ignore_sr;
    end else begin : g_noignore
        assign ignore = 1'b0;
    end
endgenerate

assign ipolarity_o = polarity;
assign polarity_o  = polarity_d1;
assign count_o     = count_d1;

endmodule
