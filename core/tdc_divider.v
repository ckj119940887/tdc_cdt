// -----------------------------------------------------------------------------
// Sequential unsigned divider
// -----------------------------------------------------------------------------
// 串行无符号除法器，每次 start_i 拉高后开始工作，ready_o 为 1 表示空闲/完成。
// 用来计算(acc * sfreq) / freq，会作为某个 raw bin 的校准后细时间值写入 LUT；之后正常测量时，raw 通过查这张 LUT 被转换成校准后的 fine time，从而补偿 bin 宽度不均匀和 PVT 漂移。
/*
你可以把这条式子先记成：
LUT[k] = (acc[k] * sfreq) / freq
它的意思不是单纯做数学运算，而是把三类信息合在一起：
acc[k]：这个 bin 在“参考统计”下累积到哪里
sfreq：参考状态下这条链有多快
freq：当前状态下这条链有多快

acc 是 accumulated histogram，也就是“累计 histogram”。
先有：hist[0], hist[1], hist[2], ...
其中 hist[k] 表示：raw = k 这个 bin 被打中了多少次

然后控制器按地址从小到大累加：
acc[0] = hist[0]
acc[1] = hist[0] + hist[1]
acc[2] = hist[0] + hist[1] + hist[2]
从 bin 0 走到 bin k 为止，一共累计了多少“宽度”。
hist[k] 近似反映 第 k 个 bin 的时间宽度
acc[k] 近似反映 从起点到第 k 个 bin 的累计时间位置

sfreq 是 saved frequency，也就是：
启动校准完成时，存下来的参考环振频率
这条延迟链在“基准状态”下的速度标尺

为什么要存这个值，因为后面环境会变：
温度变化
电压变化
工艺漂移
一旦变了，延迟链整体快慢也会变。
所以你必须记住：“我最初建参考表的时候，它的速度是多少”，这个速度就是 sfreq。

freq 是：当前时刻重新测到的 ring oscillator 频率
也就是系统在线运行时，再去测一次这个通道现在有多快。
它的物理意义是：当前状态下这条延迟链的速度代理量
*/
/*
情况 1：当前比参考慢，如果：freq < sfreq，说明现在环振变慢了。通常也意味着延迟链每一级延迟变大了。
sfreq / freq > 1, LUT[k] = acc[k] * (sfreq / freq)会变大。

情况 2：当前比参考快，如果：freq > sfreq,说明现在环振变快了。通常也意味着延迟链每一级延迟变小了。
sfreq / freq < 1, 于是 LUT 会变小。

(acc * sfreq) / freq 不是随便做的一个比例换算，而是在做这件事：把“参考状态下由 histogram 累积得到的时间坐标 acc”缩放成“当前状态下的时间坐标”。
*/
// -----------------------------------------------------------------------------
module tdc_divider #(
    parameter integer g_WIDTH = 16
) (
    input  wire                     clk_i,
    input  wire                     reset_i,
    input  wire                     start_i,
    input  wire [g_WIDTH-1:0]       dividend_i,
    input  wire [g_WIDTH-1:0]       divisor_i,
    output wire                     ready_o,
    output wire [g_WIDTH-1:0]       quotient_o,
    output wire [g_WIDTH-1:0]       remainder_o
);

function integer f_log2_size;
    input a;
    integer a;
    integer i;
    begin
        f_log2_size = 63;
        for (i = 1; i <= 64; i = i + 1) begin
            if ((2**i) >= a) begin
                f_log2_size = i;
                i = 65;
            end
        end
    end
endfunction

localparam integer COUNTER_W = f_log2_size(g_WIDTH+1);

reg  [2*g_WIDTH-1:0] qr;
reg  [COUNTER_W-1:0] counter;
reg  [g_WIDTH-1:0]   divisor_r;
wire [g_WIDTH:0]     diff;
wire                 ready;

assign quotient_o  = qr[g_WIDTH-1:0];
assign remainder_o = qr[2*g_WIDTH-1:g_WIDTH];
assign ready       = (counter == {COUNTER_W{1'b0}});
assign ready_o     = ready;
assign diff        = qr[2*g_WIDTH-1:g_WIDTH-1] - {1'b0, divisor_r};

always @(posedge clk_i) begin
    if (reset_i) begin
        qr        <= {2*g_WIDTH{1'b0}};
        counter   <= {COUNTER_W{1'b0}};
        divisor_r <= {g_WIDTH{1'b0}};
    end else begin
        if (start_i) begin
            counter   <= g_WIDTH;
            qr        <= {{g_WIDTH{1'b0}}, dividend_i};
            divisor_r <= divisor_i;
        end else if (!ready) begin
            if (diff[g_WIDTH])
                qr <= {qr[2*g_WIDTH-2:0], 1'b0};
            else
                qr <= {diff[g_WIDTH-1:0], qr[g_WIDTH-2:0], 1'b1};
            counter <= counter - 1'b1;
        end
    end
end

endmodule
