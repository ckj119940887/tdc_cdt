// -----------------------------------------------------------------------------
// Ring oscillator based on Xilinx LUT primitives
// -----------------------------------------------------------------------------
// 说明:
// - 第一个 LUT 除了反相外，还带使能控制，用于把振荡器强制拉到 0
// - 后续 LUT 全部作为反相器使用
// - 该结构本质上是组合反馈环，主要用于 FPGA 上的物理实现
// -----------------------------------------------------------------------------
module tdc_ringosc #(
    parameter integer g_LENGTH = 31
) (
    input  wire en_i,
    output wire clk_o
);

(* DONT_TOUCH = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) 
wire [g_LENGTH:0] s;

/*
LUT1 有 1 个输入（I0），这意味着它有 2^1 = 2 种输入状态：0 和 1。
在 Xilinx 架构中，INIT 值的位顺序是从高地址到低地址排列的（即 INIT[1] 对应输入为 1，INIT[0] 对应输入为 0）。
对于 INIT(2'b01)：当输入 I0 = 0 时，输出为 INIT[0]，即 1。当输入 I0 = 1 时，输出为 INIT[1]，即 0。
*/

/*
LUT2 有 2 个输入（I1, I0），总共有 2^2 = 4 种组合。
在你的代码中，I1 接的是使能信号 en_i，I0 接的是反馈信号 s[i]。
我们来看 INIT(4'b0100) 的真值表映射（注意二进制位是从高到低：INIT[3]、INIT[2]、INIT[1]、INIT[0]）：
输入 I1 (en_i)	输入 I0 (s[i])	对应 INIT 索引	输出 (INIT[n])
0	0	0	0
0	1	1	0
1	0	2	1
1	1	3	0
*/

genvar i;
generate
    for (i = 0; i < g_LENGTH; i = i + 1) begin : g_luts
        if (i == 0) begin : g_firstlut
            LUT2 #(
                .INIT(4'b0100)
            ) cmp_LUT (
                .I0(s[i]),
                .I1(en_i),
                .O(s[i+1])
            );
        end else begin : g_nextlut
            LUT1 #(
                .INIT(2'b01)
            ) cmp_LUT (
                .I0(s[i]),
                .O(s[i+1])
            );
        end
    end
endgenerate

assign s[0] = s[g_LENGTH];
assign clk_o = s[g_LENGTH];

endmodule
