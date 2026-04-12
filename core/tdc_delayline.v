// -----------------------------------------------------------------------------
// Delay line based on Xilinx CARRY4 primitives
// -----------------------------------------------------------------------------
// 说明:
// - 使用 CARRY4 构建延迟链
// - 每个 tap 经两级 FDR 打拍，以降低亚稳态影响
// - 最后再依据静态时序模型对 tap 重新排序
// -----------------------------------------------------------------------------
module tdc_delayline #(
    parameter integer g_WIDTH = 124
) (
    input  wire                    clk_i,
    input  wire                    reset_i,
    input  wire                    signal_i, // 输入信号：待注入延迟链的边沿信号
    output wire [4*g_WIDTH-1:0]    taps_o // 输出 tap 向量：重排并翻转方向后的最终延迟链采样结果
);

wire [4*g_WIDTH-1:0] unreg_rev; // 原始 tap 向量：直接来自 CARRY4 的 CO 输出，尚未寄存
wire [4*g_WIDTH-1:0] reg1_rev; // 第一级寄存后的 tap 向量
wire [4*g_WIDTH-1:0] taps_rev; // 第二级寄存后的 tap 向量，后续送去做顺序重排
wire [4*g_WIDTH-1:0] taps_rev_sorted; // 根据静态时序重排后的 tap 向量

genvar i;
generate
    for (i = 0; i < g_WIDTH; i = i + 1) begin : g_carry4
        if (i == 0) begin : g_firstcarry4 // 对第 1 个 CARRY4 单独处理，因为它没有前级 CI
            CARRY4 cmp_CARRY4 (
                .CO(unreg_rev[3:0]),
                .CI(1'b0),
                .CYINIT(signal_i),
                .DI(4'b0000),
                .S(4'b1111)
            );
        end else begin : g_nextcarry4
            CARRY4 cmp_CARRY4 (
                .CO(unreg_rev[4*(i+1)-1 -: 4]), // 该 CARRY4 的 4 个 CO 输出接到对应的 4-bit 切片
                .CI(unreg_rev[4*i-1]), // 该级的 CI 来自前一个 CARRY4 的最高位 CO，实现级联传播
                .CYINIT(1'b0), // 后续级不再使用 CYINIT 注入，因此固定为 0
                .DI(4'b0000), // DI 仍然全 0
                .S(4'b1111) // S 仍然全 1，保持进位链传播模式
            );
        end
    end
endgenerate

/*
FDR就是D Flip-Flop
C：时钟
R：复位
D：输入数据
Q：输出数据

行为就是：
每个时钟上升沿，把 D 采到 Q
如果 R=1，则在时钟上升沿把 Q 清 0
不是 R 一来立刻清零，而是要等时钟沿到来才清零。
*/
genvar j;
generate
    for (j = 0; j < 4*g_WIDTH; j = j + 1) begin : g_fd
        FDR #(
            .INIT(1'b0)
        ) cmp_FDR_1 (
            .C(clk_i),
            .R(reset_i),
            .D(unreg_rev[j]),
            .Q(reg1_rev[j])
        );

        FDR #(
            .INIT(1'b0)
        ) cmp_FDR_2 (
            .C(clk_i),
            .R(reset_i),
            .D(reg1_rev[j]),
            .Q(taps_rev[j])
        );
    end
endgenerate

tdc_ordertaps #(
    .g_WIDTH(g_WIDTH)
) cmp_ordertaps (
    .unsorted_i(taps_rev),
    .sorted_o(taps_rev_sorted)
);

// 重新排列输出，使“最小延迟 tap”位于最高位。
genvar k;
generate
    for (k = 0; k < 4*g_WIDTH; k = k + 1) begin : g_bit_reverse
        assign taps_o[k] = taps_rev_sorted[(4*g_WIDTH-1)-k];
    end
endgenerate

endmodule
