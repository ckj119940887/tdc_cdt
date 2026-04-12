// -----------------------------------------------------------------------------
// Pulse synchronizer
// -----------------------------------------------------------------------------
// 把源时钟域中的单周期脉冲变换成目的时钟域中的单周期脉冲。
// 实现方式:
//   pulse -> level toggle -> double/triple sync -> xor restore pulse
// -----------------------------------------------------------------------------
module tdc_psync (
    input  wire clk_src_i,
    input  wire p_i,
    input  wire clk_dst_i,
    output wire p_o
);

reg level    = 1'b0;
reg level_d1 = 1'b0;
reg level_d2 = 1'b0;
reg level_d3 = 1'b0;

always @(posedge clk_src_i) begin
    if (p_i)
        level <= ~level;
end

always @(posedge clk_dst_i) begin
    level_d1 <= level;
    level_d2 <= level_d1;
    level_d3 <= level_d2;
end

assign p_o = level_d2 ^ level_d3;

endmodule
