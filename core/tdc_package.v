// -----------------------------------------------------------------------------
// TDC Core / CERN
// -----------------------------------------------------------------------------
// 文件名: tdc_package.v
//
// 说明:
// 原始 VHDL 文件 tdc_package.vhd 仅用于声明 component，便于其它 VHDL 文件实例化。
// 在 Verilog 中，不需要单独的 component/package 声明文件即可实例化模块，因此这里
// 保留一个注释文件，作为接口索引与迁移说明。
//
// 已转换模块:
//   - tdc
//   - tdc_controller
//   - tdc_channelbank
//   - tdc_channelbank_single
//   - tdc_channelbank_multi
//   - tdc_channel
//   - tdc_freqc
//   - tdc_ringosc
//   - tdc_lbc
//   - tdc_delayline
//   - tdc_ordertaps
//   - tdc_divider
//   - tdc_psync
//
// 注意:
// 1. 本文件不定义任何 module。
// 2. 原工程中的 generic_spram / generic_dpram 以及 Xilinx 原语
//    (CARRY4/LUT1/LUT2/FDR) 仍然作为外部依赖保留。
// -----------------------------------------------------------------------------
