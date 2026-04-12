// -----------------------------------------------------------------------------
// Frequency counter
// -----------------------------------------------------------------------------
// 在 clk_i 域内发起一次测量，跨时钟域通知 clk_m_i 域开始/停止计数，最后把测量
// 结果同步回 clk_i 域输出。
// 
// 用最直白的话说它在干什么
// 假设你想知道一个时钟快不快，但你又不直接算 Hz，而是这样做：
// 先打开一个固定长度的计时窗口
// 在窗口打开期间，统计被测时钟来了多少个周期
// 窗口结束后，把计数值输出
// 所以它的输入输出关系可以理解成：
// clk_i：系统参考时钟，用来控制“测多久”
// clk_m_i：被测时钟，用来统计“跳了多少次”
// start_i：开始测量
// freq_o：测量结果
// ready_o：当前是否空闲、能不能开始下一次测量

// 在这套 tdc_core 里，它不是拿来测外部信号频率的，而是主要用来测：
// 每个通道里的 ring oscillator（环振）频率。
// 
// 原因是：
// 环振频率会随着 PVT 变化而变化
// 延迟链速度也会随着 PVT 变化而变化
// 所以环振频率可以作为“当前通道速度快慢”的代理量
//
// 于是控制器就能用它做在线校准：
// 启动时存一个参考频率 sfreq
// 运行时再测当前频率 freq
// 用 sfreq / freq 去缩放 LUT
//
// 所以这个文件本质上不是 TDC 的“测时间主通路”，而是：
// 给 TDC 校准系统提供“当前速度状态”的辅助测量模块。
// -----------------------------------------------------------------------------
module tdc_freqc #(
    parameter integer g_COUNTER_WIDTH = 13, // 计数器位宽：决定最多能统计多少个 clk_m_i 周期
    parameter integer g_TIMER_WIDTH   = 14 // 计时器位宽：决定测量窗口长度
) (
    input  wire                           clk_i, // 系统控制时钟域：状态机、窗口计时器工作在该时钟域
    input  wire                           reset_i,
    input  wire                           clk_m_i, // 被测时钟输入，频率计数在该时钟域进行
    input  wire                           start_i, // 测量启动请求，在 clk_i 域给出
    output reg                            ready_o, // 模块就绪标志，为 1 表示当前可接受新的 start_i
    output reg  [g_COUNTER_WIDTH-1:0]     freq_o // 输出测得的频率计数值
);

localparam [1:0]
    IDLE        = 2'd0,
    MEASURING   = 2'd1, // 测量态：窗口打开，正在统计 clk_m_i 个数
    TERMINATING = 2'd2; // 结束态：已经发出 stop，等待跨域停止确认

reg  [g_COUNTER_WIDTH-1:0] m_counter; // clk_m_i 域中的实际计数器：统计被测时钟沿数
wire                       m_start; // start 脉冲同步到 clk_m_i 域后的单周期脉冲
wire                       m_stop; // stop 脉冲同步到 clk_m_i 域后的单周期脉冲
reg                        m_started; // clk_m_i 域中的“正在计数”状态标志
reg                        start; // clk_i 域内部产生的启动脉冲，送去跨域同步
reg                        stop; // clk_i 域内部产生的停止脉冲，送去跨域同步
wire                       stop_ack; // m_stop 再同步回 clk_i 域后的确认脉冲
reg  [g_COUNTER_WIDTH-1:0] counter_r; // clk_i 域中的中间寄存器，用于同步 m_counter
reg  [g_TIMER_WIDTH-1:0]   timer; // clk_i 域窗口计时器
reg                        timer_start; // 计时器装载启动信号：为 1 时把 timer 装成全 1
wire                       timer_done; // 计时器结束标志：timer 减到 0 时为 1
reg  [1:0]                 state; // clk_i 域状态机当前状态

// 被测时钟域：计数被测时钟的上升沿数量。
// 这一段 always 在 clk_m_i 域运行。
// 它完成三件事：
// 1) m_start 到来时清零计数器并置位 m_started；
// 2) 只要 m_started=1 且还没收到 m_stop，就每个 clk_m_i 周期加 1；
// 3) m_stop 到来时清除 m_started，停止继续累加。
always @(posedge clk_m_i) begin
    if (m_started && !m_stop)
        m_counter <= m_counter + 1'b1;
    if (m_start) begin
        m_started <= 1'b1;
        m_counter <= {g_COUNTER_WIDTH{1'b0}};
    end
    if (m_stop)
        m_started <= 1'b0;
end

// 开始/停止脉冲跨时钟域同步。
// 由于 start 和 stop 都是在 clk_i 域产生的单周期脉冲，
// 不能直接送入 clk_m_i 域使用，所以这里用 tdc_psync 做脉冲同步。
tdc_psync cmp_sync_start (
    .clk_src_i(clk_i),
    .p_i(start),
    .clk_dst_i(clk_m_i),
    .p_o(m_start)
);

tdc_psync cmp_sync_stop (
    .clk_src_i(clk_i),
    .p_i(stop),
    .clk_dst_i(clk_m_i),
    .p_o(m_stop)
);

tdc_psync cmp_sync_stop_ack (
    .clk_src_i(clk_m_i),
    .p_i(m_stop),
    .clk_dst_i(clk_i),
    .p_o(stop_ack)
);

// 把测得的计数值同步到 clk_i 域。
// 这里采用最简单的两级寄存方式：
// 第一级先把 m_counter 采到 counter_r；
// 第二级再把 counter_r 送到 freq_o。
// 这样能降低跨域直接采样带来的不稳定风险。
always @(posedge clk_i) begin
    counter_r <= m_counter;
    freq_o    <= counter_r;
end

// 窗口计时器：长度为 2^g_TIMER_WIDTH - 1 个 clk_i 周期。
// 该计时器在 clk_i 域工作，用来规定一次测量持续多久。
// 当 timer_start 置位时，timer 装载为全 1；
// 之后每个 clk_i 周期减 1，直到减到 0 为止。
always @(posedge clk_i) begin
    if (timer_start) // 如果要求启动一个新的测量窗口
        timer <= {g_TIMER_WIDTH{1'b1}}; // 把计时器装载为全 1，即最大值
    else if (!timer_done) // 否则如果尚未计时结束
        timer <= timer - 1'b1; // 每拍减 1，倒计时
end
assign timer_done = (timer == {g_TIMER_WIDTH{1'b0}});

// 控制状态机。
// 状态机运行在 clk_i 域，负责协调：
// IDLE -> 等待 start_i
// MEASURING -> 计时窗口内持续测量
// TERMINATING -> 已发 stop，等待 stop_ack 返回
always @(posedge clk_i) begin
    if (reset_i) begin
        state <= IDLE;
    end else begin
        case (state)
            IDLE: begin
                if (start_i)
                    state <= MEASURING;
            end
            MEASURING: begin
                if (timer_done)
                    state <= TERMINATING;
            end
            TERMINATING: begin
                if (stop_ack)
                    state <= IDLE;
            end
            default: state <= IDLE;
        endcase
    end
end

// 状态机输出。
// 这是组合逻辑：根据当前 state 产生 ready_o/start/stop/timer_start。
// 其中：
// - ready_o 表示当前可接受新的 start_i；
// - start   是发往 clk_m_i 域的启动脉冲源；
// - stop    是发往 clk_m_i 域的停止脉冲源；
// - timer_start 用于装载计时窗口。
always @* begin
    ready_o     = 1'b0;
    start       = 1'b0;
    stop        = 1'b0;
    timer_start = 1'b0;
    case (state)
        IDLE: begin
            ready_o = 1'b1;
            if (start_i) begin
                start       = 1'b1;
                timer_start = 1'b1;
            end
        end
        MEASURING: begin
            if (timer_done)
                stop = 1'b1;
        end
        TERMINATING: begin
        end
        default: begin
        end
    endcase
end

endmodule
