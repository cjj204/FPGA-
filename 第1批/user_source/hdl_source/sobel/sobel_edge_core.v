//=============================================================================
// 子模块2：Sobel 边缘检测核心
//   输入：8bit 灰度流
//   输出：12bit 梯度幅值 g_sum（|Gx|+|Gy|）及有效标志
//   内部：2 行缓存 + 3x3 窗口 + Sobel 卷积
//
//   [移植改动 2026-10] 增加参数 H_TOTAL
//     H_RES   = 一行"有效像素"数（数据宽度）
//     H_TOTAL = 一行"总时钟数"（含行消隐）
//   当 vid_active_video 恒为 1（连续流）时，行缓存深度必须等于 H_TOTAL，
//   否则 3x3 窗口会跨到错误的行。原始设计按 H_RES 深度，仅在"无消隐的
//   连续像素流"下成立。
//
//   [改动 2026-10-15] 引出 3x3 窗口中心抽头 r1c1 作为 O_gray_center。
//     r1c1 是 Sobel 卷积的中心像素，也就是"当前正在计算梯度的那个灰度像素"，
//     它本身就是已经存在的寄存器，所以引出来不消耗任何额外资源。
//     这样就能直接在屏幕上显示灰度图 / 对灰度直接做二值化。
//     时序：gray -> r1c1 延迟 H_TOTAL+3 拍，比 edge_data 早 4 拍，
//     由上层 sobel_edge 补 4 级寄存器对齐。
//=============================================================================
module sobel_edge_core #(
    parameter H_RES   = 800,      // 一行有效像素数
    parameter H_TOTAL = H_RES     // 一行总时钟数（含消隐）；无消隐时填 H_RES
)(
    input               pclk,
    input               rst_n,
    input               gray_valid,      // 灰度有效
    input       [7:0]   gray,            // 灰度输入
    input               vid_vs,          // 有效帧期间为 1（帧同步期间为 0，清窗口）

    output reg  [11:0]  g_sum,           // 梯度幅值
    output reg          g_sum_valid,     // g_sum 有效
    output      [7:0]   O_gray_center,   // 3x3 窗口中心像素灰度（用于灰度/二值显示）
    output      [7:0]   O_median         // 3x3 窗口中值（用于中值滤波显示）
);

    //--------------------------------------------------------------------
    // 行缓存：缓存前两行灰度
    //--------------------------------------------------------------------
    wire [7:0] line0_out, line1_out;

    line_buf #( .WIDTH(8), .DEPTH(H_TOTAL) ) u_line0 (
        .clk    (pclk       ),
        .rst_n  (rst_n      ),
        .we     (gray_valid ),
        .din    (gray       ),
        .dout   (line0_out  )
    );

    line_buf #( .WIDTH(8), .DEPTH(H_TOTAL) ) u_line1 (
        .clk    (pclk       ),
        .rst_n  (rst_n      ),
        .we     (gray_valid ),
        .din    (line0_out  ),
        .dout   (line1_out  )
    );

    //--------------------------------------------------------------------
    // 3x3 窗口寄存器
    //   r2 = 当前行, r1 = 上一行(中心行), r0 = 上上行
    //--------------------------------------------------------------------
    reg [7:0] r2c0, r2c1, r2c2;
    reg [7:0] r1c0, r1c1, r1c2;
    reg [7:0] r0c0, r0c1, r0c2;

    always @(posedge pclk) begin
        if (!vid_vs) begin              // 帧同步期间清窗口
            r2c0<=0; r2c1<=0; r2c2<=0;
            r1c0<=0; r1c1<=0; r1c2<=0;
            r0c0<=0; r0c1<=0; r0c2<=0;
        end else if (gray_valid) begin
            r2c2 <= r2c1;  r2c1 <= r2c0;  r2c0 <= gray;
            r1c2 <= r1c1;  r1c1 <= r1c0;  r1c0 <= line0_out;
            r0c2 <= r0c1;  r0c1 <= r0c0;  r0c0 <= line1_out;
        end
    end

    // 中心像素灰度（卷积中心）
    assign O_gray_center = r1c1;

    //--------------------------------------------------------------------
    // 3x3 中值滤波：直接复用已经存在的 3x3 窗口寄存器
    //   median9 内部 4 级流水，输出直接与 edge_data 对齐。
    //--------------------------------------------------------------------
    median9 u_median9 (
        .clk      (pclk),
        .I_a0 (r0c0), .I_a1 (r0c1), .I_a2 (r0c2),
        .I_a3 (r1c0), .I_a4 (r1c1), .I_a5 (r1c2),
        .I_a6 (r2c0), .I_a7 (r2c1), .I_a8 (r2c2),
        .O_median (O_median)
    );

    //--------------------------------------------------------------------
    // Sobel 卷积：Gx、Gy
    //--------------------------------------------------------------------
    wire [10:0] gx_pos = r0c2 + (r1c2 << 1) + r2c2;
    wire [10:0] gx_neg = r0c0 + (r1c0 << 1) + r2c0;
    wire [10:0] gy_pos = r2c0 + (r2c1 << 1) + r2c2;
    wire [10:0] gy_neg = r0c0 + (r0c1 << 1) + r0c2;

    wire [10:0] gx = (gx_pos >= gx_neg) ? (gx_pos - gx_neg) : (gx_neg - gx_pos);
    wire [10:0] gy = (gy_pos >= gy_neg) ? (gy_pos - gy_neg) : (gy_neg - gy_pos);

    // |G| = |Gx| + |Gy|
    wire [11:0] g_sum_w = gx + gy;

    //--------------------------------------------------------------------
    // 输出寄存
    //--------------------------------------------------------------------
    always @(posedge pclk) begin
        g_sum       <= g_sum_w;
        g_sum_valid <= gray_valid;
    end

endmodule
