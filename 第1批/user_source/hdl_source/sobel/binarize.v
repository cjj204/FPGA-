//=============================================================================
// 子模块3：二值化处理 + 同步信号延迟对齐
//   输入：12bit 梯度幅值
//   输出：24bit 二值化边缘图（白边黑底）+ 对齐后的同步信号
//
//   [移植改动 2026-10] DLY 由 localparam 改为 parameter
//     原始值 H_RES+1 只在"无消隐的连续像素流 + 行缓存深度=H_RES"下成立。
//     集成到 1280x720@60 的 HDMI 链路时，数据通路真实延迟为 H_TOTAL+5，
//     由 sobel_edge 顶层算出后传入。
//
//   [改动 2026-10-15] THRESHOLD 由 parameter 改为输入端口 I_threshold
//     这样阈值可以在运行时通过按键改变，无需重新综合。
//     比较器直接驱动 edge_data 寄存器的 D 端，不增加流水级，
//     因此数据通路延迟仍是 1 拍（保持不变）。
//
//   注意：edge_active_video 是 vid_active_video 的延迟版本。
//     集成时 vid_active_video 恒为 1（连续流），故该输出为常量 1，
//     显示用的 de 由 sobel_video_wrap 单独做同深度延迟，请勿使用本输出。
//     edge_vs / edge_hs 仍为 vid_vs / vid_hs 的正确延迟版本。
//=============================================================================
module binarize #(
    parameter H_RES     = 800,
    parameter DLY       = H_RES + 1    // 同步信号延迟拍数（须与数据通路一致）
)(
    input               pclk,
    input               rst_n,

    // 梯度幅值输入
    input       [11:0]  g_sum,
    input               g_sum_valid,

    // 二值化阈值（运行时可调，12bit）
    input       [11:0]  I_threshold,

    // 原始同步信号（用于延迟对齐）
    input               vid_active_video,
    input               vid_hs,
    input               vid_vs,

    // 二值化输出
    output reg  [23:0]  edge_data,
    output              edge_active_video,
    output              edge_hs,
    output              edge_vs
);

    //--------------------------------------------------------------------
    // 阈值判决：白边黑底（阈值来自端口，可运行时改变）
    //--------------------------------------------------------------------
    always @(posedge pclk) begin
        if (g_sum >= I_threshold)
            edge_data <= 24'hFFFFFF;   // 超过阈值：白色
        else
            edge_data <= 24'h000000;   // 否则：黑色
    end

    //--------------------------------------------------------------------
    // 同步信号延迟对齐
    //--------------------------------------------------------------------
    reg [DLY-1:0] act_d, hs_d, vs_d;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            act_d <= {DLY{1'b0}};
            hs_d  <= {DLY{1'b0}};
            vs_d  <= {DLY{1'b0}};
        end else begin
            act_d <= {act_d[DLY-2:0], vid_active_video};
            hs_d  <= {hs_d [DLY-2:0], vid_hs};
            vs_d  <= {vs_d [DLY-2:0], vid_vs};
        end
    end

    assign edge_active_video = act_d[DLY-1];
    assign edge_hs           = hs_d [DLY-1];
    assign edge_vs           = vs_d [DLY-1];

endmodule
