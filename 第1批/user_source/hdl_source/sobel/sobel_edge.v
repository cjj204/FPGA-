//========================================================================
// 顶层模块：Sobel 边缘检测系统
//   接口与原 binary 模块兼容：
//     - 原图直通：video_active/hs/vs/data
//     - 处理输出：edge_active_video/hs/vs/data
//
//   [移植改动 2026-10] 增加参数 H_TOTAL，并把真实数据通路延迟算给 binarize
//     H_RES   = 一行有效像素数
//     H_TOTAL = 一行总时钟数（含行消隐）
//
//   数据通路延迟推导（vid_active_video 恒为 1 的连续流）：
//     gray_extract 寄存            1 拍
//     line_buf DEPTH=H_TOTAL       H_TOTAL 拍
//     窗口 r1c0 / r1c1            2 拍
//     g_sum 寄存                  1 拍
//     binarize edge_data 寄存     1 拍
//     ------------------------------------
//     合计                        H_TOTAL + 7 拍（含中值 4 级流水）
//   故 SYNC_DLY = H_TOTAL + 7，保证 edge_hs/edge_vs 与 edge_data 严格对齐。
//
//   [改动 2026-10-15] 支持三种处理结果分别输出：
//     THRESHOLD 参数改为端口 I_edge_threshold（运行时按键可调，不增加流水级）
//     新增 O_gray_aligned：与 edge_data 严格对齐的 8bit 灰度图
//       推导：gray -> r1c1 (3x3 窗口中心) 延迟 H_TOTAL+3 拍
//             再补 4 级寄存器 => H_TOTAL+7，与 edge_data 完全对齐
//========================================================================
module sobel_edge #(
    parameter H_RES     = 800,
    parameter V_RES     = 480,
    parameter H_TOTAL   = H_RES,       // 一行总时钟数（含消隐）；无消隐填 H_RES
    parameter GRAY_MODE = 1            // 0: 取R分量  1: 加权亮度
)(
    input               pclk,
    input               rst_n,

    // 视频输入（来自 cmos_data）
    input               vid_active_video,
    input               vid_hs,
    input               vid_vs,
    input       [23:0]  vid_data,

    // 边缘二值化阈值（运行时可调）
    input       [11:0]  I_edge_threshold,

    // 原图直通输出
    output              video_active,
    output              video_hs,
    output              video_vs,
    output      [23:0]  video_data,

    // 边缘检测输出（原 binary 的 bin_* 位置）
    output              edge_active_video,
    output              edge_hs,
    output              edge_vs,
    output      [23:0]  edge_data,

    // 灰度图输出（与 edge_data 严格对齐，8bit）
    output      [7:0]   O_gray_aligned,

    // 3x3 中值滤波输出（与 edge_data 严格对齐，8bit）
    output      [7:0]   O_median_aligned
);

    // 同步信号延迟 = 数据通路延迟（中值 4 级流水后统一为 H_TOTAL+7）
    localparam integer SYNC_DLY = H_TOTAL + 7;

    //--------------------------------------------------------------------
    // 原图直通
    //--------------------------------------------------------------------
    assign video_active = vid_active_video;
    assign video_hs     = vid_hs;
    assign video_vs     = vid_vs;
    assign video_data   = vid_data;

    //--------------------------------------------------------------------
    // 子模块1：灰度提取
    //--------------------------------------------------------------------
    wire [7:0] gray;
    wire       gray_valid;

    gray_extract #(
        .GRAY_MODE (GRAY_MODE)
    ) u_gray_extract (
        .pclk             (pclk            ),
        .vid_active_video (vid_active_video),
        .vid_data         (vid_data        ),
        .gray             (gray            ),
        .gray_valid       (gray_valid      )
    );

    //--------------------------------------------------------------------
    // 子模块2：Sobel 边缘检测核心（同时引出 3x3 窗口中心灰度）
    //--------------------------------------------------------------------
    wire [11:0] g_sum;
    wire        g_sum_valid;
    wire [7:0]  gray_center;
    wire [7:0]  median_raw;

    sobel_edge_core #(
        .H_RES   (H_RES  ),
        .H_TOTAL (H_TOTAL)
    ) u_sobel_core (
        .pclk           (pclk        ),
        .rst_n          (rst_n       ),
        .gray_valid     (gray_valid  ),
        .gray           (gray        ),
        .vid_vs         (vid_vs      ),
        .g_sum          (g_sum       ),
        .g_sum_valid    (g_sum_valid ),
        .O_gray_center  (gray_center ),
        .O_median       (median_raw  )
    );

    //--------------------------------------------------------------------
    // 灰度对齐：gray_center 比 edge_data 早 4 拍，补 4 级寄存器
    //   median9 内部已做 4 级流水，O_median 直接与 edge_data 对齐。
    //--------------------------------------------------------------------
    reg [7:0] gray_d1, gray_d2, gray_d3, gray_d4;

    always @(posedge pclk) begin
        gray_d1   <= gray_center;
        gray_d2   <= gray_d1;
        gray_d3   <= gray_d2;
        gray_d4   <= gray_d3;
    end

    assign O_median_aligned = median_raw;
    assign O_gray_aligned   = gray_d4;

    //--------------------------------------------------------------------
    // 子模块3：二值化 + 同步信号延迟对齐
    //--------------------------------------------------------------------
    binarize #(
        .H_RES     (H_RES       ),
        .DLY       (SYNC_DLY    )
    ) u_binarize (
        .pclk              (pclk            ),
        .rst_n             (rst_n           ),
        .g_sum             (g_sum           ),
        .g_sum_valid       (g_sum_valid     ),
        .I_threshold       (I_edge_threshold),
        .vid_active_video  (vid_active_video),
        .vid_hs            (vid_hs          ),
        .vid_vs            (vid_vs          ),
        .edge_data         (edge_data       ),
        .edge_active_video (edge_active_video),
        .edge_hs           (edge_hs         ),
        .edge_vs           (edge_vs         )
    );

endmodule
