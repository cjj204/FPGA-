//=============================================================================
//  sobel_video_wrap —— 6 种图像处理结果接入 HDMI 链路
//
//  插入位置：hdmi_mixer 与 hdmi_tx 之间
//     hdmi_mixer.O_hdmi_data/de/vsync/hsync  →  本模块  →  hdmi_tx
//
//  显示模式（I_mode，按键/串口切换）：
//     0 = 彩色原图      零延迟直通
//     1 = 边缘检测      3x3 Sobel：|Gx|+|Gy| >= 阈值
//     2 = 灰度变换      Y = (77R + 150G + 29B) >> 8
//     3 = 二值化        灰度 >= 灰度阈值 ? 白 : 黑
//     4 = 中值滤波      3x3 窗口 9 点中值
//     5 = 直方图均衡    全局直方图 CDF 映射（使用上一帧统计的 LUT）
//
//  阈值档位（I_level，按键/串口切换）：
//     level 0/1/2/3 -> 边缘阈值 30/60/120/240，灰度二值阈值 64/128/170/210
//
//  关键设计点：
//   1) vid_active_video 恒接 1'b1 —— 让行缓存与窗口寄存器每个时钟都移位，
//      使数据通路延迟成为"恒定 H_TOTAL+7 拍"的时钟域延迟。
//      同时 line_buf 深度取 H_TOTAL(1650，含消隐) 而不是 H_RES(1280)，
//      这样行缓存正好覆盖一整行扫描，3x3 窗口不会跨错行。
//   2) 显示用 de 由本模块单独做 SOBEL_DLY 拍延迟，与数据严格对齐。
//   3) 模式 1~4 共用 SOBEL_DLY 时序；模式 5 的 HE LUT 同步读再 +1 拍；
//      模式 0 使用原始时序。
//   4) 左上角叠加 logo + 模式 + FPGA/上位机延迟；输出端统一打 1 拍。 
//=============================================================================
module sobel_video_wrap #(
    parameter H_RES     = 1280,        // 一行有效像素
    parameter V_RES     = 720,         // 一帧有效行
    parameter H_TOTAL   = 1650,        // 一行总时钟数（含消隐）
    parameter GRAY_MODE = 1
)(
    input  wire        I_clk,          // 像素时钟
    input  wire        I_rst_n,
    input  wire [2:0]  I_mode,         // 显示模式 0~5
    input  wire [1:0]  I_level,        // 阈值档位 0~3

    // OSD 用的延迟/帧计数（像素时钟域）
    input  wire [7:0]  I_fpga_frames,
    input  wire [15:0] I_fpga_ms,
    input  wire [15:0] I_host_ms,
    input  wire [15:0] I_frame_cnt,

    input  wire        I_video_de,
    input  wire        I_video_vsync,  // 高有效的帧同步脉冲
    input  wire        I_video_hsync,
    input  wire [23:0] I_video_data,

    output wire        O_video_de,
    output wire        O_video_vsync,
    output wire [23:0] O_video_data
);

    // 与 sobel_edge 内部 SYNC_DLY 必须一致：H_TOTAL + 7
    localparam integer SOBEL_DLY = H_TOTAL + 7;

    localparam [2:0] MODE_ORIG   = 3'd0;
    localparam [2:0] MODE_EDGE   = 3'd1;
    localparam [2:0] MODE_GRAY   = 3'd2;
    localparam [2:0] MODE_BIN    = 3'd3;
    localparam [2:0] MODE_MEDIAN = 3'd4;
    localparam [2:0] MODE_HE     = 3'd5;

    //------------------------------------------------------------------
    // 阈值档位译码
    //------------------------------------------------------------------
    wire [11:0] S_edge_thr = (I_level == 2'd0) ? 12'd30  :
                             (I_level == 2'd1) ? 12'd60  :
                             (I_level == 2'd2) ? 12'd120 : 12'd240;

    wire [7:0]  S_gray_thr = (I_level == 2'd0) ? 8'd64  :
                             (I_level == 2'd1) ? 8'd128 :
                             (I_level == 2'd2) ? 8'd170 : 8'd210;

    //------------------------------------------------------------------
    // Sobel / 灰度 / 中值通路
    //------------------------------------------------------------------
    wire [23:0] S_edge_data;
    wire        S_edge_hs;
    wire        S_edge_vs;
    wire [7:0]  S_gray;
    wire [7:0]  S_median;

    //------------------------------------------------------------------
    // 显示数据有效信号的同深度延迟链
    //------------------------------------------------------------------
    reg [SOBEL_DLY-1:0] S_de_dly;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_de_dly <= {SOBEL_DLY{1'b0}};
        else
            S_de_dly <= {S_de_dly[SOBEL_DLY-2:0], I_video_de};
    end

    wire S_edge_de = S_de_dly[SOBEL_DLY-1];

    sobel_edge #(
        .H_RES     ( H_RES     ),
        .V_RES     ( V_RES     ),
        .H_TOTAL   ( H_TOTAL   ),
        .GRAY_MODE ( GRAY_MODE )
    ) u_sobel_edge (
        .pclk              ( I_clk          ),
        .rst_n             ( I_rst_n        ),
        .vid_active_video  ( 1'b1           ),   // 连续流：消隐期也移位，延迟恒定
        .vid_hs            ( I_video_hsync  ),
        .vid_vs            ( ~I_video_vsync ),   // 有效帧期间为 1
        .vid_data          ( I_video_data   ),
        .I_edge_threshold  ( S_edge_thr     ),

        .video_active      (                ),
        .video_hs          (                ),
        .video_vs          (                ),
        .video_data        (                ),

        .edge_active_video (                ),
        .edge_hs           ( S_edge_hs      ),
        .edge_vs           ( S_edge_vs      ),
        .edge_data         ( S_edge_data    ),

        .O_gray_aligned    ( S_gray         ),
        .O_median_aligned  ( S_median       )
    );

    //------------------------------------------------------------------
    // 直方图均衡：常开，保证切到模式 5 时 LUT 已就绪
    //------------------------------------------------------------------
    wire [7:0] S_he_gray;
    wire       S_he_de;
    wire       S_he_vs;

    hist_equalize #(
        .PIXELS ( H_RES * V_RES )
    ) u_hist_equalize (
        .clk    ( I_clk      ),
        .rst_n  ( I_rst_n    ),
        .I_de   ( S_edge_de  ),
        .I_vs   ( ~S_edge_vs ),   // 帧同步高脉冲（消隐期）
        .I_gray ( S_gray     ),
        .O_gray ( S_he_gray  ),
        .O_de   ( S_he_de    ),
        .O_vs   ( S_he_vs    )
    );

    //------------------------------------------------------------------
    // 模式选择
    //------------------------------------------------------------------
    wire [23:0] S_gray_rgb   = {S_gray,   S_gray,   S_gray};
    wire [23:0] S_median_rgb = {S_median, S_median, S_median};
    wire [23:0] S_he_rgb     = {S_he_gray, S_he_gray, S_he_gray};
    wire [23:0] S_gray_bin   = (S_gray >= S_gray_thr) ? 24'hFFFFFF : 24'h000000;

    wire S_use_delayed = (I_mode != MODE_ORIG) && (I_mode != MODE_HE);

    wire        S_ode   = (I_mode == MODE_HE) ? S_he_de :
                          (S_use_delayed ? S_edge_de : I_video_de);
    wire        S_ovs   = (I_mode == MODE_HE) ? S_he_vs :
                          (S_use_delayed ? ~S_edge_vs : I_video_vsync);
    wire [23:0] S_odata = (I_mode == MODE_ORIG)   ? I_video_data :
                          (I_mode == MODE_EDGE)   ? S_edge_data  :
                          (I_mode == MODE_GRAY)   ? S_gray_rgb   :
                          (I_mode == MODE_BIN)    ? S_gray_bin   :
                          (I_mode == MODE_MEDIAN) ? S_median_rgb :
                                                    S_he_rgb;

    //------------------------------------------------------------------
    // 状态叠加（纯组合，不改变时序对齐）
    //------------------------------------------------------------------
    wire [23:0] S_osd_data;

    osd_status #(
        .H_ACTIVE ( H_RES )
    ) u_osd (
        .pclk          ( I_clk        ),
        .rst_n         ( I_rst_n      ),
        .I_de          ( S_ode        ),
        .I_vs          ( S_ovs        ),
        .I_mode        ( I_mode       ),
        .I_level       ( I_level      ),
        .I_fpga_frames ( I_fpga_frames),
        .I_fpga_ms     ( I_fpga_ms    ),
        .I_host_ms     ( I_host_ms    ),
        .I_frame_cnt   ( I_frame_cnt  ),
        .I_data        ( S_odata      ),
        .O_data        ( S_osd_data   )
    );

    //------------------------------------------------------------------
    // 输出打一拍：OSD 字模 ROM + 数据 mux 的组合路径较长，在 75MHz 下
    // 用一级寄存器切断；de/vs 同步延迟 1 拍，保持三者对齐。
    //------------------------------------------------------------------
    reg [23:0] S_video_data_r;
    reg        S_video_de_r;
    reg        S_video_vs_r;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_video_data_r <= 24'd0;
            S_video_de_r   <= 1'b0;
            S_video_vs_r   <= 1'b0;
        end else begin
            S_video_data_r <= S_osd_data;
            S_video_de_r   <= S_ode;
            S_video_vs_r   <= S_ovs;
        end
    end

    assign O_video_de    = S_video_de_r;
    assign O_video_vsync = S_video_vs_r;
    assign O_video_data  = S_video_data_r;

endmodule
