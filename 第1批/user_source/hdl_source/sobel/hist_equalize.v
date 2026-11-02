//=============================================================================
//  hist_equalize —— 全局直方图均衡（帧间 LUT）
//
//  数据流：
//    · 当前帧：对每个有效灰度像素做直方图计数（256 bin）。
//    · 帧消隐期：用 512 个时钟顺序读出 256 个 bin，累加 CDF，
//      计算 256 项映射 LUT，同时把直方图清零。
//    · 下一帧：用新 LUT 对灰度做映射，输出与 de 严格对齐。
//
//  资源：
//    hist 256x20 / lut 256x8 均为简单双口 BRAM（一写一读），
//    不占用额外帧缓存；CDF 归一化用常数乘加 (cdf * 291) >> 20，
//    避免使用昂贵的除法器。
//
//  延迟：
//    像素级流水 1 拍（LUT 同步读）；算法上使用"上一帧"的统计结果，
//    因此相比原图多 1 帧的统计延迟（OSD/遥测中会计入）。
//=============================================================================
module hist_equalize #(
    parameter PIXELS = 1280 * 720
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        I_de,       // 有效像素
    input  wire        I_vs,       // 帧同步高脉冲（消隐期）
    input  wire [7:0]  I_gray,     // 与 I_de 对齐的灰度

    output reg  [7:0]  O_gray,     // 与 O_de 对齐的均衡灰度
    output reg         O_de,
    output reg         O_vs
);

    //------------------------------------------------------------------
    // 帧同步上升沿
    //------------------------------------------------------------------
    reg S_vs_d;
    wire S_vs_rise = I_vs & ~S_vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            S_vs_d <= 1'b0;
        else
            S_vs_d <= I_vs;
    end

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam [1:0] S_INIT   = 2'd0,
                     S_RUN    = 2'd1,
                     S_PROC_R = 2'd2,
                     S_PROC_C = 2'd3;

    reg [1:0]  S_state;
    reg [7:0]  S_idx;        // INIT / PROC 地址
    reg [7:0]  S_idx_d;      // PROC 计算级地址
    reg [20:0] S_cdf;        // 累计像素数（最大 921600）
    reg        S_cdf_min_found;
    reg [20:0] S_cdf_min;
    reg        S_lut_ready;
    reg        S_stats_valid;

    //------------------------------------------------------------------
    // 直方图 / LUT 存储
    //------------------------------------------------------------------
    reg [19:0] S_hist [0:255];
    reg [7:0]  S_lut  [0:255];

    reg [19:0] S_hist_q;

    // hist：RUN 时计数；INIT 清零；PROC_C 读后清零
    // 20bit 计数可容纳 1280*720=921600 个同灰度像素，避免 16bit 溢出
    always @(posedge clk) begin
        if (S_state == S_INIT)
            S_hist[S_idx] <= 20'd0;
        else if (S_state == S_RUN) begin
            if (I_de)
                S_hist[I_gray] <= S_hist[I_gray] + 20'd1;
        end
        else if (S_state == S_PROC_C)
            S_hist[S_idx_d] <= 20'd0;
    end

    // hist 读端口（PROC_R 同步读）
    always @(posedge clk) begin
        if (S_state == S_PROC_R)
            S_hist_q <= S_hist[S_idx];
    end

    // LUT：INIT 写恒等映射；PROC_C 写均衡映射；RUN 读
    always @(posedge clk) begin
        if (S_state == S_INIT)
            S_lut[S_idx] <= S_idx;
        else if (S_state == S_PROC_C)
            S_lut[S_idx_d] <= S_lut_value;
    end

    //------------------------------------------------------------------
    // CDF 归一化
    //    cdf_eff = cdf + hist_q - cdf_min
    //    lut     = (cdf_eff * 291) >> 20，饱和到 255
    //------------------------------------------------------------------
    wire [20:0] S_cdf_next = S_cdf + {1'b0, S_hist_q};
    wire        S_first_bin = (S_hist_q != 20'd0) && !S_cdf_min_found;
    wire [20:0] S_cdf_min_n = S_first_bin ? S_cdf_next : S_cdf_min;
    wire [20:0] S_cdf_eff   = S_cdf_next - S_cdf_min_n;
    wire [31:0] S_scaled    = S_cdf_eff * 32'd291;
    wire [7:0]  S_lut_value = (S_scaled[27:20] > 8'd255) ? 8'd255
                                                          : S_scaled[27:20];

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state         <= S_INIT;
            S_idx           <= 8'd0;
            S_idx_d         <= 8'd0;
            S_cdf           <= 21'd0;
            S_cdf_min_found <= 1'b0;
            S_cdf_min       <= 21'd0;
            S_lut_ready     <= 1'b0;
            S_stats_valid   <= 1'b0;
        end else begin
            case (S_state)
                S_INIT: begin
                    if (S_idx == 8'hFF) begin
                        S_state     <= S_RUN;
                        S_lut_ready <= 1'b1;
                    end else begin
                        S_idx <= S_idx + 8'd1;
                    end
                end

                S_RUN: begin
                    if (I_de)
                        S_stats_valid <= 1'b1;

                    if (S_vs_rise && S_stats_valid) begin
                        S_state         <= S_PROC_R;
                        S_idx           <= 8'd0;
                        S_cdf           <= 21'd0;
                        S_cdf_min_found <= 1'b0;
                        S_cdf_min       <= 21'd0;
                        S_stats_valid   <= 1'b0;
                    end
                end

                S_PROC_R: begin
                    S_idx_d <= S_idx;
                    S_state <= S_PROC_C;
                end

                S_PROC_C: begin
                    S_cdf <= S_cdf_next;
                    if (S_first_bin) begin
                        S_cdf_min_found <= 1'b1;
                        S_cdf_min       <= S_cdf_next;
                    end

                    if (S_idx_d == 8'hFF) begin
                        S_state     <= S_RUN;
                        S_lut_ready <= 1'b1;
                    end else begin
                        S_idx   <= S_idx_d + 8'd1;
                        S_state <= S_PROC_R;
                    end
                end

                default: S_state <= S_RUN;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 像素输出：LUT 同步读，de/vs 各延迟 1 拍
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            O_gray <= 8'd0;
            O_de   <= 1'b0;
            O_vs   <= 1'b0;
        end else begin
            O_de <= I_de;
            O_vs <= I_vs;
            if (I_de)
                O_gray <= S_lut_ready ? S_lut[I_gray] : I_gray;
            else
                O_gray <= 8'd0;
        end
    end

endmodule
