//=============================================================================
//  latency_monitor —— 采集到显示的延迟测量（v2 优化版）
//
//  原理（帧标签法）：
//    链路用 4 个 DDR 帧缓存。video_in 每写一帧，就给该缓存打上"写入时刻"标签
//      （写入帧序号 + 毫秒时间戳）；
//    video_out 每次开始显示一帧，取出当前读指针 rp 那个缓存的标签，
//      age_frames = 写帧序号 - 标签帧序号
//      age_ms     = 现在毫秒   - 标签毫秒
//    就是"这一帧从被采集到现在过了多久"。
//
//  ---------------------------------------------------------------
//  相对 v1 的 6 处改进（v1 的问题写在注释里，方便答辩讲）
//  ---------------------------------------------------------------
//  1) 毫秒是"真时间"，不再靠 1 帧=16.667ms 查表硬换算
//     v1 的 frames_to_ms 只对 60fps 成立：相机若 30fps，2 帧其实是 66ms，
//     却被算成 33ms（差一倍）。
//     v2 用像素时钟打 1ms 时基，age_ms 直接由时间戳相减得到，与帧率无关。
//  2) 统一到像素时钟域，杜绝"位偏斜"
//     v1 在 DDR 域算完再把 8bit/16bit 结果逐位 2FF 同步，可能拼出
//     "新的帧数 + 旧的毫秒"。v2 只把两个帧事件用 toggle 同步过来，
//     全部运算在像素域完成，O_frames/O_ms/O_valid 天然同拍。
//  3) 消除同拍竞争
//     v1 写帧与显示帧同一拍时 age 可能差 1。v2 用 age_f>=1 且 age_f<16
//     双重限定（>=1 排除"本帧刚写就显示"的错值）。
//  4) 抗抖动：1/8 指数滑动平均（可调 SMOOTH_S）
//     60Hz 显示 60Hz 源，真实帧龄在 2/3 之间跳，v1 直接显示会乱跳；
//     v2 平滑后 OSD 数字稳定。
//  5) 失配保持：标签未建立/瞬时无效时，保持上一次有效值约 1 秒，
//     不让 OSD 和遥测突然闪 000。
//  6) 帧周期实测输出 O_period_ms
//     直方图均衡要 +1 帧补偿，v1 硬编码 +17ms；v2 给出实测周期，
//     由顶层相加，帧率变了也不会错。
//
//  口径说明：本延迟包含 DDR 帧缓存 + 扫描输出；
//            不含 sensor 曝光与 ISP 内部流水（那部分没有跨帧时间戳）。
//=============================================================================
module latency_monitor #(
    parameter integer PIX_CLK_HZ = 74250000,   // 像素时钟（74.25MHz）
    parameter integer SMOOTH_S   = 3           // 平均窗口 = 2^SMOOTH_S 帧（3 -> 8 帧）
)(
    input  wire        I_ddr_clk,
    input  wire        I_ddr_rst_n,

    input  wire        I_write_frame_start,    // video_in 写帧开始（DDR 域）
    input  wire        I_display_frame_start,  // video_out 显示帧开始（DDR 域）
    input  wire [1:0]  I_wr_ptr,               // 写缓存编号
    input  wire [1:0]  I_rd_ptr,               // 读缓存编号
    input  wire        I_clear,                // 清统计（DDR 域单周期）

    input  wire        I_pix_clk,
    input  wire        I_pix_rst_n,

    output reg  [7:0]  O_frames,               // 帧龄（帧）
    output reg  [15:0] O_ms,                   // 帧龄（毫秒）
    output reg         O_valid,                // 结果有效
    output reg  [15:0] O_period_ms             // 实测帧周期（毫秒）
);

    localparam integer MS_DIV = (PIX_CLK_HZ + 500) / 1000;   // 1ms 的时钟数
    localparam integer HOLD_N = 64;                          // 无效时保持约 1s
    localparam [16:0] MS_DIV_M1 = MS_DIV - 1;                // 定宽常量，避免有符号比较
    localparam [6:0]  HOLD_M1   = HOLD_N - 1;

    //=========================================================================
    // 1) DDR 域事件 -> 像素域（toggle + 3FF + 边沿）
    //=========================================================================
    reg wr_tog, rd_tog, clr_tog;

    always @(posedge I_ddr_clk or negedge I_ddr_rst_n) begin
        if (!I_ddr_rst_n) begin
            wr_tog  <= 1'b0;
            rd_tog  <= 1'b0;
            clr_tog <= 1'b0;
        end else begin
            if (I_write_frame_start)   wr_tog  <= ~wr_tog;
            if (I_display_frame_start) rd_tog  <= ~rd_tog;
            if (I_clear)               clr_tog <= ~clr_tog;
        end
    end

    // 缓存编号随事件一起"寄存一下"，保证像素域采到的是这一帧的值
    reg [1:0] wr_ptr_d, rd_ptr_d;
    always @(posedge I_ddr_clk or negedge I_ddr_rst_n) begin
        if (!I_ddr_rst_n) begin
            wr_ptr_d <= 2'd0;
            rd_ptr_d <= 2'd0;
        end else begin
            if (I_write_frame_start)   wr_ptr_d <= I_wr_ptr;
            if (I_display_frame_start) rd_ptr_d <= I_rd_ptr;
        end
    end

    reg [2:0] wr_s, rd_s, clr_s;
    reg [1:0] wp_s1, wp_s2, rp_s1, rp_s2;

    always @(posedge I_pix_clk or negedge I_pix_rst_n) begin
        if (!I_pix_rst_n) begin
            wr_s  <= 3'd0;
            rd_s  <= 3'd0;
            clr_s <= 3'd0;
            wp_s1 <= 2'd0;
            wp_s2 <= 2'd0;
            rp_s1 <= 2'd0;
            rp_s2 <= 2'd0;
        end else begin
            wr_s  <= {wr_s[1:0],  wr_tog};
            rd_s  <= {rd_s[1:0],  rd_tog};
            clr_s <= {clr_s[1:0], clr_tog};
            wp_s1 <= wr_ptr_d;
            wp_s2 <= wp_s1;
            rp_s1 <= rd_ptr_d;
            rp_s2 <= rp_s1;
        end
    end

    wire wr_evt  = wr_s[2]  ^ wr_s[1];
    wire rd_evt  = rd_s[2]  ^ rd_s[1];
    wire clr_evt = clr_s[2] ^ clr_s[1];

    //=========================================================================
    // 2) 1ms 时基（像素时钟分频）
    //=========================================================================
    reg [16:0] ms_div;
    reg [15:0] now_ms;

    always @(posedge I_pix_clk or negedge I_pix_rst_n) begin
        if (!I_pix_rst_n) begin
            ms_div <= 17'd0;
            now_ms <= 16'd0;
        end else if (clr_evt) begin
            ms_div <= 17'd0;
            now_ms <= 16'd0;
        end else if (ms_div >= MS_DIV_M1) begin
            ms_div <= 17'd0;
            now_ms <= now_ms + 16'd1;
        end else begin
            ms_div <= ms_div + 17'd1;
        end
    end

    //=========================================================================
    // 3) 每个缓存的"写入标签"：帧序号 + 毫秒时间戳
    //=========================================================================
    reg [15:0] tag_ms [0:3];
    reg [7:0]  tag_fr [0:3];
    reg [3:0]  tag_v;
    reg [7:0]  wr_frm;
    reg [15:0] last_wr_ms;
    reg [15:0] per_ms;

    always @(posedge I_pix_clk or negedge I_pix_rst_n) begin
        if (!I_pix_rst_n) begin
            tag_ms[0] <= 16'd0; tag_ms[1] <= 16'd0;
            tag_ms[2] <= 16'd0; tag_ms[3] <= 16'd0;
            tag_fr[0] <= 8'd0;  tag_fr[1] <= 8'd0;
            tag_fr[2] <= 8'd0;  tag_fr[3] <= 8'd0;
            tag_v     <= 4'd0;
            wr_frm    <= 8'd0;
            last_wr_ms <= 16'd0;
            per_ms    <= 16'd16;
        end else if (clr_evt) begin
            tag_ms[0] <= 16'd0; tag_ms[1] <= 16'd0;
            tag_ms[2] <= 16'd0; tag_ms[3] <= 16'd0;
            tag_fr[0] <= 8'd0;  tag_fr[1] <= 8'd0;
            tag_fr[2] <= 8'd0;  tag_fr[3] <= 8'd0;
            tag_v     <= 4'd0;
            wr_frm    <= 8'd0;
            last_wr_ms <= 16'd0;
            per_ms    <= 16'd16;
        end else if (wr_evt) begin
            tag_ms[wp_s2] <= now_ms;
            tag_fr[wp_s2] <= wr_frm + 8'd1;
            tag_v[wp_s2]  <= 1'b1;
            wr_frm        <= wr_frm + 8'd1;
            last_wr_ms    <= now_ms;
            per_ms        <= now_ms - last_wr_ms;      // 实测帧周期
        end
    end

    //=========================================================================
    // 4) 显示侧：算帧龄 / 毫秒，并做滑动平均
    //=========================================================================
    wire [7:0]  age_f  = wr_frm - tag_fr[rp_s2];
    wire [15:0] age_ms = now_ms - tag_ms[rp_s2];
    wire        lat_ok = tag_v[rp_s2] && (age_f >= 8'd1) && (age_f < 8'd16);

    // 平均用"放大 8 倍"的累加器：acc = acc - acc/8 + new
    reg [11:0] f_acc;
    reg [19:0] ms_acc;
    reg [6:0]  hold_cnt;

    wire [11:0] f_acc_nxt  = f_acc  - (f_acc  >> SMOOTH_S) + {4'd0, age_f};
    wire [19:0] ms_acc_nxt = ms_acc - (ms_acc >> SMOOTH_S) + {4'd0, age_ms};
    wire [11:0] f_avg  = f_acc_nxt  >> SMOOTH_S;
    wire [19:0] ms_avg = ms_acc_nxt >> SMOOTH_S;

    always @(posedge I_pix_clk or negedge I_pix_rst_n) begin
        if (!I_pix_rst_n) begin
            f_acc    <= 12'd0;
            ms_acc   <= 20'd0;
            hold_cnt <= 7'd0;
            O_frames <= 8'd0;
            O_ms     <= 16'd0;
            O_valid  <= 1'b0;
        end else if (clr_evt) begin
            f_acc    <= 12'd0;
            ms_acc   <= 20'd0;
            hold_cnt <= 7'd0;
            O_frames <= 8'd0;
            O_ms     <= 16'd0;
            O_valid  <= 1'b0;
        end else if (rd_evt) begin
            if (lat_ok) begin
                f_acc    <= f_acc_nxt;
                ms_acc   <= ms_acc_nxt;
                O_frames <= f_avg[7:0];
                O_ms     <= ms_avg[15:0];
                O_valid  <= 1'b1;
                hold_cnt <= 7'd0;
            end else if (hold_cnt < HOLD_M1) begin
                // 短暂无效（启动/切模式）：保持上一次有效值，避免闪 000
                hold_cnt <= hold_cnt + 7'd1;
            end else begin
                f_acc    <= 12'd0;
                ms_acc   <= 20'd0;
                O_frames <= 8'd0;
                O_ms     <= 16'd0;
                O_valid  <= 1'b0;
            end
        end
    end

    //=========================================================================
    // 5) 实测帧周期输出（1/4 平滑，给直方图均衡补偿用）
    //=========================================================================
    wire [15:0] per_chk  = ((per_ms >= 16'd1) && (per_ms <= 16'd255)) ? per_ms : O_period_ms;
    wire [15:0] per_nxt  = O_period_ms - (O_period_ms >> 2) + (per_chk >> 2);

    always @(posedge I_pix_clk or negedge I_pix_rst_n) begin
        if (!I_pix_rst_n) begin
            O_period_ms <= 16'd16;
        end else if (clr_evt) begin
            O_period_ms <= 16'd16;
        end else if (wr_evt) begin
            O_period_ms <= per_nxt;
        end
    end

endmodule
