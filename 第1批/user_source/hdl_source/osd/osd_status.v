//=============================================================================
//  osd_status —— 屏幕左上角状态叠加
//
//  布局（1280x720）：
//    +------------------------------------------------------+
//    |  [安路彩色 logo 160x51]  边缘检测                     |
//    |                          FPGA 002F 033MS             |
//    |                          HOST 012MS  FRM 00123       |
//    +------------------------------------------------------+
//
//  说明：
//    · 去掉了原来的"安路"16x16 中文字模，改为 osd_logo_rom 中的
//      160x51 双色 logo（深蓝 + 安路红）。
//    · 模式名使用 16x16 中文字模放大 2 倍；延迟/帧计数使用 8x16 ASCII。
//    · 全部为纯组合叠加，不增加 de/vs/data 的流水级。
//    · 数字由 bin2bcd16 组合转换为十进制，便于直接观察。
//=============================================================================
module osd_status #(
    parameter H_ACTIVE = 1280,
    parameter BOX_X    = 24,
    parameter BOX_Y    = 20,
    parameter BOX_W    = 600,
    parameter BOX_H    = 84,
    parameter LOGO_X   = 30,
    parameter LOGO_Y   = 36,
    parameter LOGO_W   = 160,
    parameter LOGO_H   = 51,
    parameter TEXT_X   = 204,
    parameter MODE_Y   = 22,
    parameter LINE1_Y  = 58,
    parameter LINE2_Y  = 76,
    parameter BG_COLOR = 24'h101828,
    parameter FG_COLOR = 24'hFFFFFF
)(
    input  wire        pclk,
    input  wire        rst_n,

    input  wire        I_de,
    input  wire        I_vs,

    input  wire [2:0]  I_mode,          // 0..5
    input  wire [1:0]  I_level,         // 保留

    input  wire [7:0]  I_fpga_frames,   // 采集->显示延迟（帧）
    input  wire [15:0] I_fpga_ms,       // 对应毫秒（整数）
    input  wire [15:0] I_host_ms,       // 上位机回传的指令往返延迟（毫秒）
    input  wire [15:0] I_frame_cnt,     // 显示帧计数（低 16 位）

    input  wire [23:0] I_data,
    output wire [23:0] O_data
);

    //------------------------------------------------------------------
    // 中文字模编码（与 osd_font_rom 生成脚本一致）
    //------------------------------------------------------------------
    localparam [7:0] C_YUAN = 8'h80, C_TU   = 8'h81,
                     C_BIAN = 8'h82, C_YUAN2= 8'h83,
                     C_JIAN = 8'h84, C_CE   = 8'h85,
                     C_HUI  = 8'h86, C_DU   = 8'h87,
                     C_BIAN2= 8'h88, C_HUAN = 8'h89,
                     C_ER   = 8'h8A, C_ZHI  = 8'h8B,
                     C_HUA  = 8'h8C, C_ZHONG= 8'h8D,
                     C_LV   = 8'h8E, C_BO   = 8'h8F,
                     C_ZHI2 = 8'h90, C_FANG = 8'h91,
                     C_JUN  = 8'h92, C_HENG = 8'h93;

    localparam [23:0] NAVY_COL = 24'h1B2A4A;
    localparam [23:0] RED_COL  = 24'hD71920;
    localparam [23:0] WHITE_COL= 24'hFFFFFF;

    //------------------------------------------------------------------
    // 光栅坐标（与数据同拍）
    //------------------------------------------------------------------
    reg [11:0] S_x;
    reg [11:0] S_y;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end else if (I_vs) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end else if (I_de) begin
            if (S_x == (H_ACTIVE - 1)) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // 区域判断
    //------------------------------------------------------------------
    wire in_box = (S_x >= BOX_X) && (S_x < (BOX_X + BOX_W)) &&
                  (S_y >= BOX_Y) && (S_y < (BOX_Y + BOX_H));

    wire [11:0] lx = S_x - LOGO_X;
    wire [11:0] ly = S_y - LOGO_Y;
    wire in_logo = (S_x >= LOGO_X) && (S_x < (LOGO_X + LOGO_W)) &&
                   (S_y >= LOGO_Y) && (S_y < (LOGO_Y + LOGO_H));

    wire in_box_border = in_box &&
                         ((S_x < (BOX_X + 2)) || (S_x >= (BOX_X + BOX_W - 2)) ||
                          (S_y < (BOX_Y + 2)) || (S_y >= (BOX_Y + BOX_H - 2)));

    wire in_logo_border = in_logo &&
                          ((lx < 1) || (lx >= (LOGO_W - 1)) ||
                           (ly < 1) || (ly >= (LOGO_H - 1)));

    wire [11:0] dx = S_x - TEXT_X;
    wire [11:0] dy_mode = S_y - MODE_Y;
    wire [11:0] dy_l1   = S_y - LINE1_Y;
    wire [11:0] dy_l2   = S_y - LINE2_Y;

    wire in_mode_line = (S_y >= MODE_Y) && (S_y < (MODE_Y + 32)) &&
                        (S_x >= TEXT_X) && (S_x < (TEXT_X + 5 * 32));
    wire in_line1 = (S_y >= LINE1_Y) && (S_y < (LINE1_Y + 16)) &&
                    (S_x >= TEXT_X) && (S_x < (TEXT_X + 15 * 16));
    wire in_line2 = (S_y >= LINE2_Y) && (S_y < (LINE2_Y + 16)) &&
                    (S_x >= TEXT_X) && (S_x < (TEXT_X + 21 * 16));

    wire [4:0] mode_col = dx[8:5];       // 32px 字符
    wire [4:0] mode_gx  = dx[4:1];       // 0..15
    wire [4:0] mode_gy  = dy_mode[4:1];  // 0..15

    wire [5:0] line_col = dx[8:4];       // 16px 字符
    wire [3:0] line_gx  = dx[3:0];
    wire [3:0] line_gy1 = dy_l1[3:0];
    wire [3:0] line_gy2 = dy_l2[3:0];

    //------------------------------------------------------------------
    // 模式文字
    //------------------------------------------------------------------
    reg [7:0] M0, M1, M2, M3, M4;

    always @(*) begin
        M0 = 8'h20; M1 = 8'h20; M2 = 8'h20; M3 = 8'h20; M4 = 8'h20;
        case (I_mode)
            3'd0: begin M0 = C_YUAN;  M1 = C_TU;   end
            3'd1: begin M0 = C_BIAN;  M1 = C_YUAN2; M2 = C_JIAN; M3 = C_CE; end
            3'd2: begin M0 = C_HUI;   M1 = C_DU;   M2 = C_BIAN2; M3 = C_HUAN; end
            3'd3: begin M0 = C_ER;    M1 = C_ZHI;  M2 = C_HUA;  end
            3'd4: begin M0 = C_ZHONG; M1 = C_ZHI;  M2 = C_LV;   M3 = C_BO; end
            3'd5: begin M0 = C_ZHI2;  M1 = C_FANG; M2 = C_TU;   M3 = C_JUN; M4 = C_HENG; end
            default: begin M0 = 8'h20; end
        endcase
    end

    //------------------------------------------------------------------
    // 十进制数字
    //------------------------------------------------------------------
    wire [19:0] frames_bcd;
    wire [19:0] fpga_ms_bcd;
    wire [19:0] host_ms_bcd;
    wire [19:0] frame_bcd;

    wire [15:0] host_ms_clamped = (I_host_ms > 16'd999) ? 16'd999 : I_host_ms;

    bin2bcd16 u_frames_bcd  (.I_bin ({8'd0, I_fpga_frames}), .O_bcd (frames_bcd));
    bin2bcd16 u_fpga_ms_bcd (.I_bin (I_fpga_ms),             .O_bcd (fpga_ms_bcd));
    bin2bcd16 u_host_bcd    (.I_bin (host_ms_clamped),       .O_bcd (host_ms_bcd));
    bin2bcd16 u_frame_bcd   (.I_bin (I_frame_cnt),           .O_bcd (frame_bcd));

    // 十进制转换是 16 级双 dabble 组合逻辑，直接接字模 ROM 会成为
    // 75MHz 下的长路径。这里打一拍；数字只按帧更新，晚一拍不可见。
    reg [19:0] frames_bcd_r;
    reg [19:0] fpga_ms_bcd_r;
    reg [19:0] host_ms_bcd_r;
    reg [19:0] frame_bcd_r;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            frames_bcd_r  <= 20'd0;
            fpga_ms_bcd_r <= 20'd0;
            host_ms_bcd_r <= 20'd0;
            frame_bcd_r   <= 20'd0;
        end else begin
            frames_bcd_r  <= frames_bcd;
            fpga_ms_bcd_r <= fpga_ms_bcd;
            host_ms_bcd_r <= host_ms_bcd;
            frame_bcd_r   <= frame_bcd;
        end
    end

    wire [7:0] d_frm_h = {4'd0, frames_bcd_r[11:8]} + 8'h30;
    wire [7:0] d_frm_t = {4'd0, frames_bcd_r[7:4]}  + 8'h30;
    wire [7:0] d_frm_o = {4'd0, frames_bcd_r[3:0]}  + 8'h30;

    wire [7:0] d_ms_h  = {4'd0, fpga_ms_bcd_r[11:8]} + 8'h30;
    wire [7:0] d_ms_t  = {4'd0, fpga_ms_bcd_r[7:4]}  + 8'h30;
    wire [7:0] d_ms_o  = {4'd0, fpga_ms_bcd_r[3:0]}  + 8'h30;

    wire [7:0] d_host_h = {4'd0, host_ms_bcd_r[11:8]} + 8'h30;
    wire [7:0] d_host_t = {4'd0, host_ms_bcd_r[7:4]}  + 8'h30;
    wire [7:0] d_host_o = {4'd0, host_ms_bcd_r[3:0]}  + 8'h30;

    wire [7:0] d_cnt_4 = {4'd0, frame_bcd_r[19:16]} + 8'h30;
    wire [7:0] d_cnt_3 = {4'd0, frame_bcd_r[15:12]} + 8'h30;
    wire [7:0] d_cnt_2 = {4'd0, frame_bcd_r[11:8]}  + 8'h30;
    wire [7:0] d_cnt_1 = {4'd0, frame_bcd_r[7:4]}   + 8'h30;
    wire [7:0] d_cnt_0 = {4'd0, frame_bcd_r[3:0]}   + 8'h30;

    //------------------------------------------------------------------
    // 按位置选字符
    //------------------------------------------------------------------
    reg [7:0] ch;

    always @(*) begin
        ch = 8'h20;
        if (in_mode_line) begin
            case (mode_col)
                5'd0: ch = M0;
                5'd1: ch = M1;
                5'd2: ch = M2;
                5'd3: ch = M3;
                5'd4: ch = M4;
                default: ch = 8'h20;
            endcase
        end else if (in_line1) begin
            case (line_col)
                6'd0:  ch = "F";
                6'd1:  ch = "P";
                6'd2:  ch = "G";
                6'd3:  ch = "A";
                6'd4:  ch = " ";
                6'd5:  ch = d_frm_h;
                6'd6:  ch = d_frm_t;
                6'd7:  ch = d_frm_o;
                6'd8:  ch = "F";
                6'd9:  ch = " ";
                6'd10: ch = d_ms_h;
                6'd11: ch = d_ms_t;
                6'd12: ch = d_ms_o;
                6'd13: ch = "M";
                6'd14: ch = "S";
                default: ch = 8'h20;
            endcase
        end else if (in_line2) begin
            case (line_col)
                6'd0:  ch = "H";
                6'd1:  ch = "O";
                6'd2:  ch = "S";
                6'd3:  ch = "T";
                6'd4:  ch = " ";
                6'd5:  ch = d_host_h;
                6'd6:  ch = d_host_t;
                6'd7:  ch = d_host_o;
                6'd8:  ch = "M";
                6'd9:  ch = "S";
                6'd10: ch = " ";
                6'd11: ch = " ";
                6'd12: ch = "F";
                6'd13: ch = "R";
                6'd14: ch = "M";
                6'd15: ch = " ";
                6'd16: ch = d_cnt_4;
                6'd17: ch = d_cnt_3;
                6'd18: ch = d_cnt_2;
                6'd19: ch = d_cnt_1;
                6'd20: ch = d_cnt_0;
                default: ch = 8'h20;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 字模
    //------------------------------------------------------------------
    wire [3:0] glyph_gx = in_mode_line ? mode_gx[3:0] : line_gx;
    wire [3:0] glyph_gy = in_mode_line ? mode_gy[3:0] :
                          ((S_y >= LINE2_Y) ? line_gy2 : line_gy1);

    wire [15:0] font_row;
    osd_font_rom u_font_rom (
        .I_ch  (ch),
        .I_y   (glyph_gy),
        .O_row (font_row)
    );

    wire pix_on = font_row[4'd15 - glyph_gx];

    //------------------------------------------------------------------
    // Logo 点阵
    //------------------------------------------------------------------
    wire [319:0] logo_row;
    osd_logo_rom u_logo_rom (
        .I_y   (ly[5:0]),
        .O_row (logo_row)
    );

    wire [1:0] logo_pix = logo_row[319 - (lx[7:0] << 1) -: 2];

    wire [23:0] logo_color = (logo_pix == 2'b01) ? NAVY_COL :
                             (logo_pix == 2'b10) ? RED_COL  : WHITE_COL;

    //------------------------------------------------------------------
    // 组合叠加
    //------------------------------------------------------------------
    wire in_text = in_mode_line || in_line1 || in_line2;

    assign O_data = !in_box        ? I_data :
                    in_box_border  ? 24'hE8E8E8 :
                    in_logo        ? (in_logo_border ? NAVY_COL : logo_color) :
                    (in_text && pix_on) ? FG_COLOR : BG_COLOR;

endmodule
