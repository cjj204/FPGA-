//========================================================================
// 子模块1：灰度提取
//   GRAY_MODE = 0 : 直接取 R 分量（最省资源）
//   GRAY_MODE = 1 : 加权亮度 Y = 0.299R + 0.587G + 0.114B
//                   定点近似 (77R + 150G + 29B) >> 8
//========================================================================
module gray_extract #(
    parameter GRAY_MODE = 1
)(
    input               pclk,
    input               vid_active_video,   // 有效视频标志
    input       [23:0]  vid_data,           // RGB888
    output reg  [7:0]   gray,               // 灰度输出
    output reg          gray_valid          // 灰度有效（与 gray 对齐）
);

    wire [7:0] r = vid_data[23:16];
    wire [7:0] g = vid_data[15:8];
    wire [7:0] b = vid_data[7:0];

    // 加权亮度：77R + 150G + 29B，位宽扩展到 18 位防溢出
    wire [17:0] y_sum = (r * 18'd77) + (g * 18'd150) + (b * 18'd29);
    wire [7:0]  y_fix = y_sum[15:8];   // 相当于 >> 8

    always @(posedge pclk) begin
        if (GRAY_MODE == 1)
            gray <= y_fix;
        else
            gray <= r;                 // 模式0：直接取R

        gray_valid <= vid_active_video; // 与 gray 同拍对齐
    end

endmodule