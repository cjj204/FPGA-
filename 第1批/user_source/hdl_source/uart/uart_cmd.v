//=============================================================================
//  uart_cmd —— 串口指令解析
//
//  支持：
//    a / A / 0  -> 原图          b / B / 1  -> 边缘检测
//    c / C / 2  -> 灰度变换      d / D / 3  -> 二值化
//    e / E / 4  -> 中值滤波      f / F / 5  -> 直方图均衡
//    + / -      -> 阈值档位 +1 / -1（0..3 回绕）
//    Hxxxx      -> 上位机延迟（4 位十六进制，单位 ms；例如 H000C=12ms）
//    ?          -> 请求立即回传状态
//    R          -> 清帧计数/延迟统计
//    CR/LF/空格 -> 忽略
//
//  所有输出均为像素时钟域的单周期脉冲（O_*_we）。
//=============================================================================
module uart_cmd (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       I_rx_valid,
    input  wire [7:0] I_rx_data,

    output reg  [2:0]  O_mode,
    output reg         O_mode_we,
    output reg  [1:0]  O_level,
    output reg         O_level_we,
    output reg  [15:0] O_host_ms,
    output reg         O_host_we,
    output reg         O_status_req,
    output reg         O_reset_req
);

    reg        S_hex_active;
    reg [1:0]  S_hex_idx;
    reg [15:0] S_hex_val;

    function [3:0] hex_nibble;
        input [7:0] c;
        begin
            if      (c >= "0" && c <= "9") hex_nibble = c - "0";
            else if (c >= "A" && c <= "F") hex_nibble = c - "A" + 4'd10;
            else if (c >= "a" && c <= "f") hex_nibble = c - "a" + 4'd10;
            else                            hex_nibble = 4'd0;
        end
    endfunction

    function is_hex;
        input [7:0] c;
        begin
            is_hex = ((c >= "0" && c <= "9") ||
                      (c >= "A" && c <= "F") ||
                      (c >= "a" && c <= "f"));
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            O_mode       <= 3'd0;
            O_mode_we    <= 1'b0;
            O_level      <= 2'd1;
            O_level_we   <= 1'b0;
            O_host_ms    <= 16'd0;
            O_host_we    <= 1'b0;
            O_status_req <= 1'b0;
            O_reset_req  <= 1'b0;
            S_hex_active <= 1'b0;
            S_hex_idx    <= 2'd0;
            S_hex_val    <= 16'd0;
        end else begin
            O_mode_we    <= 1'b0;
            O_level_we   <= 1'b0;
            O_host_we    <= 1'b0;
            O_status_req <= 1'b0;
            O_reset_req  <= 1'b0;

            if (I_rx_valid) begin
                if (S_hex_active) begin
                    if (is_hex(I_rx_data)) begin
                        S_hex_val <= {S_hex_val[11:0], hex_nibble(I_rx_data)};
                        if (S_hex_idx == 2'd3) begin
                            O_host_ms    <= {S_hex_val[11:0], hex_nibble(I_rx_data)};
                            O_host_we    <= 1'b1;
                            O_status_req <= 1'b1;
                            S_hex_active <= 1'b0;
                        end else begin
                            S_hex_idx <= S_hex_idx + 2'd1;
                        end
                    end else begin
                        S_hex_active <= 1'b0;
                    end
                end else begin
                    case (I_rx_data)
                        "a", "A", "0": begin O_mode <= 3'd0; O_mode_we <= 1'b1; end
                        "b", "B", "1": begin O_mode <= 3'd1; O_mode_we <= 1'b1; end
                        "c", "C", "2": begin O_mode <= 3'd2; O_mode_we <= 1'b1; end
                        "d", "D", "3": begin O_mode <= 3'd3; O_mode_we <= 1'b1; end
                        "e", "E", "4": begin O_mode <= 3'd4; O_mode_we <= 1'b1; end
                        "f", "F", "5": begin O_mode <= 3'd5; O_mode_we <= 1'b1; end

                        "H", "h": begin
                            S_hex_active <= 1'b1;
                            S_hex_idx    <= 2'd0;
                            S_hex_val    <= 16'd0;
                        end

                        "+": begin O_level <= O_level + 2'd1; O_level_we <= 1'b1; end
                        "-": begin O_level <= O_level - 2'd1; O_level_we <= 1'b1; end

                        "?": O_status_req <= 1'b1;
                        "R", "r": O_reset_req <= 1'b1;

                        default: ; // CR/LF/空格/未知字符忽略
                    endcase
                end
            end
        end
    end
endmodule
