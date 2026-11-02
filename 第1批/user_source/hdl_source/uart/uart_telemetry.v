//=============================================================================
//  uart_telemetry —— 状态回传（v3：字母安全 + 实测帧周期）
//
//  帧格式（ASCII，32 字节，固定长度）：
//    "$TK,MM,FF,GG,PP,HHHH,CCCC,L*XX" + CR + LF

//      MM   : 模式 00..05
//      FF   : FPGA 采集->显示延迟（帧）00..0F
//      GG   : 对应毫秒 00..FF
//      PP   : 【新增】实测帧周期（毫秒）00..FF，60fps≈11，30fps≈21
//      HHHH : 上位机延迟（ms，十六进制，回显 H 指令）
//      CCCC : 显示帧计数低 16 位
//      L    : 阈值档位 0..3
//      XX   : 从 'L' 起到 '*' 前（不含 '*'）的逐字节 XOR
//
//  【字母安全】数字用 0-9 与 G-M（不用 A-F）：
//    老上位机把收到的任何 a~f 当模式指令（先 tolower 再过滤），
//    遥测里只要有 A-F 就会疯狂误触发。改成 G-M 后整帧不含 a~f，
//    老上位机一个假指令都不会收到。
//  兼容性：字段数 7（旧，无帧周期）/ 8（新，带帧周期）自动区分；
//          字母表 0-9A-F（v1/v2）/ 0-9G-M（v3）两种都认。
//
//  发送使能为单周期脉冲；忙时忽略新的请求（周期回传会补上）。
//=============================================================================
module uart_telemetry (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        I_send,
    input  wire [2:0]  I_mode,
    input  wire [7:0]  I_fpga_frames,
    input  wire [15:0] I_fpga_ms,
    input  wire [15:0] I_period_ms,       // 实测帧周期（ms）
    input  wire [15:0] I_host_ms,
    input  wire [15:0] I_frame_cnt,
    input  wire [1:0]  I_level,

    output wire        O_busy,
    output wire        O_uart_tx
);

    localparam integer LEN = 32;

    reg [7:0]  S_msg [0:LEN-1];
    reg [4:0]  S_idx;
    reg [1:0]  S_state;
    reg        S_busy;
    reg        S_pend;
    reg [7:0]  S_tx_data;
    reg        S_tx_valid;
    wire       S_tx_ready;

    localparam [1:0] S_IDLE = 2'd0, S_SEND = 2'd1, S_WAIT = 2'd2;

    function [7:0] hexchar;
        input [3:0] n;
        begin
            hexchar = (n < 4'd10) ? ("0" + n) : ("G" + n - 4'd10);
        end
    endfunction

    uart_tx #(
        .CLK_FREQ (74_250_000),      // 实际像素时钟 74.25MHz（原来写 75MHz，波特率偏 1%）
        .BAUD     (115200)
    ) u_uart_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .I_data   (S_tx_data),
        .I_valid  (S_tx_valid),
        .O_ready  (S_tx_ready),
        .O_tx     (O_uart_tx)
    );

    assign O_busy = S_busy;

    // 毫秒/帧周期只有 8 位字段：超过 255 饱和到 255（v1 取的是高字节，<256 时永远报 00）
    wire [7:0] ms_b  = (I_fpga_ms   > 16'd255) ? 8'hFF : I_fpga_ms[7:0];
    wire [7:0] per_b = (I_period_ms > 16'd255) ? 8'hFF : I_period_ms[7:0];

    integer i;
    reg [7:0] chk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state    <= S_IDLE;
            S_idx      <= 5'd0;
            S_busy     <= 1'b0;
            S_tx_data  <= 8'd0;
            S_tx_valid <= 1'b0;
        end else begin
            case (S_state)
                S_IDLE: begin
                    S_tx_valid <= 1'b0;
                    S_pend     <= 1'b0;
                    if (I_send) begin
                        S_msg[0]  <= "$";
                        S_msg[1]  <= "T";
                        S_msg[2]  <= "K";
                        S_msg[3]  <= ",";
                        S_msg[4]  <= "0";
                        S_msg[5]  <= hexchar({1'b0, I_mode});
                        S_msg[6]  <= ",";
                        S_msg[7]  <= hexchar(I_fpga_frames[7:4]);
                        S_msg[8]  <= hexchar(I_fpga_frames[3:0]);
                        S_msg[9]  <= ",";
                        S_msg[10] <= hexchar(ms_b[7:4]);
                        S_msg[11] <= hexchar(ms_b[3:0]);
                        S_msg[12] <= ",";
                        // ---- 新增：实测帧周期 PP ----
                        S_msg[13] <= hexchar(per_b[7:4]);
                        S_msg[14] <= hexchar(per_b[3:0]);
                        S_msg[15] <= ",";
                        // ---- 上位机延迟 HHHH ----
                        S_msg[16] <= hexchar(I_host_ms[15:12]);
                        S_msg[17] <= hexchar(I_host_ms[11:8]);
                        S_msg[18] <= hexchar(I_host_ms[7:4]);
                        S_msg[19] <= hexchar(I_host_ms[3:0]);
                        S_msg[20] <= ",";
                        // ---- 帧计数 CCCC ----
                        S_msg[21] <= hexchar(I_frame_cnt[15:12]);
                        S_msg[22] <= hexchar(I_frame_cnt[11:8]);
                        S_msg[23] <= hexchar(I_frame_cnt[7:4]);
                        S_msg[24] <= hexchar(I_frame_cnt[3:0]);
                        S_msg[25] <= ",";
                        // ---- 阈值档位 L ----
                        S_msg[26] <= hexchar({2'b00, I_level});
                        S_msg[27] <= "*";

                        // 校验：'L' 起（含）到 '*' 前逐字节 XOR，用当前输入直接算
                        chk = 8'd0;
                        chk = chk ^ "T";
                        chk = chk ^ "K";
                        chk = chk ^ ",";
                        chk = chk ^ "0";
                        chk = chk ^ hexchar({1'b0, I_mode});
                        chk = chk ^ ",";
                        chk = chk ^ hexchar(I_fpga_frames[7:4]);
                        chk = chk ^ hexchar(I_fpga_frames[3:0]);
                        chk = chk ^ ",";
                        chk = chk ^ hexchar(ms_b[7:4]);
                        chk = chk ^ hexchar(ms_b[3:0]);
                        chk = chk ^ ",";
                        chk = chk ^ hexchar(per_b[7:4]);
                        chk = chk ^ hexchar(per_b[3:0]);
                        chk = chk ^ ",";
                        chk = chk ^ hexchar(I_host_ms[15:12]);
                        chk = chk ^ hexchar(I_host_ms[11:8]);
                        chk = chk ^ hexchar(I_host_ms[7:4]);
                        chk = chk ^ hexchar(I_host_ms[3:0]);
                        chk = chk ^ ",";
                        chk = chk ^ hexchar(I_frame_cnt[15:12]);
                        chk = chk ^ hexchar(I_frame_cnt[11:8]);
                        chk = chk ^ hexchar(I_frame_cnt[7:4]);
                        chk = chk ^ hexchar(I_frame_cnt[3:0]);
                        chk = chk ^ ",";
                        chk = chk ^ hexchar({2'b00, I_level});

                        S_msg[28] <= hexchar(chk[7:4]);
                        S_msg[29] <= hexchar(chk[3:0]);
                        S_msg[30] <= 8'h0D;
                        S_msg[31] <= 8'h0A;

                        S_idx   <= 5'd0;
                        S_busy  <= 1'b1;
                        S_state <= S_SEND;
                    end
                end

                //------------------------------------------------------------------
                // 【关键修复】原来这里是 "若 ready 就发下一字节"，
                // 但 uart_tx 收到字节的下一拍 ready 仍为 1（要到再下一拍才拉低），
                // 于是每发一个字节就立刻又灌一个，被 uart_tx 忽略 ——
                // 结果是隔一个字节丢一个，32 字节的帧只剩 16 字节，
                // 上位机看到的"莫名指令"其实就是这种断续帧。
                // 现在改成标准握手：valid 保持到 ready 拉低（= 已被吃掉）再推进。
                //------------------------------------------------------------------
                S_SEND: begin
                    if (!S_pend) begin
                        if (S_tx_ready) begin          // 只在 uart_tx 空闲时提交本字节
                            S_tx_data  <= S_msg[S_idx];
                            S_tx_valid <= 1'b1;
                            S_pend     <= 1'b1;
                        end
                    end else if (!S_tx_ready) begin     // ready 拉低 = 已被 uart_tx 取走
                        S_tx_valid <= 1'b0;
                        S_pend     <= 1'b0;
                        if (S_idx == (LEN - 1)) begin
                            S_state <= S_WAIT;
                        end else begin
                            S_idx <= S_idx + 5'd1;
                        end
                    end
                end

                S_WAIT: begin
                    S_tx_valid <= 1'b0;
                    if (S_tx_ready) begin
                        S_busy  <= 1'b0;
                        S_state <= S_IDLE;
                    end
                end

                default: S_state <= S_IDLE;
            endcase
        end
    end
endmodule
