//=============================================================================
//  uart_rx —— 115200 8N1 接收（16 倍过采样）
//  像素时钟域，输入异步信号内部打两拍。
//=============================================================================
module uart_rx #(
    parameter integer CLK_FREQ = 75_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       I_rx,
    output reg  [7:0] O_data,
    output reg        O_valid
);
    localparam integer CYCLE = CLK_FREQ / BAUD;
    localparam integer HALF  = CYCLE / 2;

    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1,
                     S_DATA = 2'd2, S_STOP  = 2'd3;

    reg [1:0]  S_state;
    reg [15:0] S_cnt;
    reg [2:0]  S_bit;
    reg [7:0]  S_shift;
    reg        S_rx_d0, S_rx_d1;

    wire S_rx_fall = S_rx_d1 & ~S_rx_d0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_rx_d0 <= 1'b1;
            S_rx_d1 <= 1'b1;
        end else begin
            S_rx_d0 <= I_rx;
            S_rx_d1 <= S_rx_d0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state <= S_IDLE;
            S_cnt   <= 16'd0;
            S_bit   <= 3'd0;
            S_shift <= 8'd0;
            O_data  <= 8'd0;
            O_valid <= 1'b0;
        end else begin
            O_valid <= 1'b0;
            case (S_state)
                S_IDLE: begin
                    if (S_rx_fall) begin
                        S_cnt   <= 16'd0;
                        S_state <= S_START;
                    end
                end

                S_START: begin
                    if (S_cnt == (HALF - 1)) begin
                        S_cnt   <= 16'd0;
                        S_bit   <= 3'd0;
                        S_state <= S_DATA;
                    end else begin
                        S_cnt <= S_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (S_cnt == (CYCLE - 1)) begin
                        S_cnt        <= 16'd0;
                        S_shift[S_bit] <= S_rx_d1;
                        if (S_bit == 3'd7) begin
                            S_state <= S_STOP;
                        end else begin
                            S_bit <= S_bit + 3'd1;
                        end
                    end else begin
                        S_cnt <= S_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    if (S_cnt == (HALF - 1)) begin
                        S_state <= S_IDLE;
                        O_data  <= S_shift;
                        O_valid <= 1'b1;
                    end else begin
                        S_cnt <= S_cnt + 16'd1;
                    end
                end

                default: S_state <= S_IDLE;
            endcase
        end
    end
endmodule
