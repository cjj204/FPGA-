//=============================================================================
//  uart_tx —— 115200 8N1 发送
//=============================================================================
module uart_tx #(
    parameter integer CLK_FREQ = 75_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] I_data,
    input  wire       I_valid,
    output reg        O_ready,
    output reg        O_tx
);
    localparam integer CYCLE = CLK_FREQ / BAUD;

    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1,
                     S_DATA = 2'd2, S_STOP  = 2'd3;

    reg [1:0]  S_state;
    reg [15:0] S_cnt;
    reg [2:0]  S_bit;
    reg [7:0]  S_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state <= S_IDLE;
            S_cnt   <= 16'd0;
            S_bit   <= 3'd0;
            S_data  <= 8'd0;
            O_ready <= 1'b1;
            O_tx    <= 1'b1;
        end else begin
            case (S_state)
                S_IDLE: begin
                    O_tx <= 1'b1;
                    if (I_valid) begin
                        S_data  <= I_data;
                        S_cnt   <= 16'd0;
                        S_bit   <= 3'd0;
                        O_ready <= 1'b0;
                        S_state <= S_START;
                    end else begin
                        O_ready <= 1'b1;
                    end
                end

                S_START: begin
                    O_tx <= 1'b0;
                    if (S_cnt == (CYCLE - 1)) begin
                        S_cnt   <= 16'd0;
                        S_state <= S_DATA;
                    end else begin
                        S_cnt <= S_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    O_tx <= S_data[S_bit];
                    if (S_cnt == (CYCLE - 1)) begin
                        S_cnt <= 16'd0;
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
                    O_tx <= 1'b1;
                    if (S_cnt == (CYCLE - 1)) begin
                        S_cnt   <= 16'd0;
                        O_ready <= 1'b1;
                        S_state <= S_IDLE;
                    end else begin
                        S_cnt <= S_cnt + 16'd1;
                    end
                end

                default: S_state <= S_IDLE;
            endcase
        end
    end
endmodule
