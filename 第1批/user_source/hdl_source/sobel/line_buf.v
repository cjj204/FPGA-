//========================================================================
// 行缓存模块
//========================================================================
module line_buf #(
    parameter WIDTH = 8,
    parameter DEPTH = 800
)(
    input                   clk,
    input                   rst_n,
    input                   we,
    input       [WIDTH-1:0] din,
    output reg  [WIDTH-1:0] dout
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [15:0]      wr_ptr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 16'd0;
            dout   <= {WIDTH{1'b0}};
        end else if (we) begin
            mem[wr_ptr] <= din;
            dout        <= mem[wr_ptr];   // 读旧值，实现整行延迟
            wr_ptr      <= (wr_ptr == DEPTH-1) ? 16'd0 : wr_ptr + 1'b1;
        end
    end

endmodule