//=============================================================================
//  bin2bcd16 —— 16bit 二进制转 5 位 BCD（组合，双 dabble）
//  仅用于 OSD 数字显示；结果 bcd[19:16]=万位 ... bcd[3:0]=个位。
//=============================================================================
module bin2bcd16 (
    input  wire [15:0] I_bin,
    output reg  [19:0] O_bcd
);
    reg [35:0] shift;
    integer i;

    always @(*) begin
        shift = 36'd0;
        shift[15:0] = I_bin;
        for (i = 0; i < 16; i = i + 1) begin
            if (shift[19:16] >= 4'd5) shift[19:16] = shift[19:16] + 4'd3;
            if (shift[23:20] >= 4'd5) shift[23:20] = shift[23:20] + 4'd3;
            if (shift[27:24] >= 4'd5) shift[27:24] = shift[27:24] + 4'd3;
            if (shift[31:28] >= 4'd5) shift[31:28] = shift[31:28] + 4'd3;
            if (shift[35:32] >= 4'd5) shift[35:32] = shift[35:32] + 4'd3;
            shift = shift << 1;
        end
        O_bcd = shift[35:16];
    end
endmodule
