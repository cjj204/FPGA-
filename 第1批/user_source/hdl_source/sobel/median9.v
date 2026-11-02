//=============================================================================
//  median9 —— 3x3 中值滤波（9 输入中值，4 级流水）
//
//  25 比较器排序网络（0/1 原理穷举验证），取第 5 个元素作为中值。
//  75MHz 下单级超过约 7 个比较器就会成为长路径，故拆成 6/6/6/7 四级：
//    stage1(6) -> p1 -> stage2(6) -> p2 -> stage3(6) -> p3 -> stage4(7) -> O_median
//  相对 3x3 窗口寄存器总延迟 4 拍。
//  上层 SOBEL_DLY 相应为 H_TOTAL+7，灰度/边缘也补到 4 拍，保持对齐。
//  无复位，保证流水线延迟恒定。
//=============================================================================
module median9 (
    input  wire       clk,
    input  wire [7:0] I_a0,
    input  wire [7:0] I_a1,
    input  wire [7:0] I_a2,
    input  wire [7:0] I_a3,
    input  wire [7:0] I_a4,
    input  wire [7:0] I_a5,
    input  wire [7:0] I_a6,
    input  wire [7:0] I_a7,
    input  wire [7:0] I_a8,
    output reg  [7:0] O_median
);

    //---------------- stage 1 : 比较器 1~6 ----------------
    reg [7:0] s0, s1, s2, s3, s4, s5, s6, s7, s8;
    reg [7:0] t1;
    always @(*) begin
        t1 = 8'd0;
        s0 = I_a0;
        s1 = I_a1;
        s2 = I_a2;
        s3 = I_a3;
        s4 = I_a4;
        s5 = I_a5;
        s6 = I_a6;
        s7 = I_a7;
        s8 = I_a8;

        if (s0 > s1) begin t1 = s0; s0 = s1; s1 = t1; end
        if (s3 > s4) begin t1 = s3; s3 = s4; s4 = t1; end
        if (s6 > s7) begin t1 = s6; s6 = s7; s7 = t1; end
        if (s1 > s2) begin t1 = s1; s1 = s2; s2 = t1; end
        if (s4 > s5) begin t1 = s4; s4 = s5; s5 = t1; end
        if (s7 > s8) begin t1 = s7; s7 = s8; s8 = t1; end
    end

    reg [7:0] p0, p1, p2, p3, p4, p5, p6, p7, p8;
    always @(posedge clk) begin
        p0 <= s0;
        p1 <= s1;
        p2 <= s2;
        p3 <= s3;
        p4 <= s4;
        p5 <= s5;
        p6 <= s6;
        p7 <= s7;
        p8 <= s8;
    end

    //---------------- stage 2 : 比较器 7~12 ----------------
    reg [7:0] q0, q1, q2, q3, q4, q5, q6, q7, q8;
    reg [7:0] t2;
    always @(*) begin
        t2 = 8'd0;
        q0 = p0;
        q1 = p1;
        q2 = p2;
        q3 = p3;
        q4 = p4;
        q5 = p5;
        q6 = p6;
        q7 = p7;
        q8 = p8;

        if (q0 > q1) begin t2 = q0; q0 = q1; q1 = t2; end
        if (q3 > q4) begin t2 = q3; q3 = q4; q4 = t2; end
        if (q6 > q7) begin t2 = q6; q6 = q7; q7 = t2; end
        if (q0 > q3) begin t2 = q0; q0 = q3; q3 = t2; end
        if (q3 > q6) begin t2 = q3; q3 = q6; q6 = t2; end
        if (q0 > q3) begin t2 = q0; q0 = q3; q3 = t2; end
    end

    reg [7:0] r0, r1, r2, r3, r4, r5, r6, r7, r8;
    always @(posedge clk) begin
        r0 <= q0;
        r1 <= q1;
        r2 <= q2;
        r3 <= q3;
        r4 <= q4;
        r5 <= q5;
        r6 <= q6;
        r7 <= q7;
        r8 <= q8;
    end

    //---------------- stage 3 : 比较器 13~18 ----------------
    reg [7:0] u0, u1, u2, u3, u4, u5, u6, u7, u8;
    reg [7:0] t3;
    always @(*) begin
        t3 = 8'd0;
        u0 = r0;
        u1 = r1;
        u2 = r2;
        u3 = r3;
        u4 = r4;
        u5 = r5;
        u6 = r6;
        u7 = r7;
        u8 = r8;

        if (u1 > u4) begin t3 = u1; u1 = u4; u4 = t3; end
        if (u4 > u7) begin t3 = u4; u4 = u7; u7 = t3; end
        if (u1 > u4) begin t3 = u1; u1 = u4; u4 = t3; end
        if (u2 > u5) begin t3 = u2; u2 = u5; u5 = t3; end
        if (u5 > u8) begin t3 = u5; u5 = u8; u8 = t3; end
        if (u2 > u5) begin t3 = u2; u2 = u5; u5 = t3; end
    end

    reg [7:0] v0, v1, v2, v3, v4, v5, v6, v7, v8;
    always @(posedge clk) begin
        v0 <= u0;
        v1 <= u1;
        v2 <= u2;
        v3 <= u3;
        v4 <= u4;
        v5 <= u5;
        v6 <= u6;
        v7 <= u7;
        v8 <= u8;
    end

    //---------------- stage 4 : 比较器 19~25 ----------------
    reg [7:0] w0, w1, w2, w3, w4, w5, w6, w7, w8;
    reg [7:0] t4;
    always @(*) begin
        t4 = 8'd0;
        w0 = v0;
        w1 = v1;
        w2 = v2;
        w3 = v3;
        w4 = v4;
        w5 = v5;
        w6 = v6;
        w7 = v7;
        w8 = v8;

        if (w1 > w3) begin t4 = w1; w1 = w3; w3 = t4; end
        if (w5 > w7) begin t4 = w5; w5 = w7; w7 = t4; end
        if (w2 > w6) begin t4 = w2; w2 = w6; w6 = t4; end
        if (w4 > w6) begin t4 = w4; w4 = w6; w6 = t4; end
        if (w2 > w4) begin t4 = w2; w2 = w4; w4 = t4; end
        if (w2 > w3) begin t4 = w2; w2 = w3; w3 = t4; end
        if (w5 > w6) begin t4 = w5; w5 = w6; w6 = t4; end
    end

    always @(posedge clk)
        O_median <= w4;

endmodule