module pwm_gen #(
    parameter CLK_HZ = 50_000_000,
    parameter PWM_HZ = 20_000
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              en,
    input  wire signed [7:0] u,
    output reg               pwma,
    output reg               ain1,
    output reg               ain2
);
    localparam integer PERIOD = CLK_HZ / PWM_HZ;
    localparam integer STEP   = PERIOD / 100;

    // 幅值 |u| 显式饱和到 100（不依赖 duty>=PERIOD 恰好恒高这一参数巧合）
    wire        neg   = u[7];
    wire [7:0]  mag   = neg ? -u : u;               
    wire [7:0]  mag_s = (mag > 8'd100) ? 8'd100 : mag;

    // u 与方向只在周期边界锁存：当前周期不换向、不出残缺脉冲
    reg  [15:0] cnt;
    reg  [15:0] duty_l;
    reg         neg_l;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= 16'd0;
            duty_l <= 16'd0;
            neg_l  <= 1'b0;
            pwma   <= 1'b0;
            ain1   <= 1'b0;
            ain2   <= 1'b0;
        end else begin
            if (cnt == PERIOD - 1) begin
                cnt    <= 16'd0;
                duty_l <= mag_s * STEP;
                neg_l  <= neg;
            end else begin
                cnt <= cnt + 1'b1;
            end
            if (!en) begin
                pwma <= 1'b0;
                ain1 <= 1'b0;
                ain2 <= 1'b0;
            end else begin
                pwma <= (cnt < duty_l);
                ain1 <= neg_l;
                ain2 <= ~neg_l;
            end
        end
    end
endmodule
