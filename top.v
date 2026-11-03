module top (
    input wire clk_50m,
    input wire key_forward_n,
    input wire key_reverse_n,
    input wire reset_n,
    output wire [3:0] led,
    output wire uart_tx,
    input wire uart_rx,
    output wire motor_pwm,
    output wire motor_in1,
    output wire motor_in2,
    input wire enc_a,
    input wire enc_b,
    output wire adc_clk,
    output wire adc_oe,
    input wire [9:0] adc_d
);

    wire sys_clk = clk_50m;

    // ================= ADC 驱动 =================
    assign adc_oe = 1'b0; 
    reg [4:0] adc_cnt;
    always @(posedge sys_clk) adc_cnt <= adc_cnt + 1'b1;
    assign adc_clk = adc_cnt[4]; 

    reg [9:0] adc_d_r1, adc_d_r2;
    always @(posedge sys_clk) begin
        adc_d_r1 <= adc_d;
        adc_d_r2 <= adc_d_r1;
    end
    wire [11:0] adc_code = {adc_d_r2, 2'b00};
    wire adc_stale = 1'b0;

    // 动态中心点寄存器
    reg signed [31:0] center_x10 = 32'sd20100;
    always @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            center_x10 <= 32'sd20100; 
        end else if (~key_reverse_n) begin
            center_x10 <= $signed({20'd0, adc_code}) * 32'sd10;
        end
    end

    // ================= 编码器 =================
    wire signed [31:0] enc_raw, enc;
    quad_decoder #(.FILTER(4)) u_qd (
        .clk(sys_clk), .rst_n(reset_n), .enc_a(enc_a), .enc_b(enc_b), .count(enc_raw)
    );

    // ================= 核心控制 =================
    wire signed [7:0] u_ctrl_wire, u_final; 
    wire u_mag, u_en;
    wire is_balance_mode; 

    balance_ctrl #(
        .CLK_HZ(50_000_000), .ANG_DIV(5), .LOC_DIV(50)
    ) u_balance ( 
        .clk(sys_clk), .rst_n(reset_n),
        .stale(adc_stale),
        .adc_code(adc_code), .loc(enc),
        .center_x10_in(center_x10), 
        .key_forward_n(key_forward_n), 
        .ang_tick(), .loc_tick(), .tgt_x10(),
        .u_dir(u_ctrl_wire), .u_mag(u_mag), .en(u_en), .is_balance_mode(is_balance_mode) 
    );

    // ================= 方向映射 =================
    sign_map #(
        .CENTER_CODE(2010), 
        .ANGLE_SIGN(1),     
        .MOTOR_SIGN(-1),    // 注意保持你的极性 -1
        .ENC_SIGN(1)
    ) u_map (
        .adc_code_raw(adc_code), .enc_raw(enc_raw), .u_ctrl(u_ctrl_wire),
        .adc_code(), .enc(enc), .u(u_final)
    );

    // ================= 电机 =================
    pwm_gen #(.CLK_HZ(50_000_000), .PWM_HZ(20_000)) u_pwm (
        .clk(sys_clk), .rst_n(reset_n), .en(u_en), .u(u_final),    
        .pwma(motor_pwm), .ain1(motor_in1), .ain2(motor_in2)
    );

    // ================= LED =================
    reg [3:0] led_r;
    assign led = led_r;

    reg [24:0] cnt;
    reg hb_d;
    always @(posedge sys_clk) begin
        cnt <= cnt + 1'b1;
        hb_d <= cnt[24];
        
        led_r[0] <= cnt[24];              // 闪烁：心跳
        led_r[1] <= is_balance_mode;      // 亮起：PID 工作中
        led_r[2] <= (~key_forward_n);     // 亮起：按下 SW1
        led_r[3] <= (~key_reverse_n);     // 亮起：按下 SW2 校准
    end

    // 串口保留
    wire tx_ready; reg [7:0] tx_byte = 8'd0; reg [1:0] tx_idx = 2'd0; reg tx_valid = 1'b0; reg tx_send = 1'b0;
    uart_tx #(.CLK_HZ(50_000_000), .BAUD(115200)) u_tx (
        .clk(sys_clk), .rst_n(reset_n), .data(tx_byte), .valid(tx_valid), .ready(tx_ready), .tx(uart_tx)
    );
    always @(posedge sys_clk) begin
        tx_valid <= 1'b0;
        if (cnt[24] & ~hb_d & tx_ready & ~tx_send) begin tx_send <= 1'b1; tx_idx <= 2'd0; end 
        else if (tx_send & ~tx_valid & tx_ready) begin
            case (tx_idx)
                2'd0: tx_byte <= 8'hA5; 2'd1: tx_byte <= adc_code[7:0]; 2'd2: tx_byte <= enc_raw[7:0]; default: tx_byte <= 8'd0;
            endcase
            tx_valid <= 1'b1; if (tx_idx == 2'd3) tx_send <= 1'b0; else tx_idx <= tx_idx + 1'b1;
        end
    end
endmodule
