module balance_ctrl #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer ANG_DIV = 5,
    parameter integer LOC_DIV = 50 
)(
    input wire clk,
    input wire rst_n,
    input wire stale, 
    input wire [11:0] adc_code,
    input wire signed [31:0] loc,
    input wire signed [31:0] center_x10_in, 
    input wire key_forward_n, 
    output wire ang_tick,
    output wire loc_tick,
    output wire signed [31:0] tgt_x10,
    output wire signed [7:0] u_dir,
    output wire u_mag,
    output wire en,
    output wire is_balance_mode 
);

    // 1kHz 分频
    reg [15:0] tick_cnt = 16'd0;
    reg [15:0] ang_cnt = 16'd0;
    reg [15:0] loc_cnt = 16'd0;
    
    wire tick_1k = (tick_cnt == (CLK_HZ/1000) - 1);
    wire ang_t = tick_1k && (ang_cnt == ANG_DIV - 1);
    wire loc_t = tick_1k && (loc_cnt == LOC_DIV - 1); 
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 16'd0;
            ang_cnt <= 16'd0;
            loc_cnt <= 16'd0;
        end else if (tick_1k) begin
            tick_cnt <= 16'd0;
            ang_cnt <= ang_t ? 16'd0 : ang_cnt + 1'b1;
            loc_cnt <= loc_t ? 16'd0 : loc_cnt + 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
        end
    end

    // ================= ADC 滤波 =================
    reg [15:0] adc_sum;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) adc_sum <= 16'd0;
        else adc_sum <= adc_sum - adc_sum[15:4] + adc_code;
    end
    wire [11:0] adc_filt = adc_sum[15:4];
    wire signed [31:0] adc_x10 = $signed({20'd0, adc_filt}) * 32'sd10;
    
    // ================= 外环：极弱速度+位置反馈（防跑飞） =================
    reg signed [31:0] loc_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) loc_prev <= 32'sd0;
        else if (loc_t) loc_prev <= loc;
    end
    wire signed [31:0] loc_spd = loc - loc_prev; 

    // 限制位置和速度的输入范围，防止极端情况下数值溢出
    wire signed [31:0] spd_clamped = (loc_spd > 20) ? 32'sd20 : ((loc_spd < -20) ? -32'sd20 : loc_spd);
    wire signed [31:0] loc_clamped = (loc > 200) ? 32'sd200 : ((loc < -200) ? -32'sd200 : loc);

    // 外环极弱：位置项 /10，速度项 *2
    wire signed [31:0] loc_out = (loc_clamped / 10) + (spd_clamped * 2);

    wire signed [31:0] loc_out_lim = (loc_out > 30) ? 32'sd30 : 
                                     ((loc_out < -30) ? -32'sd30 : loc_out);

    // 动态目标角度
    reg signed [31:0] target_angle_eff;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) target_angle_eff <= center_x10_in;
        else if (state == S_SWING) target_angle_eff <= center_x10_in;
        else target_angle_eff <= center_x10_in - loc_out_lim; 
    end

    // ================= 状态机 =================
    localparam S_BALANCE = 1'b0;
    localparam S_SWING   = 1'b1;
    reg state;
    
    wire signed [31:0] err;
    wire signed [31:0] d_err;
    reg signed [31:0] err_prev;

    assign err = target_angle_eff - adc_x10;
    assign d_err = err - err_prev; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_SWING; 
            err_prev <= 32'sd0;
        end else if (ang_t) begin
            err_prev <= err;
            if (state == S_SWING) begin
                if (err < 2000 && err > -2000 && d_err < 1500 && d_err > -1500) begin
                    state <= S_BALANCE;
                end
            end else begin
                if (err > 3000 || err < -3000) begin
                    state <= S_SWING;
                end
            end
        end
    end
    assign is_balance_mode = (state == S_BALANCE);

    // ================= 内环：PD 计算 =================
    wire signed [63:0] p_term = err * 32'sd45;
    wire signed [63:0] d_term = d_err * 32'sd70;
    wire signed [63:0] pd_out_64 = p_term + d_term;
    wire signed [31:0] pd_out = pd_out_64 / 1000;

    // ================= 最终输出 =================
    reg signed [7:0] final_u;
    always @(*) begin
        if (state == S_BALANCE) begin
            if (pd_out > 85) final_u = 8'sd85;
            else if (pd_out < -85) final_u = -8'sd85;
            else final_u = pd_out[7:0]; 
            
            // 死区补偿：克服齿轮静摩擦
            if (final_u > 0 && final_u < 12) final_u = 8'sd12;
            else if (final_u < 0 && final_u > -12) final_u = -8'sd12;
        end else begin
            if (err > 0) final_u = 8'sd70;
            else final_u = -8'sd70;
        end
    end

    wire is_holding_key = (~key_forward_n); 
    assign en = is_holding_key;              
    
    assign u_dir = final_u;  
    assign u_mag = (final_u != 0);
    
    assign ang_tick = ang_t;
    assign loc_tick = loc_t;
    assign tgt_x10 = target_angle_eff;

endmodule
