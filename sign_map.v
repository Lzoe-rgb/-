module sign_map #(
    parameter integer CENTER_CODE = 2010,   
    parameter integer ANGLE_SIGN  = 1,
    parameter integer MOTOR_SIGN  = -1,
    parameter integer ENC_SIGN    = 1
) (
    input  wire [11:0]        adc_code_raw,
    input  wire signed [31:0] enc_raw,
    input  wire signed [7:0]  u_ctrl,
    output wire [11:0]        adc_code,
    output wire signed [31:0] enc,
    output wire signed [7:0]  u
);
    localparam integer TWO_CENTER = CENTER_CODE * 2;
    wire signed [13:0] adc_mirror = TWO_CENTER[13:0] - $signed({2'b0, adc_code_raw});
    wire        [11:0] adc_clamped = (adc_mirror < 0)     ? 12'd0 :
                                     (adc_mirror > 4095)  ? 12'd4095 :
                                                            adc_mirror[11:0];
    assign adc_code = (ANGLE_SIGN > 0) ? adc_code_raw : adc_clamped;

    assign enc = (ENC_SIGN > 0) ? enc_raw : -enc_raw;

    assign u = (MOTOR_SIGN > 0) ? u_ctrl : -u_ctrl;
endmodule
