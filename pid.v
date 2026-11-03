module pid #(
    parameter integer KP =2.5,
    parameter integer KI = 0,     
    parameter integer KD = 0.008,
    parameter integer DIV = 1000,
    parameter integer OUT_MAX = 100,
    parameter integer INT_LIM = 100000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick,
    input  wire clr_int,
    input  wire signed [31:0] target,
    input  wire signed [31:0] actual,
    output reg  signed [31:0] out,
    output reg  done
);

    localparam integer QB = $clog2(OUT_MAX + 1);
    localparam [63:0] SAT_ABS = OUT_MAX * DIV;

    localparam S_IDLE  = 3'd0;
    localparam S_SUM   = 3'd1;
    localparam S_ABS   = 3'd2;
    localparam S_DIV   = 3'd3;
    localparam S_OUT   = 3'd4;

    reg signed [31:0] e0, e1, ei;
    reg signed [31:0] de;
    reg signed [63:0] sum_r;
    reg [2:0] st;
    reg neg;
    reg [63:0] rem;
    reg [31:0] quo;
    reg [5:0]  bitn;

    wire signed [31:0] e_new = target - actual;

    wire signed [31:0] ei_base = clr_int ? 32'sd0 : ei;
    wire signed [32:0] ei_raw = $signed({ei_base[31], ei_base}) + $signed({e_new[31], e_new});
    
    wire signed [31:0] ei_new = (KI == 0) ? 32'sd0 :
                               (ei_raw > INT_LIM) ? INT_LIM :
                               (ei_raw < -INT_LIM) ? -INT_LIM :
                               ei_raw[31:0];

    wire signed [63:0] sum = $signed(KP) * e0 + $signed(KI) * ei + $signed(KD) * de;
    
    wire [63:0] sum_abs = sum_r[63] ? -sum_r : sum_r;
    wire [63:0] dsh = (64'd0 + DIV) << bitn;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e0 <= 0; e1 <= 0; ei <= 0; de <= 0; sum_r <= 0;
            st <= S_IDLE; neg <= 1'b0;
            rem <= 0; quo <= 0; bitn <= 0;
            out <= 32'sd0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)
                S_IDLE: if (tick) begin
                    e1 <= e0;
                    e0 <= e_new;
                    de <= e_new - e0;
                    ei <= ei_new;
                    st <= S_SUM;
                end
                S_SUM: begin
                    sum_r <= sum;
                    st <= S_ABS;
                end
                S_ABS: begin
                    neg <= sum_r[63];
                    rem <= sum_abs; 
                    bitn <= QB - 1;
                    if (sum_abs >= SAT_ABS) begin
                        quo <= OUT_MAX;
                        st <= S_OUT;
                    end else begin
                        quo <= 0;
                        st <= S_DIV;
                    end
                end
                S_DIV: begin
                    if (rem >= dsh) begin
                        rem <= rem - dsh;
                        quo <= quo | (32'd1 << bitn);
                    end
                    if (bitn == 0) st <= S_OUT;
                    else bitn <= bitn - 1'b1;
                end
                S_OUT: begin
                    out <= neg ? -$signed(quo) : $signed(quo);
                    done <= 1'b1;
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase

            if (clr_int && !(st == S_IDLE && tick)) ei <= 32'sd0;
        end
    end

endmodule
