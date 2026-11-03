module key_ctrl (
    input  wire clk, rst_n, key_fwd_n, key_rev_n,
    output reg  en_out,
    output reg  dir_flip
);
    reg [19:0] cnt_fwd, cnt_rev;
    reg fwd_stable, rev_stable;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cnt_fwd <= 0; cnt_rev <= 0; fwd_stable <= 1; rev_stable <= 1;
        end else begin
            if (key_fwd_n != fwd_stable) begin
                if (cnt_fwd == 20'd1_000_000) begin fwd_stable <= key_fwd_n; cnt_fwd <= 0; end
                else cnt_fwd <= cnt_fwd + 1;
            end else cnt_fwd <= 0;

            if (key_rev_n != rev_stable) begin
                if (cnt_rev == 20'd1_000_000) begin rev_stable <= key_rev_n; cnt_rev <= 0; end
                else cnt_rev <= cnt_rev + 1;
            end else cnt_rev <= 0;
        end
    end

    reg fwd_d1, rev_d1;
    always @(posedge clk) begin
        fwd_d1 <= fwd_stable; rev_d1 <= rev_stable;
    end
    wire fwd_fall = (fwd_d1 && !fwd_stable);
    wire rev_fall = (rev_d1 && !rev_stable);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            en_out <= 1'b0; 
            dir_flip <= 1'b0;
        end else begin
            if (fwd_fall) en_out <= ~en_out;
            if (rev_fall) dir_flip <= ~dir_flip;
        end
    end
endmodule
