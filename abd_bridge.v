module adc_bridge #(
    parameter CLK_HZ     = 50_000_000,
    parameter BAUD       = 1_000_000,
    parameter TIMEOUT_MS = 5
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output reg  [11:0] code,
    output reg         valid,
    output reg         stale,
    output reg         framing_err
);
    localparam BIT_CLKS  = CLK_HZ / BAUD;
    localparam HALF_CLKS = BIT_CLKS / 2;

    // ---------------- 两级同步 ----------------
    reg [1:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 2'b11;
        else        rx_sync <= {rx_sync[0], rx};
    end
    wire rxs = rx_sync[1];

    // ---------------- UART 字节接收 ----------------
    localparam R_IDLE = 2'd0, R_START = 2'd1, R_DATA = 2'd2, R_STOP = 2'd3;
    reg [1:0]  r_state;
    reg [15:0] r_cnt;
    reg [2:0]  r_bit;
    reg [7:0]  r_shift;
    reg        byte_valid;
    reg [7:0]  byte_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state    <= R_IDLE;
            r_cnt      <= 16'd0;
            r_bit      <= 3'd0;
            r_shift    <= 8'd0;
            byte_valid <= 1'b0;
            byte_data  <= 8'd0;
            framing_err <= 1'b0;
        end else begin
            byte_valid  <= 1'b0;
            framing_err <= 1'b0;
            case (r_state)
                R_IDLE: if (!rxs) begin
                    r_state <= R_START;
                    r_cnt   <= 16'd0;
                end
                R_START: if (r_cnt == HALF_CLKS - 1) begin
                    r_state <= rxs ? R_IDLE : R_DATA;
                    r_cnt   <= 16'd0;
                    r_bit   <= 3'd0;
                end else r_cnt <= r_cnt + 1'b1;
                R_DATA: if (r_cnt == BIT_CLKS - 1) begin
                    r_cnt   <= 16'd0;
                    r_shift <= {rxs, r_shift[7:1]};         
                    if (r_bit == 3'd7) r_state <= R_STOP;
                    else               r_bit   <= r_bit + 1'b1;
                end else r_cnt <= r_cnt + 1'b1;
                R_STOP: if (r_cnt == BIT_CLKS - 1) begin
                    r_state <= R_IDLE;
                    if (rxs) begin                           
                        byte_valid <= 1'b1;
                        byte_data  <= r_shift;
                    end else begin
                        framing_err <= 1'b1;              
                    end
                end else r_cnt <= r_cnt + 1'b1;
            endcase
        end
    end

    // ---------------- 帧解析 ----------------
    localparam SYNC = 8'hA5;
    localparam F_SYNC = 2'd0, F_HI = 2'd1, F_LO = 2'd2, F_CHK = 2'd3;
    reg [1:0] f_state;
    reg [7:0] f_hi, f_lo;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_state <= F_SYNC;
            f_hi    <= 8'd0;
            f_lo    <= 8'd0;
            code    <= 12'd0;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;
            if (byte_valid) begin
                case (f_state)
                    F_SYNC: if (byte_data == SYNC) f_state <= F_HI;
                    F_HI:   if (byte_data[7:4] != 4'd0)
                                f_state <= (byte_data == SYNC) ? F_HI : F_SYNC;
                            else begin
                                f_hi    <= byte_data;
                                f_state <= F_LO;
                            end
                    F_LO:   begin f_lo <= byte_data; f_state <= F_CHK; end
                    F_CHK:  begin
                        f_state <= F_SYNC;
                        if (byte_data == (SYNC ^ f_hi ^ f_lo)) begin
                            code  <= {f_hi[3:0], f_lo};
                            valid <= 1'b1;
                        end
                    end
                endcase
            end
        end
    end

    // ---------------- 超时 ----------------
    localparam integer TIMEOUT_CLKS = (CLK_HZ / 1000) * TIMEOUT_MS;
    reg [31:0] to_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            to_cnt <= 32'd0;
            stale  <= 1'b1;
        end else if (valid) begin
            to_cnt <= 32'd0;
            stale  <= 1'b0;
        end else if (to_cnt >= TIMEOUT_CLKS - 1) begin
            stale  <= 1'b1;
        end else begin
            to_cnt <= to_cnt + 1'b1;
        end
    end
endmodule
