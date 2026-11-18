`timescale 1ns / 1ps
`default_nettype none

// Passive I2C bus decoder.
//
//   START   : SDA falls while SCL is high
//   STOP    : SDA rises while SCL is high
//   bits    : SDA sampled on each SCL rising edge, MSB first, 8 data + 1 ACK
//
// The first byte after a (repeated) START is reported as an ADDRESS event,
// later bytes as DATA. `nack` is set when the receiver left SDA high in the
// ACK slot - the "missing acknowledgment" condition used for triggering.
//
// I2C allows 0 ns SDA hold time after SCL falls. SCL and SDA pass through
// separate synchronizers and have different rise/fall times, so a data
// change right at the SCL falling edge can be seen a clock *before* SCL goes
// low - which looks exactly like a START or STOP. Delaying SDA by SDA_DELAY
// (>= 2) clocks relative to SCL makes such changes appear after SCL is
// already low, while staying far below the minimum data setup time
// (50 ns in Fast-mode Plus = 2.5 clocks at 50 MHz; 100 ns in Fast-mode).
// The sda_skew_does_not_fake_start_stop test fails without this delay.
module i2c_decoder #(
    parameter integer SDA_DELAY = 2
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       scl,             // synchronized lines
    input  wire       sda_in,
    output reg        valid,
    output reg  [2:0] kind,
    output reg  [7:0] data,
    output reg        nack
);
    localparam [2:0] K_START = 3'd0, K_RESTART = 3'd1, K_STOP = 3'd2,
                     K_ADDR  = 3'd3, K_DATA    = 3'd4;

    reg [SDA_DELAY-1:0] sda_pipe = {SDA_DELAY{1'b1}};
    wire sda = sda_pipe[SDA_DELAY-1];

    reg       scl_d, sda_d;
    reg       in_frame;
    reg       first_byte;
    reg [3:0] bit_cnt;
    reg [7:0] shreg;

    wire start_det = scl & scl_d & sda_d & ~sda;
    wire stop_det  = scl & scl_d & ~sda_d & sda;
    wire scl_rise  = scl & ~scl_d;

    always @(posedge clk) begin
        sda_pipe <= {sda_pipe[SDA_DELAY-2:0], sda_in};
    end

    always @(posedge clk) begin
        valid <= 1'b0;
        scl_d <= scl;
        sda_d <= sda;
        if (rst) begin
            in_frame   <= 1'b0;
            first_byte <= 1'b0;
            bit_cnt    <= 4'd0;
            shreg      <= 8'd0;
            kind       <= K_START;
            data       <= 8'd0;
            nack       <= 1'b0;
            scl_d      <= 1'b1;
            sda_d      <= 1'b1;
        end else if (start_det) begin
            valid      <= 1'b1;
            kind       <= in_frame ? K_RESTART : K_START;
            data       <= 8'd0;
            nack       <= 1'b0;
            in_frame   <= 1'b1;
            first_byte <= 1'b1;
            bit_cnt    <= 4'd0;
        end else if (stop_det) begin
            valid    <= 1'b1;
            kind     <= K_STOP;
            data     <= 8'd0;
            nack     <= 1'b0;
            in_frame <= 1'b0;
        end else if (in_frame && scl_rise) begin
            if (bit_cnt != 4'd8) begin
                shreg   <= {shreg[6:0], sda};
                bit_cnt <= bit_cnt + 4'd1;
            end else begin
                valid      <= 1'b1;
                kind       <= first_byte ? K_ADDR : K_DATA;
                data       <= shreg;
                nack       <= sda;
                first_byte <= 1'b0;
                bit_cnt    <= 4'd0;
            end
        end
    end
endmodule

`default_nettype wire
