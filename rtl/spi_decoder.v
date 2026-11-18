`timescale 1ns / 1ps
`default_nettype none

// Passive SPI decoder (all four modes), MSB first, 8-bit words.
//
// SCLK is oversampled by the system clock, so f_sclk must be below f_clk / 4.
// Data is captured on the sampling edge for the selected mode:
//   mode 0 (CPOL=0, CPHA=0) and mode 3 (1,1): rising edge
//   mode 1 (CPOL=0, CPHA=1) and mode 2 (1,0): falling edge
// If chip select is released part-way through a word, an `incomplete`
// event reports the bits received so far and how many there were.
module spi_decoder (
    input  wire       clk,
    input  wire       rst,
    input  wire       sclk,            // synchronized lines
    input  wire       mosi,
    input  wire       miso,
    input  wire       cs_n,
    input  wire [1:0] mode,            // {CPOL, CPHA}
    output reg        valid,           // full byte received
    output reg        incomplete,      // CS released mid-byte
    output reg  [2:0] partial_bits,    // bits received for an incomplete word
    output reg  [7:0] mosi_byte,
    output reg  [7:0] miso_byte
);
    reg       sclk_d;
    reg [2:0] bit_cnt;
    reg [7:0] mo_sh, mi_sh;

    wire rise   = sclk & ~sclk_d;
    wire fall   = ~sclk & sclk_d;
    wire sample = (mode[1] ^ mode[0]) ? fall : rise;

    always @(posedge clk) begin
        valid      <= 1'b0;
        incomplete <= 1'b0;
        sclk_d     <= sclk;
        if (rst) begin
            bit_cnt      <= 3'd0;
            mo_sh        <= 8'd0;
            mi_sh        <= 8'd0;
            mosi_byte    <= 8'd0;
            miso_byte    <= 8'd0;
            partial_bits <= 3'd0;
            sclk_d       <= mode[1];
        end else if (cs_n) begin
            if (bit_cnt != 0) begin
                incomplete   <= 1'b1;
                partial_bits <= bit_cnt;
                mosi_byte    <= mo_sh;
                miso_byte    <= mi_sh;
            end
            bit_cnt <= 3'd0;
            mo_sh   <= 8'd0;
            mi_sh   <= 8'd0;
        end else if (sample) begin
            bit_cnt <= bit_cnt + 3'd1;          // wraps to 0 after 8 bits
            if (bit_cnt == 3'd7) begin
                valid     <= 1'b1;
                mosi_byte <= {mo_sh[6:0], mosi};
                miso_byte <= {mi_sh[6:0], miso};
                // start each word from zero so a partial word is right-aligned
                // and never contains bits left over from the previous word
                mo_sh     <= 8'd0;
                mi_sh     <= 8'd0;
            end else begin
                mo_sh <= {mo_sh[6:0], mosi};
                mi_sh <= {mi_sh[6:0], miso};
            end
        end
    end
endmodule

`default_nettype wire
