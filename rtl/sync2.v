`timescale 1ns / 1ps
`default_nettype none

// Two-flop synchronizer for asynchronous inputs.
// INIT sets the power-up value of each bit (idle level of the line), so no
// spurious edges are seen right after configuration.
module sync2 #(
    parameter integer W    = 1,
    parameter [W-1:0] INIT = {W{1'b1}}
) (
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    reg [W-1:0] s1 = INIT;
    reg [W-1:0] s2 = INIT;

    always @(posedge clk) begin
        s1 <= d;
        s2 <= s1;
    end

    assign q = s2;
endmodule

`default_nettype wire
