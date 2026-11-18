`timescale 1ns / 1ps
`default_nettype none

// 8N1 UART transmitter used to stream captures to the host PC.
// Pulse `start` while `ready` is high to send `data`.
module uart_tx (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] clks_per_bit,
    input  wire        start,
    input  wire [7:0]  data,
    output reg         tx,
    output wire        ready
);
    reg        busy;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;            // 0 = start bit, 1..8 = data, 9 = stop bit
    reg [9:0]  frame;

    assign ready = !busy;

    always @(posedge clk) begin
        if (rst) begin
            busy    <= 1'b0;
            tx      <= 1'b1;
            cnt     <= 16'd0;
            bit_idx <= 4'd0;
            frame   <= 10'h3FF;
        end else if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                frame   <= {1'b1, data, 1'b0};
                busy    <= 1'b1;
                tx      <= 1'b0;
                cnt     <= clks_per_bit - 16'd1;
                bit_idx <= 4'd0;
            end
        end else if (cnt != 0) begin
            cnt <= cnt - 16'd1;
        end else if (bit_idx == 4'd9) begin
            busy <= 1'b0;
            tx   <= 1'b1;
        end else begin
            bit_idx <= bit_idx + 4'd1;
            tx      <= frame[bit_idx + 4'd1];
            cnt     <= clks_per_bit - 16'd1;
        end
    end
endmodule

`default_nettype wire
