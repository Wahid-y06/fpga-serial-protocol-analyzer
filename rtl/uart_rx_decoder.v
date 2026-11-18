`timescale 1ns / 1ps
`default_nettype none

// Passive UART decoder: 8 data bits, LSB first, optional parity, 1 stop bit.
//
// The start-bit edge starts a counter that samples every bit at its centre
// (half a bit after the edge, then every clks_per_bit). A start bit that is
// no longer low at its centre is treated as a glitch. A low stop bit flags a
// framing error; the decoder then waits for the line to return high so a
// break condition is not decoded as a stream of 0x00 bytes.
//
// clks_per_bit = f_clk / baud, must be >= 4.
module uart_rx_decoder (
    input  wire        clk,
    input  wire        rst,
    input  wire        rx,              // synchronized line
    input  wire [15:0] clks_per_bit,
    input  wire        parity_en,
    input  wire        parity_odd,
    output reg         valid,
    output reg  [7:0]  data,
    output reg         frame_err,
    output reg         parity_err
);
    localparam [2:0] S_IDLE   = 3'd0,
                     S_START  = 3'd1,
                     S_DATA   = 3'd2,
                     S_PARITY = 3'd3,
                     S_STOP   = 3'd4,
                     S_BREAK  = 3'd5;

    reg [2:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;
    reg        par_bit;

    wire [15:0] full_bit = clks_per_bit - 16'd1;
    wire [15:0] half_bit = (clks_per_bit >> 1) - 16'd1;

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            state      <= S_IDLE;
            cnt        <= 16'd0;
            bit_idx    <= 3'd0;
            shreg      <= 8'd0;
            par_bit    <= 1'b0;
            data       <= 8'd0;
            frame_err  <= 1'b0;
            parity_err <= 1'b0;
        end else begin
            case (state)
                S_IDLE: if (!rx) begin
                    state <= S_START;
                    cnt   <= half_bit;
                end

                S_START: if (cnt != 0) begin
                    cnt <= cnt - 16'd1;
                end else if (!rx) begin
                    state   <= S_DATA;
                    cnt     <= full_bit;
                    bit_idx <= 3'd0;
                end else begin
                    state <= S_IDLE;        // glitch, not a real start bit
                end

                S_DATA: if (cnt != 0) begin
                    cnt <= cnt - 16'd1;
                end else begin
                    shreg   <= {rx, shreg[7:1]};
                    cnt     <= full_bit;
                    bit_idx <= bit_idx + 3'd1;
                    if (bit_idx == 3'd7)
                        state <= parity_en ? S_PARITY : S_STOP;
                end

                S_PARITY: if (cnt != 0) begin
                    cnt <= cnt - 16'd1;
                end else begin
                    par_bit <= rx;
                    cnt     <= full_bit;
                    state   <= S_STOP;
                end

                S_STOP: if (cnt != 0) begin
                    cnt <= cnt - 16'd1;
                end else begin
                    valid      <= 1'b1;
                    data       <= shreg;
                    frame_err  <= !rx;
                    // even parity: data + parity bit has an even number of 1s
                    parity_err <= parity_en && ((^shreg ^ par_bit) != parity_odd);
                    state      <= rx ? S_IDLE : S_BREAK;
                end

                S_BREAK: if (rx) state <= S_IDLE;

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
