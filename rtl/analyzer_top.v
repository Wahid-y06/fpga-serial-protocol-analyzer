`timescale 1ns / 1ps
`default_nettype none

// Serial protocol analyzer: UART + SPI + I2C decoders feeding a timestamped,
// triggerable capture buffer that is read out over a UART link.
//
// All probe inputs are passive and asynchronous; each passes through a
// two-flop synchronizer. Each decoder emits single-cycle events; a small
// arbiter holds one pending event per decoder (stamped with the time it
// occurred) and writes one per clock into the capture buffer. `dropped` is a
// sticky flag set if a decoder produces a new event before its previous one
// was stored (cannot happen at supported line rates).
module analyzer_top #(
    parameter integer AW = 8                // capture depth = 2^AW events
) (
    input  wire          clk,
    input  wire          rst,
    // probes
    input  wire          uart_rx_pin,
    input  wire          spi_sclk_pin,
    input  wire          spi_mosi_pin,
    input  wire          spi_miso_pin,
    input  wire          spi_cs_n_pin,
    input  wire          i2c_scl_pin,
    input  wire          i2c_sda_pin,
    // configuration
    input  wire [15:0]   uart_cpb,          // clocks per bit of the probed UART
    input  wire          uart_parity_en,
    input  wire          uart_parity_odd,
    input  wire [1:0]    spi_mode,
    input  wire [5:0]    trig_mask,
    input  wire [7:0]    match_value,
    input  wire [15:0]   post_count,
    input  wire [15:0]   ts_prescale,       // timestamp tick = (ts_prescale+1) clocks
    input  wire [15:0]   host_cpb,          // clocks per bit of the host readout UART
    // control / status
    input  wire          arm,
    input  wire          dump,
    output wire          host_tx,
    output wire          armed,
    output wire          triggered,
    output wire          frozen,
    output wire          dumping,
    output wire [AW:0]   event_count,
    output reg           dropped
);
    // ------------------------------------------------------------ synchronizers
    wire uart_rx, sclk, mosi, miso, cs_n, scl, sda;
    sync2 #(.W(1), .INIT(1'b1)) u_s_rx   (.clk(clk), .d(uart_rx_pin),  .q(uart_rx));
    sync2 #(.W(1), .INIT(1'b0)) u_s_sclk (.clk(clk), .d(spi_sclk_pin), .q(sclk));
    sync2 #(.W(1), .INIT(1'b0)) u_s_mosi (.clk(clk), .d(spi_mosi_pin), .q(mosi));
    sync2 #(.W(1), .INIT(1'b0)) u_s_miso (.clk(clk), .d(spi_miso_pin), .q(miso));
    sync2 #(.W(1), .INIT(1'b1)) u_s_cs   (.clk(clk), .d(spi_cs_n_pin), .q(cs_n));
    sync2 #(.W(1), .INIT(1'b1)) u_s_scl  (.clk(clk), .d(i2c_scl_pin),  .q(scl));
    sync2 #(.W(1), .INIT(1'b1)) u_s_sda  (.clk(clk), .d(i2c_sda_pin),  .q(sda));

    // ---------------------------------------------------------------- decoders
    wire       u_valid, u_ferr, u_perr;
    wire [7:0] u_data;
    uart_rx_decoder u_uart (
        .clk(clk), .rst(rst), .rx(uart_rx), .clks_per_bit(uart_cpb),
        .parity_en(uart_parity_en), .parity_odd(uart_parity_odd),
        .valid(u_valid), .data(u_data), .frame_err(u_ferr), .parity_err(u_perr));

    wire       s_valid, s_incomplete;
    wire [2:0] s_bits;
    wire [7:0] s_mosi, s_miso;
    spi_decoder u_spi (
        .clk(clk), .rst(rst), .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n),
        .mode(spi_mode), .valid(s_valid), .incomplete(s_incomplete),
        .partial_bits(s_bits), .mosi_byte(s_mosi), .miso_byte(s_miso));

    wire       i_valid, i_nack;
    wire [2:0] i_kind;
    wire [7:0] i_data;
    i2c_decoder u_i2c (
        .clk(clk), .rst(rst), .scl(scl), .sda_in(sda),
        .valid(i_valid), .kind(i_kind), .data(i_data), .nack(i_nack));

    // --------------------------------------------------------------- timestamp
    reg [31:0] ts;
    reg [15:0] presc;
    always @(posedge clk) begin
        if (rst || arm) begin
            ts    <= 32'd0;
            presc <= 16'd0;
        end else if (presc == ts_prescale) begin
            presc <= 16'd0;
            ts    <= ts + 32'd1;
        end else begin
            presc <= presc + 16'd1;
        end
    end

    // ----------------------------------------------------------- event arbiter
    wire [23:0] u_ev = {2'd0, 3'd0, u_ferr, u_perr, 1'b0, u_data, 8'd0};
    wire [23:0] s_ev = s_incomplete ? {2'd1, s_bits, 1'b1, 1'b0, 1'b0, s_mosi, s_miso}
                                    : {2'd1, 3'd0,   1'b0, 1'b0, 1'b0, s_mosi, s_miso};
    wire [23:0] i_ev = {2'd2, i_kind, i_nack, 1'b0, 1'b0, i_data, 8'd0};

    reg [2:0]  pend;
    reg [55:0] hold0, hold1, hold2;
    reg        rec_valid;
    reg [55:0] rec;

    wire [2:0] new_ev = {i_valid, s_valid | s_incomplete, u_valid};
    // fixed priority: UART > SPI > I2C, one record per clock
    wire [2:0] grant  = {pend[2] & ~pend[1] & ~pend[0], pend[1] & ~pend[0], pend[0]};
    // a held event is lost if its source fires again before it is granted
    wire       overrun = |(new_ev & pend & ~grant);

    always @(posedge clk) begin
        rec_valid <= 1'b0;
        if (rst) begin
            pend    <= 3'b000;
            dropped <= 1'b0;
        end else begin
            if (arm)          dropped <= 1'b0;
            else if (overrun) dropped <= 1'b1;

            if (grant[0]) rec <= hold0;
            if (grant[1]) rec <= hold1;
            if (grant[2]) rec <= hold2;
            rec_valid <= |grant;

            // a new event re-sets its pending bit even when granted this cycle
            pend <= (pend & ~grant) | new_ev;
            if (new_ev[0]) hold0 <= {ts, u_ev};
            if (new_ev[1]) hold1 <= {ts, s_ev};
            if (new_ev[2]) hold2 <= {ts, i_ev};
        end
    end

    // ------------------------------------------------------ buffer and readout
    wire       tx_start, tx_ready;
    wire [7:0] tx_data;

    capture_buffer #(.AW(AW)) u_buf (
        .clk(clk), .rst(rst), .arm(arm), .dump(dump),
        .rec_valid(rec_valid), .rec(rec),
        .trig_mask(trig_mask), .match_value(match_value), .post_count(post_count),
        .armed(armed), .triggered(triggered), .frozen(frozen), .dumping(dumping),
        .count(event_count),
        .tx_start(tx_start), .tx_data(tx_data), .tx_ready(tx_ready));

    uart_tx u_tx (
        .clk(clk), .rst(rst), .clks_per_bit(host_cpb),
        .start(tx_start), .data(tx_data), .tx(host_tx), .ready(tx_ready));
endmodule

`default_nettype wire
