`timescale 1ns / 1ps
`default_nettype none

// Terasic DE10-Lite (MAX 10) wrapper for analyzer_top.
//
// Port names follow the DE10-Lite System Builder golden top; generate the
// pin assignments with Terasic's System Builder. Not yet tested on hardware.
//
//   GPIO[0] UART probe     GPIO[1] SPI SCLK    GPIO[2] SPI MOSI   GPIO[3] SPI MISO
//   GPIO[4] SPI CS_n       GPIO[5] I2C SCL     GPIO[6] I2C SDA
//   GPIO[7] host TX -> USB-UART adapter RX (115200 8N1)
//
//   KEY[0] arm     KEY[1] dump capture to PC
//   SW[1:0] SPI mode   SW[2] UART parity enable   SW[3] odd parity
//   SW[9:4] trigger mask (bit 4 framing, 5 parity, 6 I2C NACK,
//           7 SPI incomplete, 8 data == 0x00, 9 any event)
//   LEDR[0] armed  [1] triggered  [2] frozen  [3] dumping  [4] event dropped
//   LEDR[9:5] capture fill level (top bits of the event count)
//
// All probes are 3.3 V LVTTL inputs - do not connect 5 V signals directly.
module de10lite_top (
    input  wire        MAX10_CLK1_50,
    input  wire [1:0]  KEY,
    input  wire [9:0]  SW,
    output wire [9:0]  LEDR,
    inout  wire [35:0] GPIO
);
    localparam integer CPB_115200 = 434;           // 50 MHz / 115200
    localparam integer AW = 8;

    wire clk = MAX10_CLK1_50;

    // power-on reset: hold for 16 clocks after configuration
    reg [4:0] por = 5'd0;
    wire rst = !por[4];
    always @(posedge clk) if (!por[4]) por <= por + 5'd1;

    // debounced, active-low push buttons -> one-clock pulses on press
    wire arm_pulse, dump_pulse;
    button_pulse u_key0 (.clk(clk), .btn_n(KEY[0]), .pulse(arm_pulse));
    button_pulse u_key1 (.clk(clk), .btn_n(KEY[1]), .pulse(dump_pulse));

    wire [AW:0] count;
    wire armed, triggered, frozen, dumping, dropped, host_tx;

    analyzer_top #(.AW(AW)) u_la (
        .clk(clk), .rst(rst),
        .uart_rx_pin(GPIO[0]), .spi_sclk_pin(GPIO[1]), .spi_mosi_pin(GPIO[2]),
        .spi_miso_pin(GPIO[3]), .spi_cs_n_pin(GPIO[4]),
        .i2c_scl_pin(GPIO[5]), .i2c_sda_pin(GPIO[6]),
        .uart_cpb(CPB_115200[15:0]), .uart_parity_en(SW[2]), .uart_parity_odd(SW[3]),
        .spi_mode(SW[1:0]), .trig_mask(SW[9:4]), .match_value(8'h00),
        .post_count(16'd64), .ts_prescale(16'd49),             // 1 us timestamps
        .host_cpb(CPB_115200[15:0]),
        .arm(arm_pulse), .dump(dump_pulse), .host_tx(host_tx),
        .armed(armed), .triggered(triggered), .frozen(frozen), .dumping(dumping),
        .event_count(count), .dropped(dropped));

    assign GPIO[7]     = host_tx;
    assign GPIO[6:0]   = 7'bz;
    assign GPIO[35:8]  = 28'bz;
    assign LEDR[4:0]   = {dropped, dumping, frozen, triggered, armed};
    assign LEDR[9:5]   = count[AW:AW-4];
endmodule


// Synchronize an active-low button, require it to be stable for ~10 ms,
// and emit a single-clock pulse when it becomes pressed.
module button_pulse #(
    parameter integer STABLE_CLKS = 500_000         // 10 ms at 50 MHz
) (
    input  wire clk,
    input  wire btn_n,
    output reg  pulse = 1'b0
);
    reg [1:0]  sync = 2'b11;
    reg        state = 1'b1;
    reg [19:0] cnt = 20'd0;

    always @(posedge clk) begin
        sync  <= {sync[0], btn_n};
        pulse <= 1'b0;
        if (sync[1] == state) begin
            cnt <= 20'd0;
        end else if (cnt == STABLE_CLKS - 1) begin
            cnt   <= 20'd0;
            state <= sync[1];
            pulse <= !sync[1];                      // pressed (went low)
        end else begin
            cnt <= cnt + 20'd1;
        end
    end
endmodule

`default_nettype wire
