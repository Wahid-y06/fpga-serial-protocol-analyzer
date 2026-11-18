`timescale 1ns / 1ps
`default_nettype none

// Circular capture buffer with trigger, post-trigger count and UART readout.
//
// Record (56 bits): {timestamp[31:0], tag[7:0], d0[7:0], d1[7:0]}
//   tag = {proto[1:0], kind[2:0], flag_a, flag_b, 1'b0}
//   proto 0 = UART : d0 = byte, flag_a = framing error, flag_b = parity error
//   proto 1 = SPI  : d0 = MOSI, d1 = MISO, flag_a = incomplete word,
//                    kind = number of bits in an incomplete word
//   proto 2 = I2C  : kind 0 START, 1 RESTART, 2 STOP, 3 ADDRESS, 4 DATA;
//                    d0 = byte, flag_a = NACK
//
// Trigger mask bits:
//   0 UART framing error   1 UART parity error   2 I2C NACK
//   3 SPI incomplete word  4 data byte == match_value   5 any event
//
// IDLE --arm--> ARMED --trigger--> POST --post_count events--> FROZEN
// While ARMED the buffer keeps the most recent 2^AW events (pre-trigger
// history). After the trigger, post_count more events are stored, then the
// buffer freezes. A `dump` pulse in FROZEN streams it out over the UART:
//
//   header : 'L' 'A' count[15:8] count[7:0] trig_idx[15:8] trig_idx[7:0]
//   record : 0xA5 ts[31:24] ts[23:16] ts[15:8] ts[7:0] tag d0 d1
//
// Records are sent oldest first; trig_idx is the trigger record's position.
module capture_buffer #(
    parameter integer AW = 8
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          arm,
    input  wire          dump,
    input  wire          rec_valid,
    input  wire [55:0]   rec,
    input  wire [5:0]    trig_mask,
    input  wire [7:0]    match_value,
    input  wire [15:0]   post_count,
    output wire          armed,
    output wire          triggered,
    output wire          frozen,
    output wire          dumping,
    output reg  [AW:0]   count,
    output reg           tx_start,
    output reg  [7:0]    tx_data,
    input  wire          tx_ready
);
    localparam integer DEPTH = 1 << AW;
    localparam [2:0] S_IDLE = 3'd0, S_ARMED = 3'd1, S_POST = 3'd2,
                     S_FROZEN = 3'd3, S_DUMP = 3'd4;

    reg [2:0]    state;
    reg [AW-1:0] wr_ptr, trig_ptr, rd_ptr;
    reg [15:0]   post_left;
    reg [55:0]   mem [0:DEPTH-1];
    reg [55:0]   rd_data;

    // dump sequencing
    reg          in_hdr;
    reg [2:0]    hdr_idx;
    reg [2:0]    byte_idx;
    reg [AW:0]   rec_idx;

    assign armed     = (state == S_ARMED) || (state == S_POST);
    assign triggered = (state == S_POST) || (state == S_FROZEN) || (state == S_DUMP);
    assign frozen    = (state == S_FROZEN) || (state == S_DUMP);
    assign dumping   = (state == S_DUMP);

    // ---------------------------------------------------------- trigger match
    wire [1:0] proto  = rec[23:22];
    wire [2:0] kind   = rec[21:19];
    wire       flag_a = rec[18];
    wire       flag_b = rec[17];
    wire [7:0] d0     = rec[15:8];

    wire i2c_byte = (proto == 2'd2) && (kind == 3'd3 || kind == 3'd4);
    wire has_data = (proto == 2'd0) || (proto == 2'd1 && !flag_a) || i2c_byte;
    wire [5:0] hits = {
        1'b1,                              // 5 any event
        has_data && (d0 == match_value),   // 4 data match
        (proto == 2'd1) && flag_a,         // 3 SPI incomplete
        i2c_byte && flag_a,                // 2 I2C NACK
        (proto == 2'd0) && flag_b,         // 1 UART parity error
        (proto == 2'd0) && flag_a          // 0 UART framing error
    };
    wire hit = |(hits & trig_mask);

    wire [15:0] post_max = DEPTH - 1;      // never overwrite the trigger record
    wire [15:0] post_eff = (post_count > post_max) ? post_max : post_count;

    // ------------------------------------------------------------- memory
    wire capturing = (state == S_ARMED) || (state == S_POST);
    wire we = capturing && rec_valid && !arm;

    always @(posedge clk) begin
        if (we) mem[wr_ptr] <= rec;
        rd_data <= mem[rd_ptr];
    end

    wire [AW-1:0] oldest   = (count == DEPTH) ? wr_ptr : {AW{1'b0}};
    wire [AW-1:0] trig_rel = trig_ptr - oldest;

    // header fields zero-extended to 16 bits (AW <= 14)
    wire [15:0] count16 = {{(15 - AW){1'b0}}, count};
    wire [15:0] trig16  = {{(16 - AW){1'b0}}, trig_rel};

    // ------------------------------------------------------ dump byte select
    reg [7:0] cur_byte;
    always @* begin
        if (in_hdr) begin
            case (hdr_idx)
                3'd0: cur_byte = 8'h4C;                          // 'L'
                3'd1: cur_byte = 8'h41;                          // 'A'
                3'd2: cur_byte = count16[15:8];
                3'd3: cur_byte = count16[7:0];
                3'd4: cur_byte = trig16[15:8];
                default: cur_byte = trig16[7:0];
            endcase
        end else begin
            case (byte_idx)
                3'd0: cur_byte = 8'hA5;
                3'd1: cur_byte = rd_data[55:48];
                3'd2: cur_byte = rd_data[47:40];
                3'd3: cur_byte = rd_data[39:32];
                3'd4: cur_byte = rd_data[31:24];
                3'd5: cur_byte = rd_data[23:16];
                3'd6: cur_byte = rd_data[15:8];
                default: cur_byte = rd_data[7:0];
            endcase
        end
    end

    // ------------------------------------------------------------ control
    always @(posedge clk) begin
        tx_start <= 1'b0;
        if (rst) begin
            state     <= S_IDLE;
            wr_ptr    <= {AW{1'b0}};
            trig_ptr  <= {AW{1'b0}};
            rd_ptr    <= {AW{1'b0}};
            count     <= {(AW + 1){1'b0}};
            post_left <= 16'd0;
            tx_data   <= 8'd0;
            in_hdr    <= 1'b0;
            hdr_idx   <= 3'd0;
            byte_idx  <= 3'd0;
            rec_idx   <= {(AW + 1){1'b0}};
        end else if (arm && state != S_DUMP) begin
            state  <= S_ARMED;
            wr_ptr <= {AW{1'b0}};
            count  <= {(AW + 1){1'b0}};
        end else begin
            case (state)
                S_ARMED, S_POST: if (rec_valid) begin
                    wr_ptr <= wr_ptr + 1'b1;
                    if (count != DEPTH) count <= count + 1'b1;
                    if (state == S_ARMED) begin
                        if (hit) begin
                            trig_ptr  <= wr_ptr;
                            post_left <= post_eff;
                            state     <= (post_eff == 0) ? S_FROZEN : S_POST;
                        end
                    end else begin
                        post_left <= post_left - 16'd1;
                        if (post_left == 16'd1) state <= S_FROZEN;
                    end
                end

                S_FROZEN: if (dump) begin
                    state    <= S_DUMP;
                    in_hdr   <= 1'b1;
                    hdr_idx  <= 3'd0;
                    byte_idx <= 3'd0;
                    rec_idx  <= {(AW + 1){1'b0}};
                    rd_ptr   <= oldest;
                end

                S_DUMP: if (!tx_start && tx_ready) begin
                    tx_start <= 1'b1;
                    tx_data  <= cur_byte;
                    if (in_hdr) begin
                        hdr_idx <= hdr_idx + 3'd1;
                        if (hdr_idx == 3'd5) begin
                            in_hdr <= 1'b0;
                            if (count == 0) state <= S_FROZEN;
                        end
                    end else begin
                        byte_idx <= byte_idx + 3'd1;
                        if (byte_idx == 3'd7) begin
                            rd_ptr  <= rd_ptr + 1'b1;
                            rec_idx <= rec_idx + 1'b1;
                            if (rec_idx == count - 1'b1) state <= S_FROZEN;
                        end
                    end
                end

                default: ;
            endcase
        end
    end
endmodule

`default_nettype wire
