// i2c_master.v
// I2C master supporting WRITE, READ, and combined WRITE-then-READ
// (via repeated-START) transactions - covers the common "write a
// register address, then read back its value" pattern (e.g. MPU6050).
//
// num_write_bytes / num_read_bytes: either can be 0.
//   write>0, read==0        -> plain write transaction
//   write==0, read>0        -> plain read transaction (address+R first)
//   write>0, read>0         -> write phase, then REPEATED START, then read phase
//   write==0, read==0       -> address-only probe (address+W, ACK-check, STOP)
//
// Master-driven ACK/NACK during reads: ACKs every received byte except
// the last one, which gets NACKed - this tells the slave "stop sending,
// I'm done" per the I2C spec.
//
// Open-drain modeling: sda_oe/scl_oe = 1 means "actively pull LOW",
// 0 means "released" (pulled HIGH by the bus's pull-up resistors, or
// driven HIGH by another device). The actual bidirectional wire is
// created at the TOP LEVEL, not here - this module only ever decides
// oe/driven-value, never assigns 'z' itself.

module i2c_master #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer SCL_FREQ = 100_000       // standard mode I2C
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       start,             // pulse to begin a transaction
    input  wire [6:0] dev_addr,
    input  wire [4:0] num_write_bytes,   // 0-16
    input  wire [4:0] num_read_bytes,    // 0-16

    // write-byte-request handshake (unchanged from v1): pulses for one
    // cycle when the NEXT write byte is needed; consumer must present
    // it on wr_data combinationally that same cycle.
    output reg        byte_req,
    input  wire [7:0] wr_data,

    // read-byte-ready: pulses for one cycle each time a received byte
    // is ready; consumer must capture read_byte_data that same cycle.
    output reg        read_byte_valid,
    output reg  [7:0] read_byte_data,

    output reg        busy,
    output reg        done,              // one-cycle pulse, transaction complete
    output reg        ack_error,         // latched HIGH if any slave NACKed unexpectedly

    // ---- physical I2C bus (open-drain, wired at top level) ----
    output reg        sda_oe,
    input  wire       sda_in,
    output reg        scl_oe
);

    // ---- Tick generator: SCL half-period ----
    localparam integer HALF_PERIOD_DIV = CLK_FREQ / (SCL_FREQ * 2);

    reg [31:0] div_counter;
    wire       tick_pulse;

    localparam [4:0] IDLE            = 5'd0;
    localparam [4:0] START_A         = 5'd1;   // SDA falls while SCL still high
    localparam [4:0] START_B         = 5'd2;   // then SCL falls, ready for first bit
    localparam [4:0] DATA_SETUP      = 5'd3;   // SCL low, set up next bit on SDA
    localparam [4:0] DATA_HOLD       = 5'd4;   // SCL high, bit is being read by slave
    localparam [4:0] ACK_SETUP       = 5'd5;   // SCL low, release SDA for slave to drive
    localparam [4:0] ACK_SAMPLE      = 5'd6;   // SCL high, sample ACK/NACK
    localparam [4:0] READ_SETUP      = 5'd7;   // SCL low, release SDA so slave can drive a bit
    localparam [4:0] READ_HOLD       = 5'd8;   // SCL high, sample the bit
    localparam [4:0] MASTER_ACK_SETUP= 5'd9;   // SCL low, master drives ACK(0)/NACK(1)
    localparam [4:0] MASTER_ACK_HOLD = 5'd10;  // SCL high, hold that ack/nack bit
    localparam [4:0] RSTART_PREP     = 5'd11;  // SCL low, release SDA (let it go high)
    localparam [4:0] RSTART_SCLHIGH  = 5'd12;  // release SCL -> goes high, SDA still high
    localparam [4:0] RSTART_SDALOW   = 5'd13;  // pull SDA low while SCL high - the repeated START
    localparam [4:0] RSTART_SCLLOW   = 5'd14;  // pull SCL low again, ready for address+R
    localparam [4:0] STOP_A          = 5'd15;  // SCL low, SDA driven low
    localparam [4:0] STOP_B          = 5'd16;  // SCL rises
    localparam [4:0] STOP_C          = 5'd17;  // SDA released (rises) -> STOP condition
    localparam [4:0] DONE_STATE      = 5'd18;

    reg [4:0] state;
    reg [2:0] bit_index;        // 0-7, which bit of the current byte
    reg [7:0] shift_reg;        // byte being shifted OUT (address or write data)
    reg [7:0] rx_shift;         // byte being shifted IN (read data)
    reg [4:0] write_bytes_left; // counts down write DATA bytes (not the address byte)
    reg [4:0] read_bytes_left;  // counts down read DATA bytes
    reg       phase_read_addr;  // 1 = the byte we just sent was address+R (about to read);
                                 // 0 = it was address+W or a write-data byte

    // Divider only runs while actively bit-banging (not in IDLE/DONE)
    always @(posedge clk or posedge rst) begin
        if (rst)
            div_counter <= 32'd0;
        else if (state == IDLE || state == DONE_STATE)
            div_counter <= 32'd0;
        else if (tick_pulse)
            div_counter <= 32'd0;
        else
            div_counter <= div_counter + 32'd1;
    end

    assign tick_pulse = (div_counter == HALF_PERIOD_DIV - 1);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= IDLE;
            sda_oe           <= 1'b0;
            scl_oe           <= 1'b0;
            busy             <= 1'b0;
            done             <= 1'b0;
            ack_error        <= 1'b0;
            byte_req         <= 1'b0;
            read_byte_valid  <= 1'b0;
            read_byte_data   <= 8'd0;
            bit_index        <= 3'd0;
            shift_reg        <= 8'd0;
            rx_shift         <= 8'd0;
            write_bytes_left <= 5'd0;
            read_bytes_left  <= 5'd0;
            phase_read_addr  <= 1'b0;
        end else begin
            done            <= 1'b0;  // one-cycle pulses, default low each cycle
            byte_req        <= 1'b0;
            read_byte_valid <= 1'b0;

            case (state)

                IDLE: begin
                    sda_oe <= 1'b0;
                    scl_oe <= 1'b0;
                    if (start) begin
                        write_bytes_left <= num_write_bytes;
                        read_bytes_left  <= num_read_bytes;
                        busy             <= 1'b1;
                        ack_error        <= 1'b0;
                        if (num_write_bytes > 5'd0 || num_read_bytes == 5'd0) begin
                            // write transaction, or address-only probe
                            shift_reg       <= {dev_addr, 1'b0};
                            phase_read_addr <= 1'b0;
                        end else begin
                            // read-only transaction: address+R straight away
                            shift_reg       <= {dev_addr, 1'b1};
                            phase_read_addr <= 1'b1;
                        end
                        state <= START_A;
                    end
                end

                START_A: begin
                    sda_oe <= 1'b1;   // pull SDA low - the START condition
                    if (tick_pulse)
                        state <= START_B;
                end

                START_B: begin
                    scl_oe <= 1'b1;   // pull SCL low too, ready to send bits
                    if (tick_pulse) begin
                        bit_index <= 3'd7;
                        state     <= DATA_SETUP;
                    end
                end

                DATA_SETUP: begin
                    sda_oe <= shift_reg[bit_index] ? 1'b0 : 1'b1;
                    if (tick_pulse) begin
                        scl_oe <= 1'b0;
                        state  <= DATA_HOLD;
                    end
                end

                DATA_HOLD: begin
                    if (tick_pulse) begin
                        scl_oe <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            state     <= DATA_SETUP;
                        end
                    end
                end

                ACK_SETUP: begin
                    sda_oe <= 1'b0;   // release SDA so the slave can drive ACK/NACK
                    if (tick_pulse) begin
                        scl_oe <= 1'b0;
                        state  <= ACK_SAMPLE;
                    end
                end

                ACK_SAMPLE: begin
                    if (tick_pulse) begin
                        if (sda_in == 1'b1)
                            ack_error <= 1'b1;
                        scl_oe <= 1'b1;

                        if (phase_read_addr) begin
                            // just sent address+R - always move into reading
                            bit_index <= 3'd7;
                            state     <= READ_SETUP;
                        end else begin
                            if (write_bytes_left == 5'd0) begin
                                if (read_bytes_left > 5'd0) begin
                                    state <= RSTART_PREP;
                                end else begin
                                    state <= STOP_A;
                                end
                            end else begin
                                byte_req  <= 1'b1;
                                bit_index <= 3'd7;
                                state     <= DATA_SETUP;
                            end
                        end
                    end
                end

                // ---------------- Repeated START (write -> read switch) ----------------

                RSTART_PREP: begin
                    sda_oe <= 1'b0;   // release SDA, let it float high (SCL already low)
                    if (tick_pulse)
                        state <= RSTART_SCLHIGH;
                end

                RSTART_SCLHIGH: begin
                    scl_oe <= 1'b0;   // release SCL -> rises (SDA still high)
                    if (tick_pulse)
                        state <= RSTART_SDALOW;
                end

                RSTART_SDALOW: begin
                    sda_oe <= 1'b1;   // pull SDA low while SCL still high - repeated START
                    if (tick_pulse)
                        state <= RSTART_SCLLOW;
                end

                RSTART_SCLLOW: begin
                    scl_oe <= 1'b1;   // pull SCL low again, ready for address+R
                    if (tick_pulse) begin
                        shift_reg       <= {dev_addr, 1'b1};
                        phase_read_addr <= 1'b1;
                        bit_index       <= 3'd7;
                        state           <= DATA_SETUP;
                    end
                end

                // ---------------- Read data bytes ----------------

                READ_SETUP: begin
                    sda_oe <= 1'b0;   // release SDA so the slave can drive a bit
                    if (tick_pulse) begin
                        scl_oe <= 1'b0;
                        state  <= READ_HOLD;
                    end
                end

                READ_HOLD: begin
                    if (tick_pulse) begin
                        rx_shift <= {rx_shift[6:0], sda_in};
                        scl_oe   <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= MASTER_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            state     <= READ_SETUP;
                        end
                    end
                end

                MASTER_ACK_SETUP: begin
                    // NACK (release, =1) on the last byte; ACK (pull low, =1 meaning drive)
                    // on every byte before that
                    sda_oe <= (read_bytes_left == 5'd1) ? 1'b0 : 1'b1;
                    if (tick_pulse) begin
                        scl_oe          <= 1'b0;
                        read_byte_valid <= 1'b1;
                        read_byte_data  <= rx_shift;
                        read_bytes_left <= read_bytes_left - 5'd1;
                        state           <= MASTER_ACK_HOLD;
                    end
                end

                MASTER_ACK_HOLD: begin
                    if (tick_pulse) begin
                        scl_oe <= 1'b1;
                        if (read_bytes_left == 5'd0) begin
                            state <= STOP_A;
                        end else begin
                            bit_index <= 3'd7;
                            state     <= READ_SETUP;
                        end
                    end
                end

                // ---------------- STOP ----------------

                STOP_A: begin
                    sda_oe <= 1'b1;
                    if (tick_pulse)
                        state <= STOP_B;
                end

                STOP_B: begin
                    scl_oe <= 1'b0;
                    if (tick_pulse)
                        state <= STOP_C;
                end

                STOP_C: begin
                    sda_oe <= 1'b0;
                    if (tick_pulse)
                        state <= DONE_STATE;
                end

                DONE_STATE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;

            endcase

            // Latch in the next WRITE byte and decrement the counter the
            // cycle byte_req fires (wr_data must be valid combinationally
            // on that same cycle, per the handshake contract)
            if (byte_req) begin
                shift_reg        <= wr_data;
                write_bytes_left <= write_bytes_left - 5'd1;
            end
        end
    end

endmodule
