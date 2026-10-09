// spi_master.v
// SPI master supporting all 4 modes (CPOL/CPHA configurable per
// transaction) and multi-byte transfers with cs_n held LOW continuously
// across the whole transaction (not toggled per byte) - many real
// peripherals require this for multi-byte commands.
//
// Mode reference:
//   Mode 0: CPOL=0,CPHA=0 - idle low,  sample leading edge, setup trailing
//   Mode 1: CPOL=0,CPHA=1 - idle low,  setup leading edge,  sample trailing
//   Mode 2: CPOL=1,CPHA=0 - idle high, sample leading edge, setup trailing
//   Mode 3: CPOL=1,CPHA=1 - idle high, setup leading edge,  sample trailing
// "Leading" = first transition away from idle after CS asserts;
// "trailing" = the transition back toward idle. This definition makes
// the sample/setup logic uniform regardless of CPOL.
//
// MSB-first (SPI convention).

module spi_master #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer SCLK_FREQ = 1_000_000
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       start,        // pulse HIGH for 1 cycle to begin a transaction
    input  wire       cpol,
    input  wire       cpha,
    input  wire [4:0] num_bytes,    // 1-16 total bytes; cs_n stays low for all of them
    input  wire [7:0] tx_data,      // the FIRST byte to send, loaded on start

    // byte-request handshake (same contract as i2c_master's write path):
    // pulses for one cycle when the NEXT byte's tx data is needed;
    // consumer must present it on wr_data combinationally that cycle.
    output reg        byte_req,
    input  wire [7:0] wr_data,

    output reg        byte_done,    // one-cycle pulse each time a byte's rx_data is ready
    output reg  [7:0] rx_data,      // valid when byte_done pulses
    input  wire       byte_ack,     // consumer pulses this once it has captured rx_data -
                                     // spi_master WAITS here before continuing, so a slow
                                     // consumer (e.g. UART) can never cause a dropped byte

    output reg        busy,
    output reg        done,         // one-cycle pulse when the WHOLE transaction completes

    output reg        sclk,
    output reg        mosi,
    input  wire       miso,
    output reg        cs_n
);

    localparam integer HALF_PERIOD_DIV = CLK_FREQ / (SCLK_FREQ * 2);

    reg [31:0] div_counter;
    wire       tick_pulse;

    localparam [1:0] IDLE          = 2'd0;
    localparam [1:0] TRANSFER      = 2'd1;
    localparam [1:0] BYTE_ACK_WAIT = 2'd2;
    localparam [1:0] DONE_STATE    = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_count;    // 0-7, which bit of the current byte
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [4:0] bytes_left;
    reg       cpol_latched, cpha_latched;  // sampled once at start, held for the whole transaction

    always @(posedge clk or posedge rst) begin
        if (rst)
            div_counter <= 32'd0;
        else if (state != TRANSFER)
            div_counter <= 32'd0;
        else if (tick_pulse)
            div_counter <= 32'd0;
        else
            div_counter <= div_counter + 32'd1;
    end

    assign tick_pulse = (div_counter == HALF_PERIOD_DIV - 1);

    wire new_sclk = ~sclk;                          // value sclk WILL become this tick
    wire leading_edge = (new_sclk != cpol_latched);  // moving away from idle
    wire trailing_edge = (new_sclk == cpol_latched); // returning to idle

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= IDLE;
            sclk         <= 1'b0;
            mosi         <= 1'b0;
            cs_n         <= 1'b1;
            busy         <= 1'b0;
            done         <= 1'b0;
            byte_req     <= 1'b0;
            byte_done    <= 1'b0;
            bit_count    <= 3'd0;
            tx_shift     <= 8'd0;
            rx_shift     <= 8'd0;
            rx_data      <= 8'd0;
            bytes_left   <= 5'd0;
            cpol_latched <= 1'b0;
            cpha_latched <= 1'b0;
        end else begin
            done      <= 1'b0;  // one-cycle pulses, default low each cycle
            byte_req  <= 1'b0;
            byte_done <= 1'b0;

            case (state)

                IDLE: begin
                    cs_n <= 1'b1;
                    if (start) begin
                        cpol_latched <= cpol;
                        cpha_latched <= cpha;
                        sclk         <= cpol;         // idle level for this mode
                        tx_shift     <= tx_data;
                        bytes_left   <= num_bytes;
                        bit_count    <= 3'd0;
                        cs_n         <= 1'b0;
                        busy         <= 1'b1;
                        // CPHA=0 needs the first bit set up BEFORE any
                        // clock edge (no preceding trailing edge exists
                        // yet to do it naturally, unlike subsequent bytes)
                        if (cpha == 1'b0)
                            mosi <= tx_data[7];
                        state <= TRANSFER;
                    end
                end

                TRANSFER: begin
                    if (tick_pulse) begin
                        sclk <= new_sclk;

                        if (leading_edge) begin
                            if (!cpha_latched)
                                rx_shift <= {rx_shift[6:0], miso};       // sample
                            else begin
                                mosi     <= tx_shift[7];                  // setup
                                tx_shift <= {tx_shift[6:0], 1'b0};        // advance to next bit
                            end
                        end else begin
                            // trailing_edge
                            if (!cpha_latched) begin
                                if (bit_count != 3'd7) begin
                                    tx_shift <= {tx_shift[6:0], 1'b0};
                                    mosi     <= tx_shift[6];
                                end
                            end else begin
                                rx_shift <= {rx_shift[6:0], miso};        // sample
                            end

                            if (bit_count == 3'd7) begin
                                // byte complete. For CPHA=1, this trailing
                                // edge is ALSO the last bit's sample edge,
                                // so rx_shift (read here with its OLD,
                                // pre-this-edge value) doesn't yet include
                                // that final bit - use the freshly computed
                                // value directly instead of the stale reg.
                                rx_data   <= cpha_latched ? {rx_shift[6:0], miso} : rx_shift;
                                byte_done <= 1'b1;
                                state     <= BYTE_ACK_WAIT;
                            end else begin
                                bit_count <= bit_count + 3'd1;
                            end
                        end
                    end
                end

                BYTE_ACK_WAIT: begin
                    // Hold here (cs_n/sclk unchanged) until the consumer
                    // confirms it has captured rx_data - this is what
                    // prevents a slow consumer (e.g. UART) from missing
                    // a byte because we raced ahead to the next one.
                    if (byte_ack) begin
                        if (bytes_left == 5'd1) begin
                            state <= DONE_STATE;
                        end else begin
                            byte_req  <= 1'b1;
                            bit_count <= 3'd0;
                            state     <= TRANSFER;
                        end
                    end
                end

                DONE_STATE: begin
                    cs_n  <= 1'b1;
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;

            endcase

            // Latch in the next byte and decrement the counter the cycle
            // byte_req fires (wr_data must be valid combinationally that
            // same cycle, per the handshake contract). For CPHA=0, the
            // new byte's bit0 gets set up naturally on this same edge
            // too, since we're already inside the trailing-edge branch
            // above when byte_req fires - no extra step needed there.
            if (byte_req) begin
                tx_shift   <= wr_data;
                bytes_left <= bytes_left - 5'd1;
                if (!cpha_latched)
                    mosi <= wr_data[7];
            end
        end
    end

endmodule
