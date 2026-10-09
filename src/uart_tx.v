// uart_tx.v
// UART transmitter: 8 data bits, 1 stop bit, no parity.
// Unlike uart_rx, this module only DRIVES the line - it never samples
// anything, so there is no midpoint-check / glitch-rejection logic here.
// Uses the same tick-generator approach as uart_rx for consistent timing.

module uart_tx #(
    parameter integer CLK_FREQ   = 100_000_000,
    parameter integer BAUD_RATE  = 115200,
    parameter integer OVERSAMPLE = 16
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,   // pulse HIGH for 1 cycle to begin sending tx_data
    input  wire [7:0] tx_data,
    output reg        tx_line,
    output reg        tx_busy
);

    // ---- Tick generator: identical structure to uart_rx's ----
    localparam integer DIVISOR = CLK_FREQ / (BAUD_RATE * OVERSAMPLE);

    reg [15:0] tick_counter;
    wire       tick_pulse;

    always @(posedge clk or posedge rst) begin
        if (rst)
            tick_counter <= 16'd0;
        else if (tick_counter == DIVISOR - 1)
            tick_counter <= 16'd0;
        else
            tick_counter <= tick_counter + 16'd1;
    end

    assign tick_pulse = (tick_counter == DIVISOR - 1);

    // ---- FSM ----
    localparam [1:0] IDLE      = 2'd0;
    localparam [1:0] START_BIT = 2'd1;
    localparam [1:0] DATA      = 2'd2;
    localparam [1:0] STOP_BIT  = 2'd3;

    reg [1:0] state;
    reg [4:0] bit_timer;   // counts ticks within the current bit period
    reg [2:0] bit_index;   // which of the 8 data bits is currently on the line
    reg [7:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            tx_line   <= 1'b1;   // idle HIGH
            tx_busy   <= 1'b0;
            bit_timer <= 5'd0;
            bit_index <= 3'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)

                IDLE: begin
                    tx_line <= 1'b1;
                    if (tx_start && !tx_busy) begin
                        shift_reg <= tx_data;
                        tx_busy   <= 1'b1;
                        bit_timer <= 5'd0;
                        state     <= START_BIT;
                    end
                end

                START_BIT: begin
                    tx_line <= 1'b0;   // start bit is always LOW
                    if (tick_pulse) begin
                        if (bit_timer == (OVERSAMPLE - 1)) begin
                            bit_timer <= 5'd0;
                            bit_index <= 3'd0;
                            state     <= DATA;
                        end else begin
                            bit_timer <= bit_timer + 5'd1;
                        end
                    end
                end

                DATA: begin
                    tx_line <= shift_reg[bit_index];  // LSB first
                    if (tick_pulse) begin
                        if (bit_timer == (OVERSAMPLE - 1)) begin
                            bit_timer <= 5'd0;
                            if (bit_index == 3'd7) begin
                                state <= STOP_BIT;
                            end else begin
                                bit_index <= bit_index + 3'd1;
                            end
                        end else begin
                            bit_timer <= bit_timer + 5'd1;
                        end
                    end
                end

                STOP_BIT: begin
                    tx_line <= 1'b1;   // stop bit is always HIGH
                    if (tick_pulse) begin
                        if (bit_timer == (OVERSAMPLE - 1)) begin
                            bit_timer <= 5'd0;
                            tx_busy   <= 1'b0;
                            state     <= IDLE;
                        end else begin
                            bit_timer <= bit_timer + 5'd1;
                        end
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
