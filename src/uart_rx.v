// uart_rx.v
// UART receiver: 8 data bits, 1 stop bit, no parity.
// 16x oversampling, simple (truncating) integer baud divider.
// NOTE: divider truncates rather than rounds -- verify against your
// actual CLK_FREQ/BAUD_RATE pair whether accumulated sampling drift
// stays within tolerance (see testbench for a drift-margin test).

module uart_rx #(
    parameter integer CLK_FREQ   = 100_000_000,
    parameter integer BAUD_RATE  = 115200,
    parameter integer OVERSAMPLE = 16
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx_line,
    output reg  [7:0] rx_data,
    output reg        rx_valid,
    output reg        framing_error
);

    // ---- Tick generator: system clock -> oversample ticks ----
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
    reg [4:0] sample_counter;  // counts oversample ticks within a bit period
    reg [2:0] bit_index;       // counts which of the 8 data bits we're on
    reg [7:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= IDLE;
            sample_counter<= 5'd0;
            bit_index     <= 3'd0;
            shift_reg     <= 8'd0;
            rx_data       <= 8'd0;
            rx_valid      <= 1'b0;
            framing_error <= 1'b0;
        end else begin
            rx_valid      <= 1'b0;  // default: one-cycle pulse
            framing_error <= 1'b0;  // default: one-cycle pulse

            case (state)

                IDLE: begin
                    if (rx_line == 1'b0) begin
                        state          <= START_BIT;
                        sample_counter <= 5'd0;
                    end
                end

                START_BIT: begin
                    if (tick_pulse) begin
                        if (sample_counter == (OVERSAMPLE/2 - 1)) begin
                            if (rx_line == 1'b0) begin
                                state          <= DATA;
                                sample_counter <= 5'd0;
                                bit_index      <= 3'd0;
                            end else begin
                                state <= IDLE;  // glitch, not a real start bit
                            end
                        end else begin
                            sample_counter <= sample_counter + 5'd1;
                        end
                    end
                end

                DATA: begin
                    if (tick_pulse) begin
                        if (sample_counter == (OVERSAMPLE - 1)) begin
                            shift_reg      <= {rx_line, shift_reg[7:1]};
                            sample_counter <= 5'd0;
                            if (bit_index == 3'd7) begin
                                state <= STOP_BIT;
                            end else begin
                                bit_index <= bit_index + 3'd1;
                            end
                        end else begin
                            sample_counter <= sample_counter + 5'd1;
                        end
                    end
                end

                STOP_BIT: begin
                    if (tick_pulse) begin
                        if (sample_counter == (OVERSAMPLE - 1)) begin
                            if (rx_line == 1'b1) begin
                                rx_data  <= shift_reg;
                                rx_valid <= 1'b1;
                            end else begin
                                framing_error <= 1'b1;
                            end
                            state <= IDLE;
                        end else begin
                            sample_counter <= sample_counter + 5'd1;
                        end
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
