// dispatcher.v  (tt_efpga_hub variant)
// CHANGE vs the original: the two 16-byte buffers are now ONE shared buffer of BUF_DEPTH bytes
// (parameter, default 16 = identical behaviour). With BUF_DEPTH < 16 the host must keep every
// transaction (SPI length, I2C write count) <= BUF_DEPTH bytes. Everything else is unchanged.
// Connects uart_rx -> spi_master / i2c_master -> uart_tx.
//
// Command byte format: [7:6]=engine select (00=SPI, 01=I2C)
//   SPI (engine=00):
//     [5]=CPOL, [4]=CPHA, [3:0]=length-1 (total bytes, cs_n held low
//     continuously for all of them). Sequence: command, then `length`
//     data bytes. One result byte streamed back per data byte (SPI is
//     full-duplex).
//   I2C (engine=01):
//     [5]=R/W, [4]=reserved, [3:0]=length-1.
//     R/W=0 (write): length = write-data-byte count. Sequence: command,
//       device_addr, then `length` write data bytes. One status byte
//       (0x00=success, 0xFF=ack_error) returned at the end.
//     R/W=1 (read-capable): length = READ-byte count. Sequence: command,
//       device_addr, write_count_byte (0-15; 0 = pure read, no write
//       phase), then write_count data bytes (if >0). This supports
//       write-only, read-only, and combined write-then-read (register-
//       read pattern, via repeated-START) depending on write_count and
//       length. Each read byte is streamed back over UART as it
//       arrives, followed by one final status byte.

module dispatcher #(
    parameter integer BUF_DEPTH = 16  // bytes in the shared SPI/I2C data buffer (power of two, 2..16)
) (
    input  wire       clk,
    input  wire       rst,

    // ---- from uart_rx ----
    input  wire [7:0] rx_data,
    input  wire       rx_valid,

    // ---- to/from uart_tx ----
    output reg  [7:0] uart_tx_data,
    output reg        uart_tx_start,
    input  wire       uart_tx_busy,

    // ---- to/from spi_master (v2: CPOL/CPHA, continuous multi-byte) ----
    output reg        spi_start,
    output reg        spi_cpol,
    output reg        spi_cpha,
    output reg  [4:0] spi_num_bytes,
    output reg  [7:0] spi_tx_data,
    input  wire       spi_byte_req,
    output reg  [7:0] spi_wr_data,
    input  wire       spi_byte_done,
    input  wire [7:0] spi_rx_data,
    output reg        spi_byte_ack,
    input  wire       spi_done,

    // ---- to/from i2c_master (v2: read + repeated-START) ----
    output reg        i2c_start,
    output reg  [6:0] i2c_dev_addr,
    output reg  [4:0] i2c_num_write_bytes,
    output reg  [4:0] i2c_num_read_bytes,
    input  wire       i2c_byte_req,
    output reg  [7:0] i2c_wr_data,
    input  wire       i2c_read_byte_valid,
    input  wire [7:0] i2c_read_byte_data,
    input  wire       i2c_done,
    input  wire       i2c_ack_error
);

    localparam [4:0] IDLE               = 5'd0;
    localparam [4:0] WAIT_SPI_DATA      = 5'd1;
    localparam [4:0] SPI_START_STATE    = 5'd2;
    localparam [4:0] SPI_WAIT_STATE     = 5'd3;
    localparam [4:0] SEND_SPI_RESULT    = 5'd4;
    localparam [4:0] WAIT_I2C_ADDR      = 5'd5;
    localparam [4:0] WAIT_I2C_WCOUNT    = 5'd6;
    localparam [4:0] WAIT_I2C_WDATA     = 5'd7;
    localparam [4:0] I2C_START_STATE    = 5'd8;
    localparam [4:0] I2C_WAIT_STATE     = 5'd9;
    localparam [4:0] SEND_I2C_READ_BYTE = 5'd10;
    localparam [4:0] SEND_I2C_STATUS    = 5'd11;

    reg [4:0] state;
    reg       rw;
    reg [4:0] length_field;   // command byte's [3:0]+1, meaning depends on engine/R-W

    // SPI path registers
    // One buffer shared by SPI and I2C (the single FSM never runs both at once).
    localparam integer BIDX = $clog2(BUF_DEPTH);   // index width needed for BUF_DEPTH entries
    reg [7:0] hub_buffer [0:BUF_DEPTH-1];
    reg [4:0] spi_buf_index;
    reg [4:0] spi_req_index;
    reg [4:0] spi_total_bytes;
    reg [4:0] spi_results_sent;
    reg [7:0] spi_result_byte;

    // I2C path registers
    reg [4:0] i2c_buf_index;
    reg [4:0] i2c_req_index;
    reg [4:0] i2c_write_count;
    reg [7:0] i2c_status_byte;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state               <= IDLE;
            rw                  <= 1'b0;
            length_field        <= 5'd0;
            uart_tx_data        <= 8'd0;
            uart_tx_start       <= 1'b0;
            spi_start           <= 1'b0;
            spi_cpol            <= 1'b0;
            spi_cpha            <= 1'b0;
            spi_num_bytes       <= 5'd0;
            spi_tx_data         <= 8'd0;
            spi_byte_ack        <= 1'b0;
            spi_buf_index       <= 5'd0;
            spi_req_index       <= 5'd0;
            spi_total_bytes     <= 5'd0;
            spi_results_sent    <= 5'd0;
            spi_result_byte     <= 8'd0;
            i2c_start           <= 1'b0;
            i2c_dev_addr        <= 7'd0;
            i2c_num_write_bytes <= 5'd0;
            i2c_num_read_bytes  <= 5'd0;
            i2c_buf_index       <= 5'd0;
            i2c_req_index       <= 5'd0;
            i2c_write_count     <= 5'd0;
            i2c_status_byte     <= 8'd0;
        end else begin
            uart_tx_start <= 1'b0;
            spi_start     <= 1'b0;
            i2c_start     <= 1'b0;
            spi_byte_ack  <= 1'b0;

            // Serve byte_req handshakes whenever they fire, regardless of
            // nominal state - both happen concurrently while waiting.
            if (spi_byte_req)
                spi_req_index <= spi_req_index + 5'd1;
            if (i2c_byte_req)
                i2c_req_index <= i2c_req_index + 5'd1;

            case (state)

                IDLE: begin
                    if (rx_valid) begin
                        rw           <= rx_data[5];
                        length_field <= rx_data[3:0] + 5'd1;
                        if (rx_data[7:6] == 2'b00) begin
                            // SPI
                            spi_cpol        <= rx_data[5];
                            spi_cpha        <= rx_data[4];
                            spi_total_bytes <= rx_data[3:0] + 5'd1;
                            spi_buf_index   <= 5'd0;
                            state           <= WAIT_SPI_DATA;
                        end else if (rx_data[7:6] == 2'b01) begin
                            // I2C
                            state <= WAIT_I2C_ADDR;
                        end
                    end
                end

                // ---------------- SPI path ----------------

                WAIT_SPI_DATA: begin
                    if (rx_valid) begin
                        hub_buffer[spi_buf_index[BIDX-1:0]] <= rx_data;
                        if (spi_buf_index == spi_total_bytes - 5'd1) begin
                            state <= SPI_START_STATE;
                        end else begin
                            spi_buf_index <= spi_buf_index + 5'd1;
                        end
                    end
                end

                SPI_START_STATE: begin
                    spi_start        <= 1'b1;
                    spi_tx_data      <= hub_buffer[0];
                    spi_num_bytes    <= spi_total_bytes;
                    spi_req_index    <= 5'd1;   // buffer[0] already consumed directly
                    spi_results_sent <= 5'd0;
                    state            <= SPI_WAIT_STATE;
                end

                SPI_WAIT_STATE: begin
                    if (spi_byte_done) begin
                        spi_result_byte <= spi_rx_data;
                        state           <= SEND_SPI_RESULT;
                    end
                end

                SEND_SPI_RESULT: begin
                    if (!uart_tx_busy) begin
                        uart_tx_data     <= spi_result_byte;
                        uart_tx_start    <= 1'b1;
                        spi_byte_ack     <= 1'b1;   // only NOW tell spi_master to continue -
                                                      // this is what throttles it to UART's pace
                        spi_results_sent <= spi_results_sent + 5'd1;
                        if (spi_results_sent + 5'd1 == spi_total_bytes) begin
                            state <= IDLE;
                        end else begin
                            state <= SPI_WAIT_STATE;
                        end
                    end
                end

                // ---------------- I2C path ----------------

                WAIT_I2C_ADDR: begin
                    if (rx_valid) begin
                        i2c_dev_addr <= rx_data[6:0];
                        if (rw == 1'b0) begin
                            // write-only, length_field = write count
                            i2c_num_write_bytes <= length_field;
                            i2c_num_read_bytes  <= 5'd0;
                            i2c_buf_index       <= 5'd0;
                            if (length_field == 5'd0)
                                state <= I2C_START_STATE;
                            else
                                state <= WAIT_I2C_WDATA;
                        end else begin
                            // read-capable: length_field = read count,
                            // need the extra write_count header byte next
                            i2c_num_read_bytes <= length_field;
                            state <= WAIT_I2C_WCOUNT;
                        end
                    end
                end

                WAIT_I2C_WCOUNT: begin
                    if (rx_valid) begin
                        i2c_write_count     <= rx_data[4:0];
                        i2c_num_write_bytes <= rx_data[4:0];
                        i2c_buf_index       <= 5'd0;
                        if (rx_data[4:0] == 5'd0)
                            state <= I2C_START_STATE;
                        else
                            state <= WAIT_I2C_WDATA;
                    end
                end

                WAIT_I2C_WDATA: begin
                    if (rx_valid) begin
                        hub_buffer[i2c_buf_index[BIDX-1:0]] <= rx_data;
                        if (i2c_buf_index == i2c_num_write_bytes - 5'd1) begin
                            state <= I2C_START_STATE;
                        end else begin
                            i2c_buf_index <= i2c_buf_index + 5'd1;
                        end
                    end
                end

                I2C_START_STATE: begin
                    i2c_start     <= 1'b1;
                    i2c_req_index <= 5'd0;
                    state         <= I2C_WAIT_STATE;
                end

                I2C_WAIT_STATE: begin
                    if (i2c_read_byte_valid) begin
                        i2c_status_byte <= i2c_read_byte_data;  // reuse as temp holder
                        state           <= SEND_I2C_READ_BYTE;
                    end else if (i2c_done) begin
                        i2c_status_byte <= i2c_ack_error ? 8'hFF : 8'h00;
                        state           <= SEND_I2C_STATUS;
                    end
                end

                SEND_I2C_READ_BYTE: begin
                    if (!uart_tx_busy) begin
                        uart_tx_data  <= i2c_status_byte;  // holds the read byte here
                        uart_tx_start <= 1'b1;
                        state         <= I2C_WAIT_STATE;
                    end
                end

                SEND_I2C_STATUS: begin
                    if (!uart_tx_busy) begin
                        uart_tx_data  <= i2c_status_byte;
                        uart_tx_start <= 1'b1;
                        state         <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

    // Combinational: present whatever byte each engine has most recently
    // requested. Must be combinational per both modules' handshake
    // contracts (they latch wr_data the SAME cycle byte_req is high).
    always @(*) begin
        spi_wr_data = hub_buffer[spi_req_index[BIDX-1:0]];
        i2c_wr_data = hub_buffer[i2c_req_index[BIDX-1:0]];
    end

endmodule
