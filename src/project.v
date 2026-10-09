// project.v
// Tiny Tapeout top: UART/SPI/I2C peripheral hub (uart_rx -> dispatcher -> spi/i2c -> uart_tx)
// next to a small eFPGA that watches the hub's events and the input pins.
//
// Pin map:
//   ui_in[0]  uart rx (host -> chip)       ui_in[1]  spi miso       ui_in[7:2]  fabric inputs
//   uo_out[0] uart tx (chip -> host)       uo_out[7:1] fabric outputs (or shift register byte)
//   uio[0] cfg_data in   uio[1] cfg_clk in   uio[2] cfg_out (read-back)
//   uio[3] spi mosi      uio[4] spi sclk     uio[5] spi cs_n
//   uio[6] i2c scl (open drain)              uio[7] i2c sda (open drain, bidirectional)

`default_nettype none

module tt_um_ayush130305_efpga_hub (
    input  wire [7:0] ui_in,   // dedicated inputs
    output wire [7:0] uo_out,  // dedicated outputs
    input  wire [7:0] uio_in,  // bidirectional pins, input side
    output wire [7:0] uio_out, // bidirectional pins, output side
    output wire [7:0] uio_oe,  // bidirectional pins, output enables (1 = output)
    input  wire       ena,     // goes high when the design is selected
    input  wire       clk,     // system clock
    input  wire       rst_n    // active-low reset
);
    // Hub timing parameters, matched to clock_hz in info.yaml.
    localparam integer CLK_FREQ  = 25_000_000;
    localparam integer BAUD_RATE = 115_200;
    localparam integer SCLK_FREQ = 1_000_000;
    localparam integer SCL_FREQ  = 100_000;
    // Shared SPI/I2C buffer depth in bytes (16 = full protocol, see README).
    localparam integer BUF_DEPTH = 16;
    // Fabric size.
    localparam integer NC        = 16;

    // The hub blocks use an active-high reset.
    wire rst = ~rst_n;

    // ---- hub wires ----
    wire [7:0] rx_data;
    wire       rx_valid;
    wire [7:0] tx_data;
    wire       tx_start, tx_busy, tx_line;

    wire       spi_start, spi_cpol, spi_cpha, spi_byte_req, spi_byte_done, spi_byte_ack, spi_done;
    wire [4:0] spi_num_bytes;
    wire [7:0] spi_tx_data, spi_wr_data, spi_rx_data;
    wire       sclk, mosi, cs_n;

    wire       i2c_start, i2c_byte_req, i2c_read_byte_valid, i2c_done, i2c_ack_error;
    wire [6:0] i2c_dev_addr;
    wire [4:0] i2c_num_write_bytes, i2c_num_read_bytes;
    wire [7:0] i2c_wr_data, i2c_read_byte_data;
    wire       sda_oe, scl_oe;

    // Receiver on ui_in[0].
    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_rx (
        .clk(clk), .rst(rst), .rx_line(ui_in[0]),
        .rx_data(rx_data), .rx_valid(rx_valid), .framing_error()
    );

    // Command dispatcher.
    dispatcher #(.BUF_DEPTH(BUF_DEPTH)) u_disp (
        .clk(clk), .rst(rst),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .uart_tx_data(tx_data), .uart_tx_start(tx_start), .uart_tx_busy(tx_busy),
        .spi_start(spi_start), .spi_cpol(spi_cpol), .spi_cpha(spi_cpha), .spi_num_bytes(spi_num_bytes),
        .spi_tx_data(spi_tx_data), .spi_byte_req(spi_byte_req), .spi_wr_data(spi_wr_data),
        .spi_byte_done(spi_byte_done), .spi_rx_data(spi_rx_data), .spi_byte_ack(spi_byte_ack), .spi_done(spi_done),
        .i2c_start(i2c_start), .i2c_dev_addr(i2c_dev_addr),
        .i2c_num_write_bytes(i2c_num_write_bytes), .i2c_num_read_bytes(i2c_num_read_bytes),
        .i2c_byte_req(i2c_byte_req), .i2c_wr_data(i2c_wr_data),
        .i2c_read_byte_valid(i2c_read_byte_valid), .i2c_read_byte_data(i2c_read_byte_data),
        .i2c_done(i2c_done), .i2c_ack_error(i2c_ack_error)
    );

    // Transmitter to uo_out[0].
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx (
        .clk(clk), .rst(rst), .tx_start(tx_start), .tx_data(tx_data),
        .tx_line(tx_line), .tx_busy(tx_busy)
    );

    // SPI master, miso on ui_in[1].
    spi_master #(.CLK_FREQ(CLK_FREQ), .SCLK_FREQ(SCLK_FREQ)) u_spi (
        .clk(clk), .rst(rst), .start(spi_start), .cpol(spi_cpol), .cpha(spi_cpha),
        .num_bytes(spi_num_bytes), .tx_data(spi_tx_data),
        .byte_req(spi_byte_req), .wr_data(spi_wr_data),
        .byte_done(spi_byte_done), .rx_data(spi_rx_data), .byte_ack(spi_byte_ack),
        .busy(), .done(spi_done),
        .sclk(sclk), .mosi(mosi), .miso(ui_in[1]), .cs_n(cs_n)
    );

    // I2C master, sda_in from uio_in[7].
    i2c_master #(.CLK_FREQ(CLK_FREQ), .SCL_FREQ(SCL_FREQ)) u_i2c (
        .clk(clk), .rst(rst), .start(i2c_start), .dev_addr(i2c_dev_addr),
        .num_write_bytes(i2c_num_write_bytes), .num_read_bytes(i2c_num_read_bytes),
        .byte_req(i2c_byte_req), .wr_data(i2c_wr_data),
        .read_byte_valid(i2c_read_byte_valid), .read_byte_data(i2c_read_byte_data),
        .busy(), .done(i2c_done), .ack_error(i2c_ack_error),
        .sda_oe(sda_oe), .sda_in(uio_in[7]), .scl_oe(scl_oe)
    );

    // ---- eFPGA ----
    wire [6:0] fab_out, sh_par;
    wire       uo_mode, cfg_out;

    efpga_core #(.NC(NC), .W(8), .WO(8), .CFG_MODE(0)) u_fpga (
        .clk(clk), .rst_n(rst_n),
        .ui_in(ui_in),
        .hub_ev({tx_start, i2c_done, spi_done, rx_valid}),
        .cfg_data(uio_in[0]), .cfg_clk(uio_in[1]), .cfg_out(cfg_out),
        .fab_out(fab_out), .sh_par_out(sh_par), .uo_mode(uo_mode)
    );

    // Dedicated outputs: uart tx, then fabric (or shift register byte) on [7:1].
    assign uo_out  = {uo_mode ? sh_par : fab_out, tx_line};
    // Bidirectional outputs: open-drain lines drive 0 and are released by the enable.
    assign uio_out = {1'b0, 1'b0, cs_n, sclk, mosi, cfg_out, 2'b00};
    assign uio_oe  = {sda_oe, scl_oe, 3'b111, 1'b1, 2'b00};

    // Unused inputs.
    wire _unused = &{ena, uio_in[6:2], 1'b0};
endmodule

`default_nettype wire
